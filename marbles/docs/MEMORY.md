# Grids Mode Implementation Summary

This document records what was built for the Grids port into Marbles' T-section.
See `docs/plan.md` for the full design rationale.

---

## Model Index

`T_GENERATOR_MODEL_GRIDS = 5` in `random/t_generator.h`.
Slot formerly named `T_GENERATOR_MODEL_THREE_STATES` was repurposed.
Model 6 (`T_GENERATOR_MODEL_MARKOV`) remains unreachable in normal use.

---

## Grids Engine (`marbles/grids/`)

Files ported from the original Grids firmware with AVR/avrlib dependencies removed:

| File | Role |
|---|---|
| `pattern_generator.h / .cc` | Core pattern engine (drums map + Euclidean) |
| `grids_resources.h / .cc` | Drum map lookup tables, Euclidean LUT |
| `grids_random.h / .cc` | 16-bit LFSR used for pattern perturbation |

`PatternGenerator` is used as a fully static class. It is initialised once in
`TGenerator::Init()` with `PatternGenerator::Init(0)` and reset via
`PatternGenerator::Reset()` on t-section reset events.

`makefile` — `marbles/grids` is included in the `PACKAGES` list.

---

## T-Generator Changes (`random/t_generator.h / .cc`)

### New private fields

```cpp
float   grids_bd_density_;          // BD density (base 0.5, CV offset)
float   grids_sd_density_;          // SD density
float   grids_hh_density_;          // HH density
float   grids_chaos_;               // Drums randomness (Deja Vu knob)
bool    grids_euclidean_;           // Sub-mode: false = Drums, true = Euclidean
int     grids_euclidean_length_;    // Shared Euclidean step count (1–16)
uint8_t grids_pulse_;               // Counter 0..(kPulsesPerStep*2-1) for X ramp
bool    master_gate_;               // Overrides t2 gate output in Grids mode
```

### New setters

```cpp
set_grids_bd_density(float)
set_grids_sd_density(float)
set_grids_hh_density(float)
set_grids_chaos(float)
set_grids_euclidean(bool)
set_grids_euclidean_length(int)   // 1–16, driven by the LENGTH knob
```

### `Init()`

Initialises all new fields. Calls `PatternGenerator::Init(0)` and
`PatternGenerator::set_output_mode(OUTPUT_MODE_DRUMS)`.

### `Process()` — settings block

Runs before the sample loop. When `model_ == T_GENERATOR_MODEL_GRIDS`:

- Sets `OUTPUT_MODE_EUCLIDEAN` or `OUTPUT_MODE_DRUMS` on the pattern generator.
- **Drums**: writes `bias_ * 255` → `drums.x`, `jitter_ * 255` → `drums.y`,
  `grids_chaos_ * 255` → `drums.randomness`.
- **Euclidean**: encodes `grids_euclidean_length_` as
  `(length - 1) * 8` → all three `euclidean_length[]` channels (shared length).
- Both branches write the three density bytes.

### `Process()` — sample loop

On every `master_phase_` wrap:
- `jitter_amount` is forced to `0.0f` in Grids mode so the JITTER knob never
  introduces timing swing to any output.
- `grids_pulse_` is incremented modulo `kPulsesPerStep * 2` (= 6). This counter
  drives the step-level ramp written to `ramps.master`.
- `PatternGenerator::TickClock(1)` advances the pattern clock.
- `ConfigureSlaveRamps` is called, which calls `GenerateGrids`.

`ramps.master` in Grids mode holds a step-level ramp instead of raw
`master_phase_`:

```cpp
(static_cast<float>(grids_pulse_) + master_phase_) / static_cast<float>(kPulsesPerStep * 2)
```

This ramp cycles 0→1 over 6 master wraps (2 Grids steps = 8th-note rate),
giving the X section a steady musical clock that is decoupled from the BD/SD/HH
gate patterns. `grids_pulse_` is reset to 0 on every t-section reset.

### `GenerateGrids()`

```cpp
uint8_t state = PatternGenerator::state();
master_gate_ = state & 0x02;          // SD → t2
return (state & 0x01) | ((state & 0x04) >> 1);  // BD → t1, HH → t3
```

### `master_gate` output

In `Process()`, when `model_ == T_GENERATOR_MODEL_GRIDS`, the per-sample
`*master_gate` is taken from `master_gate_` instead of `master_phase_ < 0.5f`.

---

## Control Mapping (`marbles.cc`)

### T-section (Grids mode only)

| Marbles control | Grids parameter |
|---|---|
| RATE knob | Tempo / clock division |
| BIAS knob | Drums map X coordinate |
| JITTER knob | Drums map Y coordinate |
| RATE CV | BD density offset (base 0.5) |
| BIAS CV | SD density offset (base 0.5) |
| JITTER CV | HH density offset (base 0.5) |
| Deja Vu knob + CV | Drums randomness / chaos (Drums sub-mode only) |
| LENGTH knob | Euclidean step count 1–16 (Euclidean sub-mode only) |
| Deja Vu button | Sub-mode: OFF = Drums, ON/LOCKED = Euclidean |

Density formula: `clamp(0.5 + cv / 120.0f, 0, 1)` for BD (RATE CV scale);
`clamp(0.5 + cv, 0, 1)` for SD and HH.

`set_deja_vu()` and `set_length()` are **not** called for the T section when
Grids mode is active — the Deja Vu state is consumed as a sub-mode selector.

### X-section clock override

When `state.t_model == T_GENERATOR_MODEL_GRIDS`, the XY clock source is
overridden **after** the normal self-patching detection:

```
external clock patched → CLOCK_SOURCE_EXTERNAL   (all X channels lock to external ramp)
no external clock      → CLOCK_SOURCE_INTERNAL_T2 (all X channels lock to ramps.master)
```

This ensures all three X outputs follow one steady clock instead of the
individual BD/SD/HH gate patterns. The self-patching detector loop still runs
when external clock is patched so its state stays coherent across mode switches.

### X1 clock output

In Grids mode, X1 outputs a 5 V / 0 V square wave derived from `ramp_buffer`
(the step-level ramp) instead of a random CV voltage:

```cpp
float x1 = grids_mode ? (ramp_buffer[i] < 0.5f ? 5.0f : 0.0f) : *v;
```

The `v` pointer still advances past the X1 voltage slot so X2, X3, and Y
assignments remain correctly aligned. The clock runs at 8th-note rate
(one cycle per 2 Grids steps / 6 master wraps).

---

## UI Changes (`ui.cc`)

### `SWITCH_T_MODEL`

Long press now **toggles** between bank 0 and bank 1:
- Bank 0 → long press → bank 0 model + 3 (enters Grids / alt models)
- Bank 1 → long press → bank 1 model − 3 (returns to standard models)

Short press behaviour is unchanged (cycles within the current bank).

From model 2 (DRUMS), long press reaches model 5 (GRIDS). The T-model LED
blinks red for model 5 via the existing `MakeColor` bank-1 slow-blink logic.

### `UpdateHiddenParameters()`

The `SWITCH_T_RANGE` held-key branch that mapped BIAS/JITTER pots to
`state.grids_hh_density` / `state.grids_chaos` has been **removed**.
Those parameters are now driven entirely by CV inputs.

---

## State Fields

| Field | Used in Grids mode? | Notes |
|---|---|---|
| `t_deja_vu` | Yes — sub-mode selector | OFF = Drums, ON/LOCKED = Euclidean |
| `t_pulse_width_mean` | No | Normal T modes only |
| `t_pulse_width_std` | No | Normal T modes only |
| `grids_hh_density` | No | Superseded by fixed-base + JITTER CV |
| `grids_chaos` | No | Superseded by Deja Vu knob + CV |
| `tb3po_seed` | Yes — TB-3PO acid seed | Persisted; survives power cycle when locked |
| `tb3po_active_slot` | Yes — active bank slot (0–3) | Which of the 4 saved seeds is currently selected |

Unused fields remain in `State` for ABI stability. The TB-3PO seed and active slot consume
3 of the 5 padding bytes; remaining padding is 2 bytes.

`PersistentData` carries `uint16_t tb3po_bank[4]` (8 bytes) — the 4-slot seed bank, stored
in what was previously the 16-byte `PersistentData::padding`; 8 bytes remain.

---

## TB-3PO X-Section (`marbles/tb3po/`)

When `state.t_model == T_GENERATOR_MODEL_GRIDS` the X-section runs a TB-3PO
style generative acid sequencer (`TB3PoSequencer`) locked to the same master
clock as the Grids drum engine. Outside Grids mode the sequencer is dormant
and the X-section behaves normally.

### Files

| File | Role |
|---|---|
| `tb3po/tb3po_sequencer.h / .cc` | Plain C++ port of the Hemisphere TB_3PO algorithm |

The original applet (`tb3po/tb3po.h`) was an O&C/Hemisphere class with
`HemisphereApplet` and `gfx*` dependencies that don't exist in Marbles. The
pattern-generation algorithm is reused; UI, hex-seed editing, density
automation, `no_slides`, and `hold_pitch` toggles are dropped from v1.

`makefile` — `marbles/tb3po` is included in the `PACKAGES` list.

### Clocking

The sequencer ticks once per X1 clock cycle (one 16th note = 6 master_phase
wraps = 2 Grids steps). marbles.cc detects step boundaries by watching the
`ramps.master` step ramp:

- A downward jump (ramp < prev_ramp − 0.5) marks the rising X1 edge →
  `tb3po.Tick(tb3po_reset_pending)`.
- A 0→1 crossing of 0.5 marks the falling X1 edge / half-step →
  `tb3po.TickHalfCycle()` (drives gate-off).
- `tb3po.StepSlide()` runs every sample to advance the slide IIR.

Reset latching: marbles.cc seeds `bool tb3po_reset_pending = grids_reset;`
*before* the per-sample loop and clears it on the first step boundary. A
reset that arrives mid-block is therefore consumed on the next step boundary
— even if that boundary lands in a later block — instead of being lost when
the for-loop ends.

No changes were needed in `t_generator` for the clock signal — `ramp_buffer`
is already exposed at the marbles.cc level.

### External-clock stall watchdog

When the T-clock input is patched, the per-sample loop tracks
`t_clock_silence_samples` (samples since the last `GATE_FLAG_RISING` on
`t_clock`). The threshold is set to `2 × t_clock_last_period_samples`,
clamped between 125 ms (`kSampleRate / 8`) and 2 s (`kSampleRate * 2`).
When silence reaches the threshold, `tb3po.ForceGateOff()` fires once so a
stopped upstream sequencer can't leave the TB-3PO gate latched into a
downstream VCA/ADSR. The next rising edge resets the counter and refreshes
the period estimate. When the T-clock is unpatched, the counter is held at
zero (internal-clock mode never stalls).

### Outputs (Grids mode only)

| Output | Voltage | Source |
|---|---|---|
| X1 | 5V / 0V | Existing Grids step-clock square wave (`ramp_buffer < 0.5`) |
| X2 | 1V/oct, slewed | `tb3po.pitch_volts()` |
| X3 | 5V / 0V | `tb3po.gate()` — held through slides |
| Y  | 5V / 0V | `tb3po.accent()` (high only when accent ∧ gate) |
| t1 / t2 / t3 | drums | unchanged (BD / SD / HH) |

`xy_generator.Process()` still runs so its ramp_extractor and random_sequence
state stay coherent across mode switches; its X2/X3/Y outputs are simply
overwritten before the DAC write.

### Control Mapping

| Marbles control | TB-3PO parameter | Mapping |
|---|---|---|
| X SPREAD knob | `density_encoder` | `round(spread * 14)` → 0..14 (treated as −7..+7) |
| X SPREAD CV  | `density_cv`      | `round(cv * 7)` → −7..+7 offset |
| X BIAS knob | `transpose` (knob component) | `round((unscaled_pot − 0.5) * 36)` semitones, clamped ±18 (3 octaves) |
| X BIAS CV | `transpose` (CV component) | `cv * 60` semitones (1 V/oct on the default uncalibrated −2.0 cv() scale), clamped ±18; summed with the knob component and passed as a float semitone count to `set_transpose()` |
| X STEPS knob + CV | `num_steps` | `1 + round(parameters[ADC_CHANNEL_X_STEPS] * 31)`, clamped to `TB3PoSequencer::kMaxSteps` → 1..32. The X_STEPS HysteresisFilter (0.02) is wider than one step (1/31 ≈ 0.032), so the count is stable |
| DEJA VU CV (rising edge) | T+X reset | `hidden_gates[ADC_CHANNEL_DEJA_VU_AMOUNT] & GATE_FLAG_RISING` resets both the Grids step pointer and TB-3PO step 0 (via `tb3po_reset_pending` latching, see Clocking) |
| X DEJA VU switch | `lock_seed` | `ON\|LOCKED → OFF` → reseed; `OFF → ON\|LOCKED` → commit + flash save |
| X SCALE (existing X selector) | scale lookup | reuses `state.x_scale` |
| X RANGE switch | unused | TB-3PO pitch is always 1 V/oct on X2 |
| LENGTH knob (`DEJA VU LENGTH`) | unused by TB-3PO | Drives the Grids Euclidean step count only; TB-3PO length comes from `X STEPS` |
| Main panel DEJA VU knob | unused by TB-3PO | Consumed by the T-section in Grids mode (bipolar around 12 o'clock: CCW = drum chaos / Euclidean T2 fills, CW = Euclidean rotation). `tb3po` does not read `deja_vu_raw` |
| Main panel DEJA VU CV jack | reset only | In Grids mode `parameters[ADC_CHANNEL_DEJA_VU_AMOUNT]` is forced to pot-only so the CV's analogue value doesn't contaminate chaos / rotation or the UI lock deadband — the jack is consumed purely as a reset trigger |

### Seed Persistence and Pattern Bank

- `State.tb3po_seed` (uint16) lives in `settings.cc Init()` defaults and rides
  along with the normal `chunk_storage_` save/load path.
- `Settings::SaveState()` is called from marbles.cc on the
  `x_deja_vu OFF → ON|LOCKED` edge so the committed seed survives a power
  cycle. We deliberately do **not** save on every OFF-tap reseed — auditioning
  could flip the switch many times.
- At boot, `Init()` calls `tb3po.set_seed(state.tb3po_seed)` and
  `set_lock_seed(state.x_deja_vu != DEJA_VU_OFF)`. `prev_x_deja_vu` is
  initialised from `state.x_deja_vu` so the first audio block doesn't fire a
  spurious reseed.
- TB-3PO uses `GridsRandom` as its RNG. The shared LFSR state is
  saved/restored around each regeneration so PatternGenerator's pertubation
  stream stays deterministic.
- **4-slot bank** in `PersistentData.tb3po_bank[4]`. In Grids mode:
  - **Short press `X MODE`** → saves `state.tb3po_seed` to the active slot and calls `SavePersistentData()`.
  - **Short press `X RANGE`** → advances `state.tb3po_active_slot` (0→1→2→3→0); if the new slot is non-zero, loads that seed into both `state.tb3po_seed` and the live `TB3PoSequencer` immediately.
  - Empty slots (value 0) are skipped on load — the current seed is left unchanged.
  - `UI_MODE_TB3PO_SLOT_FEEDBACK` activates for 1 s after either gesture: `LED_X_CONTROL_MODE` lights the slot color (green/yellow/red/blink-green for slots 0–3), `LED_X_RANGE` blinks the same color.
  - During normal Grids operation `LED_X_CONTROL_MODE` shows the active slot color (replaces the unused `x_control_mode` display).

### Slide IIR

```cpp
constexpr float kSlideCoef = 0.003f;  // ~25 ms TC at 32 kHz; tune on hardware
pitch_volts_ += kSlideCoef * (slide_target_ - pitch_volts_);
// Clamp to keep direction monotonic, matching TB-3PO's CONSTRAIN.
```

### In-scale Pitch Selection

TB-3PO walks `Scale::degree[]` cells directly — it doesn't go through the
weight-aware quantizer. To avoid emitting chromatic passing tones on the
stock 12-degree weighted presets (C major, Pentatonic, raags) the sequencer
filters degrees by weight at `set_scale()` time:

- Compute `max_weight` across all degrees.
- For 12-degree scales, keep degrees with `weight >= max_weight / 4`.
  Empirically catches the diatonic notes (weight ≥ 64 in the defaults) and
  rejects the 4/8/16/32-weight chromatic passing tones.
- For smaller scales (Pelog, user-recorded), keep every degree — weight in
  those presets shapes the quantizer's selection probability, not scale
  membership.

The filtered list lives in `active_idx_[]` (degree indices) and `notes_[s]`
stores a *rank* into that list. `PitchForStep` composes transpose and octave
shifts in **active-rank units**, so transposing by +1 moves to the next
in-scale degree (e.g., C → D on C major), never to a chromatic passing tone.

Scales walked by the sequencer with this filter:

| X SCALE | num_degrees | active_count | Notes |
|---|---|---|---|
| C major | 12 | 7 | C D E F G A B |
| C minor | 12 | 7 | C D Eb F G Ab Bb |
| Pentatonic | 12 | 5 | C D F G A |
| Pelog | 7 | 7 | all 7 (no filter) |
| Raag Bhairav That | 12 | 7 | starred degrees only |
| Raag Shri | 12 | 7 | starred degrees only |

### Reset Handling

The original applet treated `Reset()` and the first post-reset clock as two
separate events; the first clock had `step_pv == step == 0`. In the ported
version `Tick(reset=true)` collapses both into a single call: it sets
`step_=0` *and* forces `prev_step=0` so the first step's slide / pitch
decisions don't inherit stale bits from whatever step the pattern was on at
the moment of reset.
