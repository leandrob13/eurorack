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

Unused fields remain in `State` for ABI stability.
