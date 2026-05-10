# Marbles Custom Firmware: Grids Mode Plan

This document outlines the plan to port the logic from Grids into Marbles, adding it as an alternative "Red" mode in the clock (T) section.

## 1. Architectural Overview

Marbles' T-generator normally produces three gate outputs:
- `t2`: Master clock (steady or with jitter).
- `t1`, `t3`: Jittered or subdivided/multiplied versions of `t2`.

Grids is a 3-channel drum pattern generator (BD, SD, HH) based on a 2D map.

The goal is to allow Marbles to run the Grids pattern generation logic, mapping BD, SD, and HH to `t1`, `t2`, and `t3` respectively when Grids mode is active.

### Technical Porting Strategy
The Grids logic in `@marbles/grids/**` will be treated as a **Fixed-Point Engine**. This preserves the original mathematical character of the map interpolation and Euclidean generation.
- **Data Types**: Keep the `uint8_t` (0-255) logic for map coordinates and densities.
- **Bridge**: Convert Marbles' normalized floats to Grids integers using `static_cast<uint8_t>(value * 255.0f)`.
- **Timing**: Bypass the Grids `Clock` class entirely. Sync `pattern_generator.TickClock(6)` to Marbles' `master_phase_` wraps (6 ticks per 16th note to reach 24 PPQN).
- **Randomness**: Use Grids' internal 16-bit LFSR (`Random` class) for pattern perturbation to ensure identity with the original hardware.

## 2. Model and UI Integration

### New Model Index
The `TGeneratorModel` enum in `marbles/random/t_generator.h` currently has 7 entries. Index 5 (`T_GENERATOR_MODEL_THREE_STATES`) is the "Alternative Red" mode. I will repurpose it as `T_GENERATOR_MODEL_GRIDS`. Index 6 (`T_GENERATOR_MODEL_MARKOV`) remains reachable only via debug/hack and is untouched.

### UI Selection
- **Mode Selection**: A long press on the `T_MODEL` button while in "Red" mode (Bank 0 or Bank 1) will toggle between the standard Marbles drum models (`DRUMS` or `THREE_STATES`) and the Grids mode.
- **Visual Feedback**: When Grids mode is active, the Red T-model LED will blink to distinguish it from the standard solid Red modes.

### Grids Sub-Mode Selection (Drums vs Euclidean)
Within Grids mode, the **T Deja Vu button** selects the active sub-mode:

| `state.t_deja_vu` | T Deja Vu LED | Grids Sub-Mode |
|---|---|---|
| `DEJA_VU_OFF` | Off | **Drums** — 2D map-based pattern generation |
| `DEJA_VU_ON` | Solid green | **Euclidean** — Euclidean rhythm generation |
| `DEJA_VU_LOCKED` | Pulsing green | **Euclidean** — same as ON |

The button's existing cycle (`OFF → ON → LOCKED → ON`) is unchanged. Grids mode intercepts the state value to select a sub-mode rather than enabling/disabling pattern repetition (which is meaningless in a pattern-generator context). The T Deja Vu LED therefore doubles as a sub-mode indicator.

## 3. Parameter Mapping

### 3a. Drums Sub-Mode (`t_deja_vu == DEJA_VU_OFF`)

| Marbles Control | Grids Parameter | Notes |
|---|---|---|
| **RATE knob** | Tempo / Division | Pattern speed |
| **BIAS knob** | Map X | X coordinate on the 2D drum map |
| **JITTER knob** | Map Y | Y coordinate on the 2D drum map |
| **RATE CV** | BD Density | Fixed base 0.5; pure CV offset. `0.5 + cv / 120.0f` |
| **BIAS CV** | SD Density | Fixed base 0.5; pure CV offset |
| **JITTER CV** | HH Density | Fixed base 0.5; pure CV offset |
| **DEJA_VU knob + CV** | Chaos (`randomness`) | Sweeps pattern perturbation 0→255 |

All three density channels start at the midpoint (128/255) with no CV patched. The Deja Vu knob (plus its CV jack, combined in `parameters[ADC_CHANNEL_DEJA_VU_AMOUNT]`) drives the `DrumsSettings::randomness` field. The Grids LFSR handles all internal pattern repetition.

### 3b. Euclidean Sub-Mode (`t_deja_vu != DEJA_VU_OFF`)

`EvaluateEuclidean` in `pattern_generator.cc` maps settings as:
- `euclidean_length[i]`: `(value >> 3) + 1` → 1–32 steps per channel
- `density[i]`: `value >> 3` → 0–31 Euclidean fill notes

There is no randomness/perturbation field in Euclidean evaluation, so the Deja Vu knob has no effect in this sub-mode.

| Marbles Control | Euclidean Parameter | Notes |
|---|---|---|
| **RATE knob** | Tempo / Division | Same as Drums sub-mode |
| **BIAS knob** | Length for BD and SD | `euclidean_length[0]` = `euclidean_length[1]` = `uint8_t(bias * 255)` |
| **JITTER knob** | Length for HH | `euclidean_length[2]` = `uint8_t(jitter * 255)` |
| **RATE CV** | BD Density | Same formula as Drums sub-mode |
| **BIAS CV** | SD Density | Same formula as Drums sub-mode |
| **JITTER CV** | HH Density | Same formula as Drums sub-mode |
| **DEJA_VU knob + CV** | (unused) | No chaos field in Euclidean evaluation |

BD and SD share the same Euclidean length (BIAS knob), giving kick and snare a common metric grid. HH gets an independent length (JITTER knob), which is the most common Euclidean drumming use-case (e.g., 5-in-16 over 4-in-16).

### Density Behaviour (both sub-modes)
All three density channels have a **hard-coded base of 0.5** (128/255). The corresponding CV input is the only offset. No hidden-parameter overrides exist for densities in grids mode.

### State Fields: Usage Summary
| `State` field | Used in Grids mode? | Notes |
|---|---|---|
| `t_deja_vu` | Yes — as sub-mode selector | OFF = Drums, ON/LOCKED = Euclidean |
| `t_pulse_width_mean` | No | Normal T modes only |
| `t_pulse_width_std` | No | Normal T modes only |
| `grids_hh_density` | No | Superseded by fixed-base + JITTER CV |
| `grids_chaos` | No | Superseded by Deja Vu knob + CV |

The four unused fields remain in `State` for ABI stability.

## 4. Implementation Steps

### Phase 1: Core Logic Integration & Port Cleanup
1.  **Refactor Port**: Remove `avrlib` dependencies from `marbles/grids/**`. Replace `LongWord` and AVR-specific headers with `stmlib` or standard C++ types.
2.  **Renaming and Enum Update**: Rename `T_GENERATOR_MODEL_THREE_STATES` to `T_GENERATOR_MODEL_GRIDS` in `t_generator.h`.
3.  **Header Inclusion**: Include `marbles/grids/pattern_generator.h` in `t_generator.cc`.
4.  **TGenerator Class Update**:
    - Add a `PatternGenerator` instance (or use the existing static one if appropriate).
    - Add `master_gate_` boolean to `TGenerator` to allow overriding the `t2` output.
5.  **Process Loop Integration**:
    - In `TGenerator::Process`, detect master phase wraps.
    - On wrap, call `pattern_generator.TickClock(6)` to align Marbles pulses with Grids steps.
    - Map Marbles parameters to `pattern_generator.mutable_settings()`.

### Phase 2: Euclidean Sub-Mode Support (`t_generator.h` / `t_generator.cc`)
Add fields and setters to `TGenerator` for the Euclidean sub-mode:

```cpp
// t_generator.h — new fields
bool  grids_euclidean_;
float grids_euclidean_length_bd_sd_;   // BIAS knob → BD + SD length
float grids_euclidean_length_hh_;      // JITTER knob → HH length

// setters
inline void set_grids_euclidean(bool e)             { grids_euclidean_ = e; }
inline void set_grids_euclidean_length_bd_sd(float v) { grids_euclidean_length_bd_sd_ = v; }
inline void set_grids_euclidean_length_hh(float v)    { grids_euclidean_length_hh_ = v; }
```

In `GenerateGrids` (t_generator.cc), branch on `grids_euclidean_`:

```cpp
PatternGeneratorSettings* s = pattern_generator.mutable_settings();
pattern_generator.set_output_mode(grids_euclidean_ ? OUTPUT_MODE_EUCLIDEAN
                                                   : OUTPUT_MODE_DRUMS);
if (grids_euclidean_) {
  uint8_t len_bd_sd = static_cast<uint8_t>(grids_euclidean_length_bd_sd_ * 255.0f);
  uint8_t len_hh    = static_cast<uint8_t>(grids_euclidean_length_hh_    * 255.0f);
  s->options.euclidean_length[0] = len_bd_sd;
  s->options.euclidean_length[1] = len_bd_sd;
  s->options.euclidean_length[2] = len_hh;
  // chaos has no effect; density[] is set the same way in both branches
} else {
  s->options.drums.x          = static_cast<uint8_t>(bias_  * 255.0f);
  s->options.drums.y          = static_cast<uint8_t>(jitter_ * 255.0f);
  s->options.drums.randomness = static_cast<uint8_t>(grids_chaos_ * 255.0f);
}
s->density[0] = static_cast<uint8_t>(grids_bd_density_ * 255.0f);
s->density[1] = static_cast<uint8_t>(grids_sd_density_ * 255.0f);
s->density[2] = static_cast<uint8_t>(grids_hh_density_ * 255.0f);
```

### Phase 3: Revised Control Mapping (`marbles.cc`)
Replace the existing grids parameter block in `Process()`:

```cpp
if (state.t_model == T_GENERATOR_MODEL_GRIDS) {
  t_generator.set_rate(cv_reader.channel(ADC_CHANNEL_T_RATE).pot());
  t_generator.set_bias(cv_reader.channel(ADC_CHANNEL_T_BIAS).pot());
  t_generator.set_jitter(cv_reader.channel(ADC_CHANNEL_T_JITTER).pot());

  float bd = 0.5f + cv_reader.channel(ADC_CHANNEL_T_RATE).cv() / 120.0f;
  float sd = 0.5f + cv_reader.channel(ADC_CHANNEL_T_BIAS).cv();
  float hh = 0.5f + cv_reader.channel(ADC_CHANNEL_T_JITTER).cv();
  CONSTRAIN(bd, 0.0f, 1.0f);
  CONSTRAIN(sd, 0.0f, 1.0f);
  CONSTRAIN(hh, 0.0f, 1.0f);
  t_generator.set_grids_bd_density(bd);
  t_generator.set_grids_sd_density(sd);
  t_generator.set_grids_hh_density(hh);

  bool euclidean = (state.t_deja_vu != DEJA_VU_OFF);
  t_generator.set_grids_euclidean(euclidean);
  if (euclidean) {
    t_generator.set_grids_euclidean_length_bd_sd(
        cv_reader.channel(ADC_CHANNEL_T_BIAS).pot());
    t_generator.set_grids_euclidean_length_hh(
        cv_reader.channel(ADC_CHANNEL_T_JITTER).pot());
    // chaos unused in Euclidean; skip set_grids_chaos
  } else {
    t_generator.set_grids_chaos(parameters[ADC_CHANNEL_DEJA_VU_AMOUNT]);
  }
}
```

Note: `t_generator.set_deja_vu(...)` is **not** called for the T section when grids mode is active — the deja_vu state is consumed for sub-mode selection instead.

### Phase 4: UI Cleanup (`ui.cc`)
Remove the `SWITCH_T_RANGE` hidden-parameter branch entirely:

```cpp
// DELETE this block:
} else if (switches_.pressed(SWITCH_T_RANGE)) {
    if (i == ADC_CHANNEL_T_BIAS) destination = &state->grids_hh_density;
    if (i == ADC_CHANNEL_T_JITTER) destination = &state->grids_chaos;
}
```

The T_MODEL held-key mappings for pulse width (`t_pulse_width_mean`, `t_pulse_width_std`) are unchanged — they apply only in non-grids modes.

### Phase 5: UI and Display
1.  **UI Logic Update**: Modify `Ui::OnSwitchReleased` in `ui.cc` to handle the Grids toggle.
2.  **LED Logic Update**: Modify `Ui::UpdateLEDs` to support a blinking state for the Grids mode. The T Deja Vu LED already handles its own color based on `state.t_deja_vu` — no extra changes needed for the sub-mode indicator.

### Phase 6: Hardware Interface
1.  **t2 Output Override**: Modify `marbles.cc` to use `t_generator.master_gate()` instead of the hardcoded `ramps.master[i] < 0.5f`.
2.  **Build System**: Add `marbles/grids` to the `PACKAGES` list in `makefile`.

## 5. Verification Plan
- **Unit Tests**: Add a new test case in `marbles_test.cc` that initializes `TGenerator` in Grids mode and verifies that gates are produced on all three channels in both sub-modes.
- **Drums Sub-Mode**:
  - Verify X/Y map coordinates (BIAS/JITTER knobs) correctly shift the drum pattern.
  - Confirm Deja Vu knob sweeps chaos from no perturbation (CCW) to full randomness (CW).
- **Euclidean Sub-Mode**:
  - Confirm BIAS knob sweeps BD/SD pattern length from 1 to 32 steps.
  - Confirm JITTER knob sweeps HH pattern length independently from 1 to 32 steps.
  - Confirm the Deja Vu knob has no audible effect (chaos is ignored).
- **Density (both sub-modes)**:
  - Confirm each density channel rests at mid-density with no CV patched.
  - Confirm ±5V CV input spans the full 0–1 density range.
- **Sub-mode switching**: Verify pressing the T Deja Vu button while in Grids mode toggles between Drums and Euclidean patterns without glitches or state corruption.

## 6. Rationale for Control Scheme

1. **Density as CV-only**: Hidden parameters require two hands and provide no visual feedback. CV-only density with a fixed midpoint base is more immediate and patching-friendly for a drum sequencer, and makes the module boot into a predictable state.
2. **Chaos on Deja Vu knob**: The Deja Vu knob is otherwise unused in grids mode (the Grids LFSR replaces dejavu). Repurposing it gives chaos a dedicated, front-panel-accessible control with CV.
3. **Euclidean via Deja Vu button**: The button is a natural on/off toggle already wired to the T section. Reusing it as a sub-mode selector costs nothing ergonomically, and the existing LED colors (off / solid green / pulsing green) provide clear visual feedback about the active sub-mode.
4. **BD+SD share length in Euclidean**: Kick and snare most commonly share a metric grid. Giving them one knob (BIAS) keeps the interface simple while allowing HH to run an independent Euclidean pattern via JITTER.
