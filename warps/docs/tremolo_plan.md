# Tremolo (Symbiote) — design plan

Stereo amplitude modulator driven by a single LFO with selectable waveshape and continuous stereo phase offset. Self-contained (no shared FX buffer); scaffolding mirrors the existing phaser mode.

## Topology

- One LFO phase accumulator (`lfo_phase_`); right channel is read at `lfo_phase_ + stereo_phase_offset`.
- Per-channel sample-and-hold state with independent wrap detection (so S&H re-samples at the R-channel offset crossing, not in lockstep with L).
- Per-channel one-pole smoother (~1 ms) on the final gain to de-zipper square / S&H transitions.
- Wet/dry crossfade in `amount_`; depth=0 → unity gain, depth=1 → fully modulated down to silence at the trough.

Gain shape per channel: `gain = 1.0f - depth * (0.5f - 0.5f * lfo)`, where `lfo ∈ [-1, +1]`.

## Waveshapes (`carrier_shape`, 4 values)

| `carrier_shape` | Shape | Notes |
|---|---|---|
| 0 | Sine | `lut_sin` lookup, same idiom as phaser |
| 1 | Square | `phase < 0x80000000 ? +1 : -1` (50% duty) |
| 2 | Sawtooth | Linear ramp from `+1 → -1` over the cycle |
| 3 | Sample & Hold | New random value latched per channel on phase wrap |

S&H uses a simple PRNG (e.g. `Random::GetFloat()` or an in-class xorshift) sampled when the phase counter wraps. Detect wrap per channel via `prev_phase_*`.

## Stereo phase offset

- Range: `0.0 .. 0.5` of a cycle (clamped at 0.5 — beyond is a mirror image of the same behaviour).
- Landmarks:
  - `0.00` → classic mono tremolo (L = R)
  - `0.25` (90°) → autopan, near-constant mono sum
  - `0.50` (180°) → Leslie / rotary; with square shape becomes a hard L↔R chopper

## Rate

Exp-mapped from `0.1 Hz` to `~15 Hz`, e.g. `lfo_freq = 0.1f * SemitonesToRatio(rate * 86.7f)` (`log2(150) * 12 ≈ 86.7`). Extends well past the 6 Hz phaser ceiling so that square / S&H reach chop / stutter territory.

## Control mapping (Warps panel)

| Hardware | Tremolo param |
|---|---|
| LEVEL CVs (`raw_level_cv[0/1]`, unpatched → 0.6) | `ApplyAmplification(..., true)` — input VCAs |
| LEVEL1 pot (`raw_level_pot[0]`) | `set_amount` (dry/wet) |
| LEVEL2 pot (`raw_level_pot[1]`) | `set_depth` |
| ALGO (`raw_algorithm`) | `set_rate` (0.1–15 Hz, exp) |
| MOD (`modulation_parameter`) | `set_stereo_phase` (0..0.5 cycle) |
| OSC button (`carrier_shape` 0..3) | `set_shape` (sine / square / saw / S&H) |

All pots read via `previous_parameters_` so values are coherent with the rendered audio block — same idiom as `ProcessPhaser` in [dsp/modulator.cc](../dsp/modulator.cc).

## Files & integration

### New: `dsp/fx/tremolo.h`

Class `Tremolo` with:

```cpp
void Init(float sample_rate);
void Reset();
void set_rate(float);
void set_depth(float);
void set_amount(float);
void set_stereo_phase(float);   // 0..0.5
void set_shape(int32_t);        // 0..3
void Process(float* left, float* right, size_t size);
```

State: `sample_rate_`, `rate_`, `depth_`, `amount_`, `stereo_phase_`, `shape_`, `lfo_phase_`, `prev_phase_l_`, `prev_phase_r_`, `sh_l_`, `sh_r_`, `gain_smooth_l_`, `gain_smooth_r_`, PRNG state.

### `dsp/parameters.h`

Add `FEATURE_MODE_TREMOLO` to the `FeatureMode` enum (immediately before `FEATURE_MODE_META`, which must remain the highest-valued entry — it is the default and the UI uses it as the count anchor; verify against [ui.cc](../ui.cc) before final placement).

### `dsp/modulator.h`

- Add `static Tremolo tremolo;` next to `static Phaser phaser;`.
- Declare `void ProcessTremolo(ShortFrame*, ShortFrame*, size_t);`.
- **Do not** add `FEATURE_MODE_TREMOLO` to the `is_fx` (shared 32k buffer) flag — tremolo is self-contained, like the phaser.

### `dsp/modulator.cc`

- `Init()`: `tremolo.Init(96000.0f);` (match the rate used for the phaser; confirm against the actual processing-rate constant near phaser init).
- `reset_fx` block at the top of `Process()`: add `tremolo.Reset();`.
- Dispatch:

```cpp
case FEATURE_MODE_TREMOLO:
  ProcessTremolo(input, output, size);
  break;
```

`ProcessTremolo` body follows the phaser shape: input VCAs via `ApplyAmplification(..., true)`, copy carrier/modulator into `main_output`/`aux_output`, call `tremolo.Process(...)`, finish with `Convert(output, main_output, aux_output, 32768.0f, size); previous_parameters_ = parameters_;`.

## Anti-click notes

- Square and S&H both produce hard amplitude jumps. The per-channel one-pole on the *output gain* (not on the LFO itself — that would slur the shape) is what keeps these clean.
- Coefficient `a = 1 - exp(-2π * fc / fs)` with `fc ≈ 150–250 Hz` is a good starting point. Tune by ear; too slow turns square into a soft tremolo, too fast lets clicks through.
- Sawtooth has one hard discontinuity per cycle (at the wrap from −1 back to +1). The same smoother handles it.

## Out of scope (v1)

- Tempo / clock sync (similar to how `FEATURE_MODE_DELAY` consumes the tempo source). Worth a follow-up if there's demand.
- Multi-LFO / cross-modulation between channels.
- Triangle shape — covered well enough by sine for tremolo purposes; saw is the more distinctive addition.

## Docs touch-ups when shipping

- Update the feature-mode table and control-mapping bullets in [warps/CLAUDE.md](../CLAUDE.md).
- Add a public manual entry in the blog repo under `blog/pages/*-symbiote.md`.
