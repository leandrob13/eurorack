# Compact pitch shifter — design plan

## Goal

Add a `FEATURE_MODE_PITCH_SHIFTER` to Warps Symbiote that pitch-shifts the
stereo input by ±1 octave (and a "shimmer"-style regenerative octave) using
**no new RAM** and a CPU budget well under the existing Reverb mode.

Non-goals (deliberate):

- No phase vocoder / FFT. The previous `phase_vocoder_port_plan.md` (now
  deleted) explored a CMSIS-DSP PV; it requires ~8 KB of FFT scratch + the
  CMSIS-DSP code-size hit and ~70 % of an F4 at full quality. Out of scope.
- No formant correction.
- No polyphonic tracking. This is a granular/time-domain shifter — wobble on
  complex melodic material is expected and acceptable.

## Topology: 2-tap crossfading delay-line (SOLA-style)

Classic constant-shift implementation:

1. Write input into a circular delay line at unit speed.
2. Read with a tap that lags the write head at *variable* speed
   `read_rate = 1.0 - (ratio - 1.0) = 2.0 - ratio` for up-shift
   (`ratio > 1`) or `read_rate = 1.0 - (ratio - 1.0)` for down-shift
   (`ratio < 1`). Equivalent formulation: the tap-to-write distance
   changes by `(1 - ratio)` samples per sample.
3. When the tap-to-write distance hits the window boundary, wrap it
   instantly to the other end of the window. To hide the discontinuity,
   run **two** taps offset by half a window and crossfade between them
   with a raised-cosine (Hann) envelope. Each tap is faded in while it is
   far from the wrap point and faded out as it approaches it.

Window length is fixed (compile-time constant ~50 ms = 2400 samples @
48 kHz). Longer windows = lower flutter rate but worse transient smearing;
50 ms is the usual sweet spot for musical material.

Tap reads use linear interpolation (`Interpolate` from `fx_engine.h` is
already q15-aware) — Hermite is not worth the cycles at this fidelity.

## RAM plan — reuse, do not allocate

Reuse the existing shared 32 k × uint16_t FX buffer (`reverb_buffer`,
64 KB, defined once in `warps.cc` and passed into `Modulator::Init`).
The same buffer already backs both `Reverb` and `Ensemble`; only one
feature mode runs at a time, so adding `PitchShifter` to the list of
consumers is safe.

Concretely:

- Add `PitchShifter pitch_shifter_;` member to `Modulator`.
- In `Modulator::Init`, call `pitch_shifter_.Init(reverb_buffer)` alongside
  the existing `reverb.Init` / `ensemble.Init` calls.
- Implement `PitchShifter` as
  `FxEngine<8192, FORMAT_16_BIT>` per channel — two FxEngines (L/R) each
  consume 16 KB of the shared buffer (`8192 * 2 bytes`), totalling 32 KB.
  This leaves the upper 32 KB of the shared buffer unused in this mode,
  which is fine (RAM is shared, not partitioned per mode).
  - Alternative if FxEngine's single-buffer assumption is awkward: use
    one `FxEngine<16384, FORMAT_16_BIT>` and split the address space into
    L (lower 8192) and R (upper 8192) by offsetting the write/read
    indices. Decide during implementation; the FxEngine API supports this
    with two `Delay` declarations of different sizes inside a single
    Context. **Preferred** — keeps the same buffer pointer that
    `reverb.Init` already uses.
- 8192 samples @ 48 kHz = 170 ms; window = 50 ms ⇒ ~3× headroom for the
  two taps + crossfade.
- Per-instance heap state in `PitchShifter` itself: two float read
  positions per channel, one crossfade phase per channel, one feedback
  sample per channel, the smoothed `ratio` (param interpolator). About
  6 floats × 2 channels = 48 bytes. Negligible.

**No `delay_buffer_` impact**, no new global buffers, no `APPLICATION_LARGE`
toggle. Phaser already cost the project 256 B off `delay_buffer_`; this
mode costs zero.

## Control mapping (`ProcessPitchShifter`)

Following the phaser/reverb idiom — use `previous_parameters_` for control
values so they line up with the audio block:

| Source | Target | Notes |
|---|---|---|
| `raw_level_cv` via `ApplyAmplification(..., true)` | input VCAs | `cv_scaler` forces `0.6f` when jack unpatched — audio passes without a cable, same as Phaser/Dual Filter |
| `raw_level_pot[0]` | dry/wet mix | 0 = dry, 1 = wet |
| `raw_level_pot[1]` | feedback (regenerative) | scaled ×0.85; routes wet output back through the shifter for shimmer octaves |
| `raw_algorithm` | coarse pitch | mapped to semitones ∈ [-12, +12], optionally quantized in some voicings (see below) |
| `modulation_parameter` | fine detune | ±50 cents around coarse, **additive**, continuous |
| `carrier_shape` (0..3) | voicing | see table |

Voicings (`carrier_shape`):

| `carrier_shape` | Behaviour |
|---|---|
| 0 | **Mono detune.** L = +detune, R = -detune, coarse ignored — small-interval chorus/thickener. Detune from `algorithm + modulation_parameter`, ±1 semitone range. |
| 1 | **Continuous shifter.** Standard ±1 octave from `algorithm`, fine from `mod_param`. L/R identical. |
| 2 | **Quantized shifter.** Same as voicing 1, but coarse snaps to {-12, -7, -5, 0, +5, +7, +12} semitones (fifths/fourths/octaves). `mod_param` still adds continuous detune. |
| 3 | **Shimmer.** Forces +12 semitones, feedback param controls the shimmer feedback gain *into the same buffer* with a small LPF in the loop (so it does not blow up). `algorithm` becomes a feedback HP/LP tilt; `mod_param` becomes a damping. |

## Pseudocode (single channel, before stereo split)

```cpp
void Process(float* in, float* out, size_t size) {
  ParameterInterpolator ratio(&ratio_, target_ratio_, size);
  ParameterInterpolator mix(&mix_, target_mix_, size);
  float fb = feedback_;

  for (size_t s = 0; s < size; ++s) {
    float r = ratio.Next();   // 0.5 .. 2.0
    float m = mix.Next();
    float x = in[s] + fb * feedback_amount_;

    // Write head fixed at index 0 of FxEngine context.
    Write(line_, x);

    // Advance two read taps; each lags the write by tap0_pos_, tap1_pos_
    // samples (float). Distance changes by (1 - r) per sample.
    tap0_pos_ += 1.0f - r;
    tap1_pos_ += 1.0f - r;
    Wrap(tap0_pos_, 0.0f, kWindow);    // jump by kWindow on wrap
    Wrap(tap1_pos_, 0.0f, kWindow);    // tap1 offset by kWindow/2 init

    float y0 = Interpolate(line_, tap0_pos_);
    float y1 = Interpolate(line_, tap1_pos_);

    // Hann crossfade based on each tap's position within its window.
    float w0 = 0.5f - 0.5f * cosf_lut(2.0f * M_PI * tap0_pos_ / kWindow);
    float w1 = 0.5f - 0.5f * cosf_lut(2.0f * M_PI * tap1_pos_ / kWindow);
    float y  = y0 * w0 + y1 * w1;

    fb = y;
    out[s] = in[s] + (y - in[s]) * m;
  }
  feedback_ = fb;
}
```

(The actual implementation will use the `FxEngine` `Context` /
`Interpolate(line, position, gain)` pattern from `reverb.h` rather than
hand-rolled wrap logic, and the `cosf_lut` lookup should come from the
existing `lut_sin` shifted by π/2.)

## Implementation phases

1. **Wire the mode.** Add `FEATURE_MODE_PITCH_SHIFTER` to `parameters.h`
   (before `FEATURE_MODE_META`), bump the UI cycle list, add the
   `case` to the `Process()` switch in `modulator.cc`. Stub
   `ProcessPitchShifter` as silence. Verify mode-select works on
   hardware.
2. **Mono shifter, voicing 1 only.** Implement `PitchShifter` class with
   one read tap (audible artefacts at every wrap) and shared FX buffer.
   Confirm coarse pitch control works and ratio scaling is correct.
3. **Two-tap crossfade.** Add second tap + Hann crossfade. Tune window
   length on sustained tones; should be inaudible on pads, mildly
   present on drums.
4. **Stereo.** Split buffer L/R, route both channels.
5. **Voicings.** Add `carrier_shape` switch for detune / quantized /
   shimmer modes.
6. **Feedback / shimmer polish.** Implement the LPF in the shimmer
   feedback path. Verify no DC accumulation, no runaway gain.
7. **CPU & RAM verification.** Run `make size`; check `arm-none-eabi-objdump`
   stack usage; profile worst case (shimmer + max feedback) on hardware.

## Open decisions for review

- **Crossfade waveform:** Hann (above) vs. equal-power (sin/cos). Hann
  has a small amplitude dip mid-fade but no audible artefacts in tests
  reported in other modules; equal-power preserves loudness perfectly
  but costs an extra `sqrtf` or LUT. Recommend Hann.
- **Tap interpolation:** linear is the budget option, Hermite/4-point is
  ~3× the cycles. Recommend starting linear; revisit if HF aliasing on
  down-shift is objectionable.
- **Coarse-pitch CV range:** 1 V/oct over `algorithm`? Currently the
  param is 0..1; a true V/oct response means we should consume the
  scaled CV before normalisation, which means following the same path
  the Frequency Shifter uses. Decide in Phase 2.
- **Buffer split:** single `FxEngine` with two `Delay` declarations
  versus two separate `FxEngine` instances. The former matches how
  `reverb_buffer` is currently shared by `Reverb` + `Ensemble`. The
  latter is simpler to reason about. Recommend the former for
  consistency with the existing code.

## Source-of-truth notes

- `dsp/fx/pitch_shifter.h` is referenced in `warps/CLAUDE.md` but **does
  not exist** in the tree as of this plan. Creating it is part of Phase 1.
- The CLAUDE.md mention of a "naive shifter commented out in the
  Frequency Shifter dispatch" reflects the deleted PV plan's framing —
  there is no current pitch-shifter implementation to fall back on. This
  plan is the from-scratch design.
