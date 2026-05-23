# Porting a Symbiote phaser to Warps

## What we already have working in our favor

The Symbiote FX dispatch in [../dsp/modulator.cc](../dsp/modulator.cc) is already shaped exactly the way a new effect needs:

- **A pluggable mode dispatch.** [../dsp/modulator.cc](../dsp/modulator.cc) lines 892–944 switches on `feature_mode_`, with one `ProcessXxx` method per mode. Adding a phaser is one new enum value in [../dsp/parameters.h](../dsp/parameters.h) (lines 44–54), one new `Process*` method, and one new `case` in the switch.
- **Reference effects with the same parameter shape.** `ProcessEnsemble` ([../dsp/modulator.cc](../dsp/modulator.cc) lines 419–453) is the closest cousin — a modulation effect driven by `modulation_parameter` and an internal LFO. Its mapping (`raw_level_pot`, `raw_algorithm`, `modulation_parameter`, `carrier_shape`) is the template to follow. We can copy its `previous_parameters_` usage and `Convert(output, main, aux, 32768.0f, size)` tail verbatim.
- **A free LFO source.** Both `lut_sin` ([used by ensemble](../dsp/fx/ensemble.h) line 132) and `stmlib::CosineOscillator` ([used by the FxEngine](../dsp/fx/fx_engine.h) line 297) are already linked in and well-tested in the FX modes. We don't need to add a new oscillator primitive.
- **Native test harness.** `warps/test/` runs the FX paths natively under g++ — we can iterate phaser coefficients against a host build and capture wet output to file before touching hardware.

## What a phaser actually is

A traditional analog-style phaser is structurally **not** a delay-network effect:

- It's a cascade of `N` (typically 4–12) **first-order allpass filters**, each with a single one-sample state variable. No buffered delay line.
- A low-frequency oscillator sweeps the allpass break frequency `f_c(t)` over a range (e.g. 80 Hz → 8 kHz). Each allpass introduces a frequency-dependent phase shift; summing the cascade output back with the dry signal produces moving notches in the response.
- Optional feedback from the last stage to the input deepens / sharpens the notches ("resonance").
- Stereo is typically achieved by giving the L and R cascades a quadrature LFO (90° phase offset) or by running two separate cascades with different stage counts / center frequencies.

The math per stage is the bilinear-transformed first-order allpass:

```
y[n] = a * x[n] + x[n-1] - a * y[n-1]    (Direct Form I)
a    = (1 - tan(π f_c / fs)) / (1 + tan(π f_c / fs))
```

`a` lives in `(-1, 1)` and only depends on the current cutoff `f_c`. Per sample, per stage: one multiply, one add, one state update. Cheap.

### Why we should *not* shoehorn it into `FxEngine`

`FxEngine`'s primitives (`Read`, `Write`, `WriteAllPass`, `Interpolate`) are designed for **Schroeder allpasses** — delay-line allpasses of the form `y = -g x + d[n-M] + g (x + g d[n-M])`, with `M` samples of integer delay. Those are the right tool for reverb and chorus diffusion, but they are not what an analog-style phaser uses. Forcing the phaser onto `FxEngine`:

- wastes the entire 64 KB shared FX buffer (the phaser needs ~tens of floats of state, not 32k samples of q15),
- masks the simpler `1 mul + 1 add` per stage behind the `Read/Write/WriteAllPass` template churn,
- prevents the phaser from running alongside reverb or ensemble in the future (the buffer is shared between those two and would now be a three-way conflict).

The phaser should be a self-contained `Phaser` class with its own float state — no shared buffer dependency. This also means it does **not** participate in the `reset_fx` clear at the top of `Modulator::Process()`, but we should give it its own `Reset()` and call it on mode entry to flush stale state.

## Topology proposal

```
                                ┌──────────────────────────────┐
                                │              feedback        │
                                ▼                              │
input ──► [+]──► AP₁ ─► AP₂ ─► ... ─► APₙ ───┬────────────────┘
           ▲                                  │
           │                                  │
           │                              wet ▼
           │                              [mix]
           └──────────────────────────► dry ──► output
                                         │
                                         ▼
                                       output
```

- `AP_i` is a 1st-order allpass with state `z_i`, coefficient `a(t)`. All stages share the same `a(t)` driven by the LFO (or we can spread them across a frequency range — see "voicings" below).
- Stereo: run a second cascade with the LFO phase offset by 90° (sine vs. cosine). Memory cost is `2 × N` floats — trivial.
- Output: `wet = AP_N output`, `mix = dry + (wet − dry) × amount` (linear blend). A "Univibe"-style switchable mode would invert the polarity of `wet` before summing (vibrato-style vs. phaser-style response — different notch placement).

### LFO

- One `stmlib::CosineOscillator` (or two phase-offset accumulators à la ensemble) sweeping at `0.05 Hz … 8 Hz`. The ensemble LFO accumulator pattern (uint32 phase + `lut_sin`) is the simplest port — see [../dsp/fx/ensemble.h](../dsp/fx/ensemble.h) lines 73–80.
- Modulate `f_c` exponentially around a center frequency: `f_c = f_center * 2^(depth * lfo)`. Exponential modulation keeps notch motion perceptually linear across the audible range. `stmlib::SemitonesToRatio` or a small `lut_exp2` is the right primitive.

### Stage count / "voicings"

Hard-code `N_MAX = 12` and use `carrier_shape` (the OSC SHAPE selector) to pick a voicing:

| `carrier_shape` | Voicing | Notes |
|---|---|---|
| 0 | 4-stage classic (MXR Phase 90 flavor) | 2 notches |
| 1 | 6-stage | 3 notches |
| 2 | 8-stage (Small Stone flavor) | 4 notches |
| 3 | 12-stage deep / barberpole | 6 notches, slowest sweep |

Inactive stages should still be evaluated (with `a = 0`, which makes them identity) **or** the cascade loop should run `N` times where `N` is chosen at the top of the block from `carrier_shape`. The latter is cheaper but introduces a click if `N` changes mid-block — gate the change on a block boundary, same pattern reverb uses for `ReverbType`.

## Control mapping

Following the convention established by `ProcessReverb` and `ProcessEnsemble`, read `previous_parameters_` so the values are coherent with the rendered block:

| Control | Field | Phaser parameter | Range |
|---|---|---|---|
| LEVEL1 pot | `previous_parameters_.raw_level_pot[0]` | MIX (dry/wet) | 0 → 1 |
| LEVEL2 pot | `previous_parameters_.raw_level_pot[1]` | FEEDBACK | 0 → 0.95 (clip well below 1 to keep the cascade stable when the LFO crosses 0) |
| ALGO knob  | `previous_parameters_.raw_algorithm` | CENTER frequency | log-mapped 80 Hz → 4 kHz |
| MOD knob   | `previous_parameters_.modulation_parameter` | DEPTH + RATE | dual-purpose: low half = depth at slow rate, upper half = depth at faster rate, OR split between MOD pot and MOD CV |
| OSC SHAPE  | `previous_parameters_.carrier_shape` | STAGE COUNT / voicing | 0–3 (table above) |

If we want RATE on its own control instead of folding it into MOD, the cleanest option is to take MOD-pot = depth and MOD-CV = rate offset (the parameters carry both `raw_modulation_pot` and `raw_modulation_cv` separately; see [../dsp/parameters.h](../dsp/parameters.h) lines 68–70).

## Implementation steps

1. **`dsp/fx/phaser.h`** — new file. Single header, template-free, no FxEngine dependency. Shape it after [../dsp/fx/ensemble.h](../dsp/fx/ensemble.h):
   - `Init()`, `Reset()`, `Process(float* left, float* right, size_t size)`.
   - `set_amount`, `set_feedback`, `set_center`, `set_depth`, `set_rate`, `set_stages` setters.
   - State: `float z_l[12]`, `float z_r[12]`, `float fb_l`, `float fb_r`, LFO phase(s).
2. **`FEATURE_MODE_PHASER`** — add to the enum in [../dsp/parameters.h](../dsp/parameters.h) (insert before `FEATURE_MODE_META`; `META` must stay last because `Modulator::Init` defaults to it and the UI cycle likely wraps on it).
3. **`Modulator::ProcessPhaser`** — new method in [../dsp/modulator.cc](../dsp/modulator.cc). Pattern after `ProcessEnsemble`:
   - `ApplyAmplification(input, parameters_.channel_drive, aux_output, size, true);`
   - Copy carrier/modulator to `main_output`/`aux_output`.
   - Set phaser params from `previous_parameters_`.
   - `phaser.Process(main_output, aux_output, size);`
   - `Convert(output, main_output, aux_output, 32768.0f, size);`
3. **Dispatch** — add `case FEATURE_MODE_PHASER: ProcessPhaser(input, output, size); break;` to the switch in `Modulator::Process` (~line 905 in [../dsp/modulator.cc](../dsp/modulator.cc)).
4. **Static instance** — declare `static Phaser phaser;` near the existing `static Ensemble ensemble;` (line 61). Call `phaser.Init()` in `Modulator::Init` alongside `ensemble.Init(reverb_buffer)`.
5. **CLAUDE.md update** — add the row to the FeatureMode table in [../CLAUDE.md](../CLAUDE.md) lines 28–39.

## CPU and memory budget

- **State per stage:** 1 float (allpass) + (top-level) 1 float feedback storage. Stereo, 12 stages max: `2 × 12 + 2 = 26` floats ≈ 104 bytes. Negligible.
- **Per-sample cost:** `N` stages × (1 mul + 1 add + 1 state update) × 2 channels + LFO update + coefficient update + dry/wet mix. At `N=12` stereo, 48 kHz, this is well under 5% of the F4 budget — far cheaper than reverb. The dominant cost is the `tan()` evaluation when recomputing the allpass coefficient; cache `a` per block (or per few samples) and recompute only when the LFO has moved by a meaningful amount. `stmlib`'s `Interpolate` against a precomputed `tan(π f / fs)` LUT is the safest path — same approach the dual filter takes.
- **Coefficient smoothing:** recompute `a` once per block from the block-end LFO value, linearly interpolate from the previous block's `a` across the block. This is what `ProcessReverb` does for its parameter sets (`set_amount`, `set_lp`, ...) — same idiom.

## Resources to generate

If we adopt the LUT path for the allpass coefficient (recommended):

- Add a `lut_tan_pi_f_over_fs` table generated from [../resources/](../resources/) Python scripts. The existing `lookup_tables.py` pattern: define an `array.append(...)` with name, length, and the Python expression; rerun the resources step; the `.cc/.h` files regenerate. Mirror the structure of `lut_sin` / `lut_ap_poles`.
- Alternatively, since the center-freq mapping is log-spaced, we can table `a` directly against a normalized control input — the table becomes a single `lut_phaser_coef` of, say, 257 entries with `f` log-mapped over 20 Hz → 16 kHz. One read + linear interp per coefficient update.

## Validation path

1. **Faust prototype** — `pm.allpassFiveStage` / `re.phaser_mono` / `re.phaser_stereo` in the Faust libraries give a known-good reference. Sweep MIX / FEEDBACK / DEPTH / RATE and compare notch placement against the firmware port at the same control values.
2. **Native host build** — wire `ProcessPhaser` into `warps/test/` and drive it with:
   - **Pink noise** → render to wav, take the magnitude spectrum, confirm `N`/2 notches at expected positions, confirm notches sweep at LFO rate.
   - **Impulse + sine sweeps** → confirm phase response (the phaser should be allpass in magnitude when MIX=wet-only).
   - **Stability sweep** at FEEDBACK=0.95 with the LFO crossing zero — listen for self-oscillation; trim the feedback ceiling if it goes unstable.
3. **Hardware** — flash, verify CPU headroom with the existing `meter.h` framework, A/B against a Boss PH-3 or MXR Phase 95 to gut-check the voicing.

## Open questions to resolve before coding

- **MOD knob semantics**: single control for depth+rate (split halfway), or `MOD pot = depth` / `MOD CV = rate`? The latter matches Warps' "two-input" idiom better, but loses RATE control when MOD CV isn't patched.
- **Feedback inversion**: traditional analog phasers invert the feedback signal (giving the characteristic "swirl"). Make this fixed, or expose it as a `carrier_shape` voicing variant?
- **Barberpole mode**: a 6th/12th voicing using a Hilbert-shifted LFO drive (continuous-rising notches) is achievable using the existing `quadrature_transform_` already linked in [../dsp/modulator.cc](../dsp/modulator.cc) for the frequency shifter. Worth scoping as Phase 2 once the standard phaser is in.
