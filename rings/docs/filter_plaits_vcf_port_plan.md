# Rings — Plaits-style VCF port plan for `ChordStringSynth`

Status: planning. Owner: Leandro. Target: Symbiote / chord-string-synth easter egg.

## Goal

Replace the block-level `ProcessFilter<>` in
[chord_string_synth.h](../dsp/chord_string_synth.h) with a per-sample,
parameter-interpolated, soft-clipped, dual-`Svf` filter modelled after the
Plaits engine2 VCF
([virtual_analog_vcf_engine.h](../../plaits/dsp/engine2/virtual_analog_vcf_engine.h),
[virtual_analog_vcf_engine.cc:60-128](../../plaits/dsp/engine2/virtual_analog_vcf_engine.cc#L60-L128)).
The user-visible upgrade is musical resonance, drive, and a smoother
cutoff sweep; the operator-visible upgrade is that the existing bank
button picks a topology *variant* instead of just a filter mode.

> Note. The original request referenced `plaits/dsp/engine/virtual_analog_engine.cc`,
> which has no VCF (it's the 2-VCO+sync engine). The actual model is the
> engine2 VCF engine — confirmed by the user during planning.

## Reference: Plaits VCF, condensed

[virtual_analog_vcf_engine.cc:60-128](../../plaits/dsp/engine2/virtual_analog_vcf_engine.cc#L60-L128) shows:

- Two `stmlib::Svf` in series: `svf_[0]` runs LP+HP simultaneously,
  `svf_[1]` is an optional 2nd LP stage cascaded after `svf_[0]`'s LP.
- All five tone-shaping parameters are smoothed with
  `ParameterInterpolator` across the block:
  `previous_cutoff_`, `previous_q_`, `previous_gain_`,
  `previous_stage2_gain_`, `previous_sub_gain_`.
- `set_f_q<FREQUENCY_FAST>` is called *per sample* on both SVFs. Cutoff
  is clamped to `0.25` (Nyquist/2 at 48 kHz).
- Resonance is shaped sharply:
  `resonance = 2.667 * max(|harmonics-0.5| - 0.125, 0)`,
  `q = resonance² * resonance² * 48`. Real `set_f_q` arg is `0.5 + q`
  (so noise floor Q never collapses).
- The 2nd LP stage Q is `0.5 + 0.025*q` (much gentler) and is mixed in
  as a *difference* term: `lp += stage2_gain * (SoftClip(svf_[1].Process<LP>(lp)) - lp)`.
- Input is pre-drive: `SoftClip((osc + sub*sub_gain) * gain)`; the LP
  output is soft-clipped again before the stage-2 mix. HP is soft-clipped
  with the same `gain`.
- HP goes to `aux`, LP to `out`.

## Current state

[chord_string_synth.h:168-199](../dsp/chord_string_synth.h#L168-L199) shows
`ProcessFilter<FilterMode>`:

- Block-level: one `set_f_q<FREQUENCY_FAST>` per call.
- LP variant cascades by recursing through the same SVF
  (`filter_.Process<LP>(filter_.Process<LP>(x))`), HP/BP single-pass.
- Resonance is hard-coded: `0.75` for LP, `2.0` for HP/BP.
- Cutoff: `f0 * SemitonesToRatio(120 * (filter_frequency - 0.2))` where
  `f0 = NoteToFrequency(synth.tonic)`. (Same shape as Plaits, just
  centered on the chord's tonic, not a free-running note.)
- Modulation: `(env + filter_cv) * filter_amount` when envelope is
  active, otherwise `filter_cv * filter_amount`. Added to cutoff and
  clamped to `[0, 1]`.
- Output: fills both `out` and `aux` with `0.5 * filter_out` — i.e.
  always 50/50 across the dual bus.

Dispatcher in
[chord_string_synth.cc:173-181](../dsp/chord_string_synth.cc#L173-L181):

```cpp
if (bank_ == 2)      ProcessFilter<FILTER_MODE_LOW_PASS>(...);
else if (bank_ == 4) ProcessFilter<FILTER_MODE_HIGH_PASS>(...);
else if (bank_ == 3) ProcessFilter<FILTER_MODE_BAND_PASS>(...);
```

`bank_ == 1` is the registration-amount-driven harmonics mode; `bank_ == 0`
is unfiltered. Those two stay untouched in this port.

## Target design

### Topology by bank (user-chosen)

| `bank_` | Topology                                                                  | Out bus           | Aux bus           |
|---------|---------------------------------------------------------------------------|-------------------|-------------------|
| 2       | Plaits dual-stage LP (svf[0] LP → soft-clip → svf[1] LP mixed by stage2)  | 0.5 · (LP + LP)   | 0.5 · (LP + LP)   |
| 3       | Single-stage BP (svf[0] BP, same drive + soft-clip path, no stage 2)      | 0.5 · BP          | 0.5 · BP          |
| 4       | Plaits dual output: svf[0] LP & HP simultaneously, soft-clipped each      | 0.5 · (HP + LP)   | 0.5 · (HP + LP)   |

Routing rationale: the user asked to keep the existing 50/50 split. So
all variants sum their topology into a single mono pre-mix and write
the same scaled buffer to both `out` and `aux`. The
[chord_string_synth.cc:228-230](../dsp/chord_string_synth.cc#L228-L230)
post-loop that flips `aux = -aux` is preserved, but with identical
content on both buses the inversion just produces an in-phase cancel
when the downstream limiter sums them — same as today.

> Open question (Q1 below): if the user later wants stereo flavor, the
> easiest unlock is `bank_ == 4` putting LP→out and HP→aux verbatim
> (Plaits routing). The plan stages this as a follow-up.

### Parameter mapping

User decision: **the brightness knob remains the cutoff control; the
bank button held + brightness knob = resonance.**

Two interpretations need confirmation (see Q2):

- **Option A** (likely intent): the *position-axis* knob currently routed
  into `synth.filter_frequency` keeps its job (it's the de-facto
  "filter cutoff" knob in the chord-string mode). "Brightness" is the
  shorthand for it.
- **Option B** (literal): re-route `patch.brightness` into
  `synth.filter_frequency` and free Position for something else (or
  for the resonance modal capture, which would be more ergonomic since
  the bank button + Position is already wired together).

Plan proceeds with **Option A** so the existing CV scaler / patch
plumbing isn't disturbed. Option B is a one-line swap in
[chord_string_synth.cc:92-96](../dsp/chord_string_synth.cc#L92-L96).

Final mapping (independent of A vs B):

| Plaits VCF param  | Source in Rings                                                |
|-------------------|----------------------------------------------------------------|
| cutoff            | `synth.filter_frequency` (pot) + `filter_cv*filter_amount` + env via `filter_amount` (existing formula, kept) |
| resonance         | new `filter_resonance_` field on `Synth`, captured modally     |
| stage2_gain       | derived: `1 - (resonance - 0.4) * 4`, clamped `[0,1]`          |
| drive gain        | derived: `(resonance - 0.7) + 0.85`, clamped `[0.7 - r²·0.3, 1]` |
| sub_gain          | not used (no sub oscillator on Rings string voices)            |

Resonance is a 0..1 control surface; the actual `q` follows the same
sharp curve Plaits uses (`q = r⁴ * 48`).

### Per-sample render loop

Sketch (LP variant for bank 2):

```cpp
ParameterInterpolator cutoff_mod(&previous_cutoff_, cutoff_target, size);
ParameterInterpolator q_mod      (&previous_q_,       q_target,       size);
ParameterInterpolator gain_mod   (&previous_gain_,    gain_target,    size);
ParameterInterpolator stage2_mod (&previous_stage2_,  stage2_target,  size);

for (size_t i = 0; i < size; ++i) {
  const float f  = std::min(cutoff_mod.Next(), 0.25f);
  const float q  = q_mod.Next();
  const float g  = gain_mod.Next();
  const float s2 = stage2_mod.Next();

  svf_[0].set_f_q<FREQUENCY_FAST>(f, 0.5f + q);
  svf_[1].set_f_q<FREQUENCY_FAST>(f, 0.5f + 0.025f * q);

  const float in_sample = SoftClip((out[i] + aux[i]) * g);
  float lp = svf_[0].Process<FILTER_MODE_LOW_PASS>(in_sample);
  lp = SoftClip(lp * g);
  lp += s2 * (SoftClip(svf_[1].Process<FILTER_MODE_LOW_PASS>(lp)) - lp);

  out[i] = lp * 0.5f;
  aux[i] = lp * 0.5f;
}
```

BP (bank 3) and the dual LP+HP path (bank 4) drop `svf_[1]` and use
`Process<FILTER_MODE_BAND_PASS>` or
`Process<FILTER_MODE_LOW_PASS, FILTER_MODE_HIGH_PASS>(in, &lp, &hp)`
respectively.

Mixing `out + aux` into a single mono pre-filter sample preserves the
content from both odd and even chord voices that
[chord_string_synth.cc:165-172](../dsp/chord_string_synth.cc#L165-L172)
just rendered to separate buses. This is identical to the implicit
behaviour of the current `filter_in_buffer_[i] = out[i] + aux[i]` line.

## Implementation steps

1. **`chord_string_synth.h`**
   - Add to `Synth`: `float filter_resonance;` (0..1, default 0).
   - Add to `ChordStringSynth` private state:
     - `stmlib::Svf svf_[2];` (replaces single `filter_`)
     - `float previous_cutoff_, previous_q_, previous_gain_, previous_stage2_;`
   - Delete `filter_in_buffer_` / `filter_out_buffer_` (no longer needed).
   - Replace the `ProcessFilter<mode>` template with three explicit
     methods (or one template parameterised by bank variant — taste
     call; explicit is simpler to read and the codegen is identical).
   - Keep `NoteToFrequency` and the current cutoff formula, factor into
     a helper `ComputeCutoff(envelope)`.

2. **`chord_string_synth.cc`**
   - `Init`: zero the four `previous_*` fields, init both SVFs.
   - In `Process`, set `synth.filter_resonance = performance_state.filter_resonance`
     (new field, see step 4) when `bank_ >= 2`.
   - Replace the three-way `ProcessFilter` dispatch with the three new
     methods; pass `envelope_value * 0.15f` as today.

3. **Filter math — pre-loop targets** (per bank, computed once):
   - `cutoff_target = f0 * SemitonesToRatio(120 * (filter_frequency - 0.2))` (unchanged) `+ modulation`.
   - `resonance_sqr = filter_resonance * filter_resonance`
   - `q_target = resonance_sqr * resonance_sqr * 48`
   - `stage2_target = clamp(1 - (filter_resonance - 0.4) * 4, 0, 1)`
   - `gain_target = clamp((filter_resonance - 0.7) + 0.85, 0.7 - resonance_sqr*0.3, 1)`

4. **Performance state / CV plumbing**
   - Add `float filter_resonance;` to `PerformanceState`.
   - In `cv_scaler.cc`, when the bank button is *held*, route the
     cutoff pot reading into `performance_state.filter_resonance`
     instead of `filter_frequency`. When released, latch the captured
     value; on subsequent unheld turns the pot resumes controlling
     cutoff.
   - This mirrors how Marbles' Symbiote already does modal pot capture
     (see `marbles/ui.cc` for the held-button latch pattern).

5. **UI — `rings/ui.{h,cc}`**
   - Add a `bank_button_held_` boolean updated in the button scan.
   - While held, blink/dim the bank LED to indicate "resonance edit
     mode" (cheap visual feedback; reuses existing LED slots).
   - Expose `bank_button_held()` to `cv_scaler` (or push the captured
     value through a getter — whichever fits the existing wiring style).

6. **Settings persistence (optional, recommend deferring)**
   - `filter_resonance` could be persisted per-bank in `Settings` like
     Marbles' `tb3po_seed` (see CLAUDE.md "Marbles" notes). For v1 keep
     it session-only — easier to ship, easier to A/B.

7. **Build/test**
   - `cd rings && make` (uses `MI_TOOLCHAIN_PATH` per repo conventions).
   - `make size` — the dual SVF + 4 interpolators add ~negligible RAM
     but the per-sample `set_f_q` doubles trig/recip work compared to
     the block-level call. Budget impact below.
   - No native test harness in `rings/` (unlike `marbles/test/`); test
     on hardware.

## Performance budget

Plaits VCF runs the same per-sample SVF coefficient update inside one
voice's render. In `ChordStringSynth` the filter sits *after* 4 chord
voices have summed into out+aux, so the per-sample loop length is the
same `kMaxBlockSize` — the extra cost is one `set_f_q` (svf[0]) per
sample for BP/HP-mode, two per sample for LP-mode (svf[0] + svf[1]),
plus the `SoftClip` calls. `stmlib::Svf::set_f_q<FREQUENCY_FAST>` is a
multiply + one approx-sin call internally. On STM32F4 @ 168 MHz with
FPU this is well inside budget at 48 kHz / kMaxBlockSize=16 (the
exact ratio Plaits ships at), but verify with `make size` and ideally
a scope-on-trigger CPU pin.

## Phase / topology caveats

- The
  [chord_string_synth.cc:228-230](../dsp/chord_string_synth.cc#L228-L230)
  inversion of `aux` happens *after* the FX block, not after the
  filter. So whatever you write to `aux` here gets fed into delay /
  chorus / reverb in-phase with `out`, and then inverted only at the
  very end before the limiter sums them. With identical 50/50 content
  on both buses, the final inversion subtracts, which is the same
  cancellation behaviour the current `ProcessFilter` produces. No
  regression.
- If we later flip `bank_ == 4` to true Plaits routing (LP→out, HP→aux),
  the inversion becomes audible polarity flip on the HP path through
  FX — has to be flagged in the changelog.

## Risks

- **CPU**: addressed above; verify on hardware.
- **Drive gain stage**: `SoftClip * gain` will increase perceived
  loudness vs. the current clean LP. Mitigate with the same
  `gain → 0.7..1.0` curve Plaits uses; if it's still too hot, scale the
  pre-filter sum by `0.5` before drive (currently it's the raw sum).
- **Cutoff smoothing depth**: `ParameterInterpolator` is linear across
  the block. If the user is modulating filter CV at audio rate the
  block-rate target update is still the bottleneck — same as Plaits.
- **State reset on bank switch**: `clear_fx_` already resets reverb/delay
  buffers on FX-type change; do the same for `previous_*` fields when
  `bank_` changes between filter banks, otherwise a stale cutoff/Q
  causes an audible glide on switch.

## Open questions

- **Q1.** Should `bank_ == 4` eventually adopt the true Plaits stereo
  routing (LP/HP on separate buses)? Currently planned as 50/50 per the
  user's answer; revisit after on-hardware listen.
- **Q2.** "Brightness knob = cutoff": is this the Position pot (current
  source for `filter_frequency`, Option A) or `patch.brightness`
  (Option B, requires re-wire)? Plan proceeds with Option A. If Option
  B is intended, also pick which pot becomes the resonance modal
  source.
- **Q3.** Modal LED feedback: dim/blink the bank LED while held, or
  flash a different LED? Defer to UI taste.
- **Q4.** Persist `filter_resonance` per-bank in `Settings`? Recommend
  v2.

## Out of scope

- Stock Rings (non-easter-egg) `Part`/`StringSynthPart`/`FMVoice`
  filter chains — unchanged.
- Adding new FX modes.
- Changing CV scaler calibration.

## Suggested commit sequence

1. Header: replace `filter_` with `svf_[2]`, add `previous_*`, add
   `filter_resonance` to `Synth`. Compile clean.
2. `.cc`: rewrite the three filter paths, dispatcher unchanged. Hardcode
   resonance to 0 first — should sound near-identical to today.
3. `PerformanceState` + `cv_scaler.cc`: introduce
   `filter_resonance` field plumbed from a constant 0. Still no
   audible change.
4. UI: bank-button-held capture; resonance becomes user-controllable.
5. Tuning pass on `gain` / `stage2_gain` curves to taste.
