# CLAUDE.md (rings)

This file gives module-local guidance for Claude Code when working inside `rings/`. For the repo-wide build/toolchain context (`make wav`, `MI_TOOLCHAIN_PATH`, native tests, etc.) see the root [CLAUDE.md](../CLAUDE.md). Design plans for in-flight Symbiote additions live in [docs/](docs/) — read the relevant plan before non-trivial changes to the affected mode.

## Directory contents

Top-level files (`rings.cc`, `ui.{h,cc}`, `cv_scaler.{h,cc}`, `settings.{h,cc}`, `meter.h`, `resources.{h,cc}`) implement the stock Rings event loop, UI, ADC normalisation, persistent settings, and audio I/O. `makefile` builds the application (`make`, `make wav`); `bootloader/` builds independently.

Subdirectories:

| Path | Role |
|---|---|
| `drivers/` | STM32F4 HAL wrappers (codec, ADC, switches/LEDs, flash, system) |
| `dsp/` | Top-level DSP: `part.{h,cc}` (stock multi-mode resonator engine), `string.{h,cc}` / `fm_voice.{h,cc}` / `resonator.{h,cc}` (per-model voices), `chord_string_synth.{h,cc}` (chord-organ easter egg — the Symbiote target), supporting blocks (`arpeggiator.h`, `chords.h`, `follower.h`, `note_filter.h`, `onset_detector.h`, `plucker.h`, `strummer.h`, `limiter.h`, `string_synth_*.h`, `waves.h`, `patch.h`, `performance_state.h`, `dsp.h`) |
| `dsp/fx/` | Shared FX: `chorus.h`, `delay.h`, `ensemble.h`, `reverb.h`, `fx_engine.h` — consumed by `ChordStringSynth` (and identical in spirit to the Clouds/Warps/Elements `FxEngine<size, format>` DSL) |
| `resources/` | Lookup-table generators (`lookup_tables*.py`) and generated `.cc/.h` — rerun via the module's resource regeneration step when tables change |
| `bootloader/` | Rings' QPSK audio-update bootloader (built separately) |
| `docs/` | Symbiote design plans (e.g. [docs/filter_plaits_vcf_port_plan.md](docs/filter_plaits_vcf_port_plan.md)) |
| `test/` | Native host-build test harness (`rings_test` — see root CLAUDE.md for the native-build pattern) |
| `hardware_design/` | Schematics / PCB |

## Two engines, one module

Rings ships two top-level synthesis engines in the same binary, switched at the UI level:

1. **Stock Rings (`Part`, [dsp/part.cc](dsp/part.cc))** — the modal/sympathetic-string/FM resonator the module is famous for. Owns multiple voice classes (`Resonator`, `String`, `FmVoice`, `ModalVoice`). Selected when the easter-egg flag in `settings.state().easter_egg` is **off**.
2. **Chord string synth (`ChordStringSynth`, [dsp/chord_string_synth.cc](dsp/chord_string_synth.cc))** — the "chord organ" easter egg, an additive/harmonic string-synth engine with arpeggiator, optional filter, and a selectable FX block. Selected when the easter-egg flag is **on**.

Dispatch happens in [rings.cc](rings.cc) — `FillBuffer` checks `settings.state().easter_egg` and routes the audio block to either `part.Process(...)` or `string_synth.Process(...)`.

The easter egg is toggled by a long-press of switch 0 in the UI ([ui.cc:OnSwitchReleased](ui.cc) ~line 258), gated by `cv_scaler_->easter_egg()` — which only returns true when the FREQUENCY pot is near zero (`adc_lp_[ADC_CHANNEL_POT_FREQUENCY] < 0.1f`) and the other lockout conditions are met. This prevents accidental mode flips.

## Chord string synth — operator concepts

### `bank_` ↔ polyphony double-duty

In stock mode, the polyphony toggle cycles `part_->set_polyphony(...)` through `{1, 2, 4, 3}`. In easter-egg mode, **the same UI action calls `string_synth_->set_bank(polyphony)`** (see [ui.cc:OnSwitchReleased](ui.cc) — the polyphony value is forwarded directly as the bank index). So:

| `bank_` | Easter-egg meaning |
|---|---|
| 1 | Default — registrations / harmonics drawbar mode. `patch.brightness` drives `synth.registration_amount`. No filter. |
| 2 | LP filter on the chord bus (currently single `Svf`, target of the [filter port plan](docs/filter_plaits_vcf_port_plan.md)) |
| 3 | BP filter on the chord bus |
| 4 | HP filter on the chord bus |

In bank 2/3/4 the Position pot reading (`performance_state.filter_frequency` / `filter_cv` / `filter_amount`) drives the filter cutoff/CV/amount; in bank 1 it is unused by the filter and the brightness axis drives drawbar registration instead. This pot-repurpose-by-bank is the central UI idiom of the easter egg.

The long-press path forces `polyphony = 3` / `bank_ = 3` as the "comfortable default" landing — call out in the changelog if you change this.

### `fx_type_` — FX block selector

`ChordStringSynth::set_fx(ChordOrganFxType)` chooses one of `{DELAY, CHORUS, REVERB, DELAY_2, ENSEMBLE, REVERB_2}` from [dsp/chord_string_synth.h](dsp/chord_string_synth.h) (enum at the top). The value is persisted as `settings.state().model` and loaded in [ui.cc:Init](ui.cc).

Cross-cutting things to know:

- **`Reverb` and the FX scratch buffer are aliased.** `ChordStringSynth::Init(uint16_t* reverb_buffer)` accepts an external reverb buffer (same convention as Warps' `ReverbProcessor`). The same buffer is reused across `reverb_` invocations; switching to/from a reverb-using `fx_type_` triggers `clear_fx_` to wipe stale state at the top of `Process()`.
- **`clear_fx_` clears both reverb and delay** when the FX *family* changes (`(fx_type % 3) != (previous % 3)` — the three families are delay/chorus/reverb, each with two voicings). See `set_fx` in [dsp/chord_string_synth.h](dsp/chord_string_synth.h).
- **`aux = -aux` after FX.** [dsp/chord_string_synth.cc](dsp/chord_string_synth.cc) (last loop in `Process`) inverts the aux bus before the limiter sums. This is to prevent main-signal cancellation when EVEN gets summed with ODD through normalisation downstream. Any new code that writes to `aux` (e.g. dual-output filters) has to account for this final-stage inversion.
- **Filter cutoff is referenced to the chord tonic, not to a free-running pitch.** `ProcessFilter` computes `f0 = NoteToFrequency(synth.tonic)` and modulates cutoff by `f0 * SemitonesToRatio(120 * (filter_frequency - 0.2))`. New filter topologies should keep this tonic-tracking unless deliberately moving to a free V/oct cutoff (call it out in the plan if you do).
- **Internal sample rate is 48 kHz**, matching the codec rate — no upsampling stage in this engine.

### Performance state plumbing

[dsp/performance_state.h](dsp/performance_state.h) carries the per-block UI/CV snapshot into both engines. The fields `filter_frequency`, `filter_amount`, `filter_cv` are populated by `cv_scaler` from the Position pot/attenueverter/CV when easter-egg mode is active. Adding new modal controls (e.g. the resonance source proposed in the filter port plan) means extending `PerformanceState` and the `cv_scaler.cc` reads — keep this struct lean since it's passed by const-ref every block.

## Active Symbiote work

### `Delay` crunch + time-knob tuning (planned)

[docs/delay_improvement_plan.md](docs/delay_improvement_plan.md) covers fixes for the audibly crunchy and unmusical-to-tune delay in [dsp/fx/delay.h](dsp/fx/delay.h). Root causes ranked by audibility:

- No slew on `delay_time_` — read pointer jumps hundreds of samples per knob nudge. Zipper / click is from this, not from the (sub-sample) LFO modulation. Confirmed via [dsp/fx/fx_engine.h:241-258](dsp/fx/fx_engine.h#L241-L258): `Interpolate(..., LFO, amplitude, scale)` shifts the read by `amplitude * lfo_value`, so `0.5f` amplitude = ±0.5 samples — not enough to mask anything.
- q15 storage (`FORMAT_16_BIT` in `FxEngine`, manual `Compress`/`Decompress` in `Process2`) compounds across feedback passes — quantization noise floor climbs per repeat.
- Bare feedback path: no soft-clip, no DC blocker, no tone shaping. Classic harsh-digital fizz.
- `Process2` reads with the integer overload (`Read(int32_t)`) — full-sample stair-stepping. Currently not in the `fx_type_` dispatcher; plan recommends deleting it.

Suggested fix order (highest ROI first): (1) 1-pole LP in feedback loop, (2) `SoftClip` the feedback multiply, (3) `ParameterInterpolator` on the read offset for tape-style slew, (4) cube the `delay_time_` mapping, (5) DC-block the feedback return, (6) delete `Process2`. Items 1–3 cover ~80 % of the crunch in ~30 lines. Bumping to `FORMAT_32_BIT` is deferred — would double the buffer (32k × 4 = 128 KB) and crowd out the shared reverb buffer on F4.

### Plaits-style VCF on the chord-string-synth filter (planned)

[docs/filter_plaits_vcf_port_plan.md](docs/filter_plaits_vcf_port_plan.md) sketches replacing the block-level `ProcessFilter<>` template in `ChordStringSynth` with a per-sample, parameter-interpolated, soft-clipped, dual-`Svf` filter modelled after the Plaits engine2 VCF ([virtual_analog_vcf_engine.cc](../plaits/dsp/engine2/virtual_analog_vcf_engine.cc)). Key decisions captured there:

- The user-visible feature is musical resonance + drive. The Plaits VCF's `resonance² × resonance² × 48` Q curve and `SoftClip(in*gain)` drive path are ported verbatim.
- Bank 2 → dual-stage LP; bank 3 → single-stage BP; bank 4 → dual LP+HP via `svf_[0].Process<FILTER_MODE_LOW_PASS, FILTER_MODE_HIGH_PASS>`. Each variant writes its mix into both `out` and `aux` (50/50) — preserving today's bus split and the post-FX `aux = -aux` flip.
- Resonance is a new modal control: **bank button held + brightness/cutoff knob captures resonance**. v1 keeps it session-only; persistence per bank in `Settings` is deferred to v2.
- All five tone-shaping coefficients (cutoff, Q, drive gain, stage-2 gain, sub gain unused) ride through `ParameterInterpolator` to avoid the audible block-edge steps the current single-`set_f_q`-per-block path produces on fast cutoff sweeps.
- Staged 5-step commit sequence in the plan keeps the firmware audibly identical until step 4 (resonance pinned to 0 in steps 1–3), so each step can be bisected on hardware independently.
- Open question: "brightness knob" = `patch.brightness` (rewire) or the Position pot currently feeding `filter_frequency` (no rewire). Plan proceeds with the no-rewire interpretation; flagged for confirmation.

## Source-of-truth ordering

When the docs disagree:

1. The C++ in this directory.
2. [docs/](docs/) — Symbiote design plans, kept aligned with what shipped or what is currently planned.
3. Repo-level design docs (`../docs/`) — historical cross-module design intent.
