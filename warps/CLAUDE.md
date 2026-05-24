# CLAUDE.md (warps)

This file gives module-local guidance for Claude Code when working inside `warps/`. For the repo-wide build/toolchain context (`make wav`, `MI_TOOLCHAIN_PATH`, native tests, etc.) see the root [CLAUDE.md](../CLAUDE.md). Design plans for in-flight Symbiote additions live in [docs/](docs/) — read the relevant plan before non-trivial changes to the affected mode.

## Directory contents

Top-level files (`warps.cc`, `ui.{h,cc}`, `cv_scaler.{h,cc}`, `settings.{h,cc}`, `meter.h`, `resources.{h,cc}`) implement the stock Warps event loop, UI, ADC normalisation, persistent settings, and audio I/O. `makefile` builds the application (`make`, `make wav`); `bootloader/` builds independently.

Subdirectories:

| Path | Role |
|---|---|
| `drivers/` | STM32F4 HAL wrappers (codec, ADC, switches/LEDs, flash, system) |
| `dsp/` | Top-level DSP: `modulator.{h,cc}` (the meta-modulator engine + Symbiote feature modes), `oscillator`, `vocoder`, `filter_bank`, `quadrature_*`, `sample_rate_*`, `limiter`, `parameters.h` |
| `dsp/filters/` | `dual_filter.h` — used by `FEATURE_MODE_DUAL_FILTER` |
| `dsp/fx/` | Shared FX scaffolding (`fx_engine.h`) and effects (`reverb.h`, `ensemble.h`, `pitch_shifter.h`) consumed by the Symbiote feature modes |
| `resources/` | Lookup-table generators (`lookup_tables*.py`) and generated `.cc/.h` — rerun via the module's resource regeneration step when tables change |
| `tools/` | `generate_src_filters.py` — regenerates `sample_rate_conversion_filters.h` |
| `bootloader/` | Warps' QPSK audio-update bootloader (built separately) |
| `docs/` | Symbiote design plans (e.g. [docs/erbeverb_port_plan.md](docs/erbeverb_port_plan.md)) |
| `test/` | Native host-build test harness (`warps_test`) — see root CLAUDE.md |
| `hardware_design/` | Schematics / PCB |

## Symbiote feature modes

Warps Symbiote extends the stock meta-modulator with a bank of additional audio effects, selected by `feature_mode_` and dispatched from `Modulator::Process()` in [dsp/modulator.cc](dsp/modulator.cc) (around line 892). The current set, enumerated in [dsp/parameters.h](dsp/parameters.h):

| `FeatureMode` | Process method | Notes |
|---|---|---|
| `FEATURE_MODE_DUAL_FILTER` | `ProcessDualFilter` | Stereo or dual-mono ladder/SVF (`alt_feature_mode_` selects between `DUAL_FILTER` and `STEREO_FILTER`) |
| `FEATURE_MODE_ENSEMBLE` | `ProcessEnsemble` | Shares the FX buffer with `reverb` |
| `FEATURE_MODE_REVERB` | `ProcessReverb` | Griesinger/Dattorro topology, voicing selected by `carrier_shape` |
| `FEATURE_MODE_FREQUENCY_SHIFTER` | `ProcessFreqShifter` | Hilbert-based; pitch-shifter variant commented out in the dispatch |
| `FEATURE_MODE_PHASER` | `ProcessPhaser` | Cascaded-allpass phaser (4/6/8/12 stages selected by `carrier_shape`) |
| `FEATURE_MODE_PITCH_SHIFTER` | `ProcessPitchShifter` | 2-tap SOLA shifter; shares the FX buffer with reverb/ensemble |
| `FEATURE_MODE_DOPPLER` | `ProcessDoppler` | |
| `FEATURE_MODE_DELAY` | `ProcessDelay` | Owns `delay_buffer_` (independent of the FX buffer) |
| `FEATURE_MODE_META` | `ProcessMeta` | Stock MI behaviour — the original meta-modulator (default at `Init`) |

`FEATURE_MODE_META` is the default after `Init`. Mode selection from the UI flows through `Modulator::set_feature_mode()` in [dsp/modulator.h](dsp/modulator.h).

## FX scaffolding

`dsp/fx/fx_engine.h` defines `FxEngine<size, format>` — a compile-time-reserved delay buffer with a `Context` that exposes `Read` / `Write` / `WriteAllPass` / `Interpolate` / `Lp` / `Hp` primitives and two LFOs. Storage is `q15`-compressed at `FORMAT_16_BIT` (the only format Symbiote currently uses for the shared buffer). This DSL is shared with Clouds/Rings/Elements/Plaits and is the right starting point for any new delay-network effect.

Cross-cutting things to know:

- **The reverb buffer is shared with ensemble.** `Modulator::Init()` ([dsp/modulator.cc](dsp/modulator.cc) ~line 44) passes a single `uint16_t* reverb_buffer` into both `reverb.Init(reverb_buffer)` and `ensemble.Init(reverb_buffer)`. The buffer is 32k × 16-bit = 64 KB (`FxEngine<32768, FORMAT_16_BIT>` in [dsp/fx/reverb.h](dsp/fx/reverb.h)). Only one of `FEATURE_MODE_REVERB` / `FEATURE_MODE_ENSEMBLE` is active at a time, so they cannot both run in the same block.
- **Reverb voicing is multiplexed by `carrier_shape`.** `ProcessReverb` reads `parameters_.carrier_shape` and switches the `Reverb` instance between `CAVEMAN`, `RINGS`, `CLOUDS`, `ELEMENTS` via `Reverb::set_type()`. New reverb flavours can be added either as a new `ReverbType` value or as a separate top-level `FEATURE_MODE_*` with its own `Process*` method (preferred when the topology is structurally different from Griesinger).
- **Reverb control mapping** in `ProcessReverb`:
  - `previous_parameters_.raw_level_pot[0]` → `set_amount` (wet)
  - `previous_parameters_.raw_level_pot[1]` → `set_lp` (damping / reverb-specific second axis)
  - `previous_parameters_.raw_algorithm` → `set_diffusion`
  - `previous_parameters_.modulation_parameter` → `set_time`
  - Input gain is hard-coded to `0.2f`.
  Use `previous_parameters_` (not `parameters_`) so the values are coherent with the rendered audio block.
- **`reset_fx` clears `reverb`, `ensemble`, `phaser`, and `pitch_shifter`** at the top of `Modulator::Process()` — set it from the UI when switching modes or freezing-then-releasing.
- **Phaser control mapping** in `ProcessPhaser` (`dsp/fx/phaser.h` is self-contained, no shared FX buffer):
  - LEVEL CVs → `ApplyAmplification(..., raw_level_cv, ..., true)` — input VCAs. `cv_scaler` forces `raw_level_cv = 0.6f` when the jack is unpatched, so audio passes without a patch cable. Same idiom as `DUAL_FILTER` mode.
  - `previous_parameters_.raw_level_pot[0]` → `set_amount` (dry/wet mix)
  - `previous_parameters_.raw_level_pot[1]` → `set_feedback` (×0.9 ceiling; inverted internally for the classic swirl)
  - `previous_parameters_.raw_algorithm` → `set_rate` (LFO speed, exp-mapped 0.1 → 6 Hz)
  - `previous_parameters_.modulation_parameter` → `set_depth` (LFO depth around the center; ±1 octave at full)
  - `parameters_.carrier_shape` → both `set_stages` (4/6/8/12 stage cascade) **and** the per-voicing center frequency via `kVoicingCenter` in `ProcessPhaser`.
  See [docs/phaser_plan.md](docs/phaser_plan.md) for the original design intent.
- **Pitch shifter control mapping** in `ProcessPitchShifter` ([dsp/fx/pitch_shifter.h](dsp/fx/pitch_shifter.h) — 2-tap SOLA shifter that **shares the 32k uint16_t FX buffer** with `reverb`/`ensemble`; mutually exclusive at runtime):
  - LEVEL CVs → `ApplyAmplification(..., raw_level_cv, ..., true)` — input VCAs, same idiom as phaser/dual filter.
  - `previous_parameters_.raw_level_pot[0]` → `set_mix` (dry/wet)
  - `previous_parameters_.raw_level_pot[1]` → `set_feedback` (×0.85; regenerative — and the *only* shimmer engine in `VOICING_SHIMMER`)
  - `previous_parameters_.raw_algorithm` → coarse pitch, mapped (pot − 0.5) × 24 = ±12 semitones
  - `previous_parameters_.modulation_parameter` → fine detune, ±0.5 semitone (≈ ±50 ¢)
  - `parameters_.carrier_shape` (0..3) → voicing: detune / continuous / quantized (5ths/4ths/8ves) / shimmer
  See [docs/pitch_shifter_plan.md](docs/pitch_shifter_plan.md).
- **Internal FX-mode sample rate is 48 kHz.** The reverb hard-codes `0.5f / 48000.0f` and `0.3f / 48000.0f` for its LFOs in [dsp/fx/reverb.h](dsp/fx/reverb.h). The wider Warps pipeline upsamples for modulation modes via `src_up_`/`src_down2_`, but the FX modes operate at the native codec rate.
- **`Convert(output, main, aux, 32768.0f, size)`** is the final fixed-point cast at the end of each `Process*` method. Match its argument layout when adding new modes — `main_output` goes to L, `aux_output` to R.

## Active Symbiote work

### ErbeVerb-flavored FDN reverb (planned)

[docs/erbeverb_port_plan.md](docs/erbeverb_port_plan.md) sketches an 8-line feedback delay network reverb in the spirit of Tom Erbe's ErbeVerb, intended to live alongside the existing Griesinger reverb. Key decisions captured there:

- Build it as a new `FEATURE_MODE_*` (separate file `dsp/fx/fdn_reverb.h`, separate `ProcessFdnReverb`) rather than shoehorning into the existing `Reverb` class — the topology differs structurally and the existing class is already heavily switched on `ReverbType`.
- Reuse the existing 32k × 16-bit shared FX buffer; FDN-8 with diffuser allpasses and a pre-delay fits in ~27k samples.
- Implement smooth SIZE morphing by reserving each line at maximum length and reading via `c.Interpolate(line, size * max_len, scale)` — the FxEngine's fractional reads already give the doppler-on-sweep behaviour for free.
- 16-bit q15 storage is audible on long quiet tails / freeze — consider `FORMAT_32_BIT` for the longest 2 lines if CPU/RAM allows.
- Prototype coefficient ranges in Faust (or against `zita-rev1` as a reference) before iterating on firmware.

## Source-of-truth ordering

When the docs disagree:

1. The C++ in this directory.
2. [docs/](docs/) — Symbiote design plans, kept aligned with what shipped or what is currently planned.
3. Repo-level design docs (`../docs/`) — historical cross-module design intent.
