# CLAUDE.md (marbles)

This file gives module-local guidance for Claude Code when working inside `marbles/`. For the repo-wide build/toolchain context (`make wav`, `MI_TOOLCHAIN_PATH`, native tests, etc.) see the root [CLAUDE.md](../CLAUDE.md). The authoritative running implementation summary for the Symbiote work lives in [docs/MEMORY.md](docs/MEMORY.md) — **read it before any non-trivial change to the Grids T-section or the TB-3PO X-section.**

## Directory contents

Top-level files (`marbles.cc`, `ui.{h,cc}`, `cv_reader.{h,cc}` + `cv_reader_channel.h`, `note_filter.h`, `scale_recorder.h`, `settings.{h,cc}`, `io_buffer.h`, `clock_self_patching_detector.h`, `resources.{h,cc}`) implement the stock Marbles event loop, UI, ADC normalisation, scale storage, and persistent-settings layer. `makefile` builds the application (`make`, `make wav`); `bootloader/` builds independently.

Subdirectories:

| Path | Role |
|---|---|
| `drivers/` | STM32F4 HAL wrappers (ADC, DAC, gates, switches/LEDs, flash, system) |
| `ramp/` | Master/slave ramp generation and ramp extraction for external clocks |
| `random/` | `TGenerator` (T-section), `XYGenerator` (X-section), quantizer, lag processor, output channel, ramp_extractor — also where the **Grids engine plugs into the T-section** |
| `grids/` | Ported Grids pattern generator, drum-map / Euclidean LUTs, 16-bit LFSR (used statically) |
| `tb3po/` | TB-3PO acid sequencer (algorithm-only port of the Hemisphere applet; no O&C deps) |
| `resources/` | Lookup-table generators (`lookup_tables*.py`) and generated `.cc/.h` — rerun via `make resources` when tables change |
| `bootloader/` | Marbles' QPSK audio-update bootloader (built separately) |
| `docs/` | `MEMORY.md` — the implementation-state-of-record |
| `test/` | Native host-build test harness (`marbles_test`) — see root CLAUDE.md |
| `hardware_design/` | Schematics / PCB |

## Recent changes (branch `grids-port`)

The active firmware work turns Marbles into a self-contained drum + bassline machine when its T-section is in Grids mode.

What shipped (full detail in [docs/MEMORY.md](docs/MEMORY.md)):

- New `T_GENERATOR_MODEL_GRIDS = 5` reachable via long-press on `T MODEL`. Reuses the old `THREE_STATES` slot.
- Grids drum engine on T1/T2/T3 (BD/SD/HH). Sub-mode toggled by `T DEJA VU` button: OFF = Drums (2D map), ON/LOCKED = Euclidean (shared length on `DEJA VU LENGTH`).
- Per-voice density driven by `RATE`/`BIAS`/`JITTER` CVs (fixed 0.5 base, additive); `BIAS`/`JITTER` knobs drive the drums map X/Y in Drums sub-mode and are unused in Euclidean.
- Bipolar `DEJA VU` knob: CCW → drum chaos (Drums) or T2 (SD) fills (Euclidean); CW → pattern rotation (Euclidean).
- TB-3PO acid sequencer (`tb3po/tb3po_sequencer.{h,cc}`) on the X-section whenever Grids is active. Clocked from `ramps.master` (step ramp wraps), 16th-note rate, with per-sample slide IIR.
- TB-3PO controls: `X SPREAD` knob+CV → density; `X BIAS` knob+CV → transpose (±18 semitones, CV is 1V/oct); `X STEPS` knob+CV → length 1..32; `X DEJA VU` switch → seed lock/reseed/flash-commit; `X SCALE` → scale lookup. `X RANGE` unused.
- Reset: `DEJA VU` CV rising edge resets both T and X to step 0 in Grids mode.
- Seed persistence via 2 reused bytes of `State::padding` (`tb3po_seed`); committed to flash on the `X DEJA VU OFF → ON|LOCKED` edge, restored on boot.
- In-scale-only pitch selection in TB-3PO: walks `Scale::degree[]` directly and filters by weight at `set_scale()` time so chromatic passing tones never sound on the diatonic / pentatonic / raag presets.

## Architecture pointers

The audio loop in `marbles.cc::Process()` is the central wiring hub. Block-level work runs first (mode detection, knob/CV reads, per-block parameter feed into `t_generator` and `tb3po`), then `t_generator.Process()` writes ramps and gate signals, then the X-section sample loop runs `xy_generator.Process()` and converts voltages → DAC codes. In Grids mode, `xy_generator` still runs so its `ramp_extractor` / `random_sequence` state stays coherent across mode switches, but its X1/X2/X3/Y outputs are overwritten with Grids clock / TB-3PO pitch / gate / accent before the DAC write loop.

Cross-cutting things to know when touching this code:

- **Grids `PatternGenerator` is used as a fully static class.** It is `Init(0)`'d once in `TGenerator::Init()`; resets are forwarded from t-section reset.
- **Step-level clock on `ramps.master`.** In Grids mode this ramp is rewritten to cycle 0→1 over 6 master_phase wraps (2 Grids steps = 8th-note rate). marbles.cc watches this ramp to fire TB-3PO `Tick` / `TickHalfCycle`. Do not assume `ramps.master == master_phase_` in Grids mode.
- **`xy_clock_source` is overridden after the self-patching detector.** In Grids mode all three X channels are forced to follow `ramps.master` (or the external ramp if patched), not the individual drum gates.
- **`t_deja_vu` is repurposed as a sub-mode selector** in Grids mode. Do not call `t_generator.set_deja_vu()` / `set_length()` when Grids is active.
- **Seed lifecycle is edge-triggered.** `prev_x_deja_vu` must be initialised from `state.x_deja_vu` at boot to avoid a spurious reseed on the first audio block.
- **TB-3PO RNG shares Grids' `GridsRandom` LFSR.** The LFSR state must be saved/restored around any TB-3PO regeneration so PatternGenerator's perturbation stream stays deterministic.
- **`State::padding` is partially consumed** by `tb3po_seed` (uint16). 3 bytes of padding remain — preserve the layout for ABI stability across firmware updates.
- **`xy_generator.Process()` cost.** It is intentionally left running in Grids mode (~40% of X-section CPU per `Process()` comments) for state coherence; short-circuiting it is a deferred optimisation if timing pressure shows up.

## Source-of-truth ordering

When the docs disagree:

1. The C++ in this directory.
2. [`docs/MEMORY.md`](docs/MEMORY.md) — kept aligned with what shipped.
3. Repo-level design docs (`../docs/plan.md`, `../docs/tb3po_port_plan.md`) — historical design intent; parts have drifted from the shipped code (length sharing, transpose units, reset CV channel). Trust them for *rationale*, not for *current behaviour*.