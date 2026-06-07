# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository context

This is a personal fork of Mutable Instruments' Eurorack firmware monorepo, used to develop **"Symbiote" alternative firmwares** for the modules. Stock MI firmwares for every module live in their own top-level directory (`marbles/`, `tides/`, `warps/`, etc.). The active development targets here are the Symbiote variants — they share the MI directory layout, build system and bootloader, and are released as `.wav` SysEx-style flash files installed through each module's normal firmware-update procedure.

Public-facing manuals for the Symbiote firmwares live in a separate blog repo (`~/Projects/blog/blog`, pages under `blog/pages/*-symbiote.md`).

## Building firmware

All modules build out of their own subdirectory using a shared make include (`stmlib/makefile.inc`). The toolchain is the ancient gcc-arm-none-eabi-4_8-2013q4 vendored in-tree — STM32F4 firmware will not build with newer toolchains because of compiler/define drift in the MI code.

The toolchain path is read from the `MI_TOOLCHAIN_PATH` env var (with the in-tree default `gcc-arm-none-eabi-4_8-2013q4/` baked into `stmlib/makefile.inc`). Set `MI_TOOLCHAIN_PATH` to that directory before running `make` if it's not in your environment.

Run make from the **repo root** using `-f <module>/makefile`. For Marbles:

```
make -f marbles/makefile wav      # full build + encode .wav (the normal target)
make -f marbles/makefile          # compile only (ELF/HEX into build/marbles/)
make -f marbles/makefile bin      # produce raw .bin
make -f marbles/makefile clean
make -f marbles/makefile size
make -f marbles/makefile disassemble
```

The `wav` target runs `stm_audio_bootloader/qpsk/encoder.py` with the module-appropriate sample rate / baud parameters baked into each module's `makefile`. That `.wav` is what ships in GitHub releases (tag pattern e.g. `v0.1.0-tides`, `v0.4.0-beta`, etc. — see `git ls-remote --tags`).

Hardware upload targets (`upload`, `upload_jtag`, `upload_combo_jtag_erase_first`, `upload_serial`, ...) drive OpenOCD / stm32loader through `stmlib/programming/`. The module makefile picks one as `UPLOAD_COMMAND`.

## Native tests (Marbles)

Marbles has a host-build test harness at `marbles/test/`:

```
cd marbles/test
make                  # compiles marbles_test natively with g++ (no ARM toolchain)
./marbles_test        # runs the test
make profile          # gperftools CPU profile -> profile.pdf
```

The host build links the same module sources (e.g. `t_generator.cc`, `x_y_generator.cc`, `ramp_extractor.cc`) compiled with `-DTEST`. When adding code that the test harness needs to link against, update the `CC_FILES` list in `marbles/test/makefile`.

## Code layout per module

Each MI module is its own self-contained tree with the same internal pattern: a top-level `<module>.cc` event loop, a `ui.{h,cc}` for buttons/LEDs/encoder, `drivers/` for HAL peripherals, `dsp/` or domain-specific subfolders, `resources/` (lookup tables generated from `resources/lookup_tables*.py`), a `makefile` and its own `bootloader/`. The `stmlib/` directory is the shared MI library (DSP, system, utils, third-party CMSIS/STM SPL) that every module compiles against via the `PACKAGES` list in its makefile.

Code generation: when DSP tables or strings change, run `python resources/lookup_tables.py` (or the module's `make resources` target where exposed) — outputs end up in the module's `resources.cc`/`.h`.

## Active Symbiote work

### Marbles (branch `grids-port`)

Marbles is being extended with a **Grids-mode** that turns the T-section into a Mutable Instruments Grids drum engine and the X-section into a TB-3PO acid sequencer locked to the same clock. Key entry points:

- `marbles/grids/` — ported Grids `PatternGenerator` (used statically), drum/Euclidean LUTs, 16-bit LFSR.
- `marbles/tb3po/tb3po_sequencer.{h,cc}` — algorithm-only port of the Hemisphere TB_3PO applet (no Hemisphere/O&C deps).
- `marbles/random/t_generator.{h,cc}` — owns the Grids engine in the T-section, exposes `set_grids_*` setters and a step-level ramp on `ramps.master` so the X-section can lock to drum steps.
- `marbles/marbles.cc` — wires UI/CV into Grids params, instantiates `TB3PoSequencer`, overwrites X1/X2/X3/Y outputs in `Process()` when Grids mode is active.
- `marbles/settings.{h,cc}` — reuses 2 bytes of `State::padding` for `tb3po_seed`; the seed is committed to flash on the `X DEJA VU OFF → ON|LOCKED` edge.

Conceptual docs (read these before non-trivial changes here):

- `docs/plan.md` — Grids T-section port design.
- `docs/tb3po_port_plan.md` — TB-3PO X-section port design. **Note:** parts of this doc have drifted from the shipped code (e.g. length sharing, transpose units, reset CV channel). When in doubt, trust the code; `marbles/docs/MEMORY.md` reflects what actually shipped.
- `marbles/docs/MEMORY.md` — running implementation summary.

### Tides / Warps Symbiote

`tides2/` and `warps/` host the already-released Symbiote firmwares (alternative oscillator/chaos modes for Tides 2018, and ladder/SVF/reverb algorithms for Warps Parasites). Their public manuals are in the blog repo.

## STM32 / target-specific notes

- All current Symbiote targets are STM32F4 (`FAMILY = f4xx`, F_CPU = 168 MHz, FPv4 hard-float). The makefile.inc also supports f37x/f0xx for legacy modules.
- `APPLICATION_LARGE = TRUE` (in `marbles/makefile`, for example) switches to a larger linker script — keep it on if firmware size grows past the default sector layout.
- Each module's `bootloader/` builds independently. `make wav` only encodes the application binary; the bootloader has its own JTAG upload step.
- The codebase is C++ but compiled with the 2013-era ARM gcc — avoid C++14/17 features in target code. Host-only test code (`marbles/test/`) uses system g++ and is unconstrained.
