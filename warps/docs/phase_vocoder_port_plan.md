# Phase Vocoder pitch shifter for Warps — plan

## 1. Diagnose the "wrong ratio" symptom first

Before any algorithm work, the immediate compile-time problem in the current file is straightforward:

`RESERVE_SIZE` and `ENGINE_SIZE` are referenced on [../dsp/fx/pitch_shifter.h](../dsp/fx/pitch_shifter.h) (lines 46, 54, 98, 104) but **were originally defined nowhere in the codebase**. Either the file failed to compile (which means whatever was being heard wasn't this code), or the macros were coming from a stale local definition that gave a `size_` value not matching the actual buffer reserved by `FxEngine`. Concretely:

- `size_` is the *window length in samples* used to scale `phase_` → read offset.
- The actual buffer length is `ENGINE_SIZE`, but the reserved region (where reads/writes happen) is `2 * RESERVE_SIZE`.
- If `size_` (set from `RESERVE_SIZE` or `set_size`) is larger than the actual reserved region, `c.Interpolate(line, phase * size_)` reads outside the reserved area into other delay-line memory or past the buffer wrap — producing garbage that *sounds like pitch shifting at the wrong ratio* with periodic glitches.

**This alone could explain "wrong ratio + sounds bad"** without touching the algorithm. The Clouds-tested values are now defined (`#define RESERVE_SIZE 2047`, `#define ENGINE_SIZE 4096`) — re-test before committing to the full PV. If the wobble disappears and a +12-semitone test now gives a true octave, the *only* remaining issue is naive-algorithm artifacts, and Tier-1 polishing may be enough to save a week of work.

I'd do this as a 30-minute test before proceeding to Phase 1.

## 2. Phase vocoder design — target parameters

Assuming the naive algorithm is insufficient:

| Parameter | Value | Rationale |
|---|---|---|
| FFT size `N` | 256 | 5.3 ms acoustic window @ 48 kHz; good freq resolution (~187 Hz/bin) for pitch shifting; halves CPU vs. 512 |
| Hop size `H` | 64 | 4× overlap (standard for PV); 1.3 ms hop period |
| Window | Hann (squared on synthesis for COLA-compliant 4× overlap reconstruction) | Standard; alt: sqrt-Hann on both sides |
| FFT library | `arm_rfft_fast_f32` from CMSIS-DSP | Already vendored, hand-tuned for Cortex-M4F; ~80 µs per 256-pt RFFT |
| Pitch-shift method | Direct bin-shifting with phase-accumulator correction (NOT time-stretch + resample) | Lower latency, simpler memory, well-suited to real-time |
| Bin interpolation | Linear between adjacent shifted bins, with phase rotation | Good enough at N=256; cubic would buy little |
| Stereo handling | Two independent PV instances sharing window/twiddle LUTs | Simpler than M/S decomposition; CPU cost is fine |

### Algorithm (per channel, per hop)

```
1. Append H new input samples to input_ring (length N)
2. window = input_ring * hann_lut                          [N multiplies]
3. spectrum = rfft(window)                                  [arm_rfft_fast_f32, N=256]
4. for each bin k in [0, N/2]:
      mag[k] = |spectrum[k]|
      phase[k] = arg(spectrum[k])
      delta_phase = phase[k] - prev_phase[k]
      prev_phase[k] = phase[k]
      true_freq[k] = bin_freq(k) + unwrap(delta_phase - k * 2π * H/N) / (2π * H/N) * bin_freq(N=1)
5. shifted spectrum: for each output bin j:
      source_bin = j / ratio
      [linear interpolate mag and phase between floor/ceil source bins]
      output_phase_accum[j] += true_freq[source_bin] * ratio * 2π * H / sample_rate
      out_spectrum[j] = mag_interp * exp(i * output_phase_accum[j])
6. windowed_output = irfft(out_spectrum) * hann_lut         [N multiplies]
7. overlap-add windowed_output into output_ring at offset 0; advance output_ring by H
8. emit H samples from output_ring tail to *left/*right
```

Step 5 is the heart of it. The `output_phase_accum` keeps phases continuous across hops at the *output* bin grid, regardless of input phase drift — this is what makes the PV phase-locked instead of randomly de-tuned-sounding.

### Memory budget (per channel)

| Buffer | Size | Bytes |
|---|---|---|
| `input_ring` (N floats) | 256 | 1024 |
| `output_ring` (N floats, OLA accumulator) | 256 | 1024 |
| `prev_phase` (N/2 + 1 floats) | 129 | 516 |
| `output_phase_accum` (N/2 + 1 floats) | 129 | 516 |
| `fft_scratch` (N complex = 2N floats) | 512 | 2048 |
| **Per-channel total** | | **~5.1 KB** |
| Stereo total | | **~10.2 KB** |
| Hann LUT (shared) | 256 floats | 1024 |
| CMSIS twiddle (const, flash) | — | ~2 KB flash |

Fits comfortably as class members on the existing `Modulator`. The 64 KB FX-shared buffer isn't needed for this — leave it to reverb/ensemble.

### CPU budget estimate at 48 kHz stereo

| Per-hop work | Cycles | Cycles/sec @ 48k/64 hop |
|---|---|---|
| 2× windowing (analysis + synthesis) per ch × 2 ch | ~4k | 3.0 M |
| 2× FFT per ch × 2 ch (analysis + synthesis) | ~120k | 90 M |
| Bin processing (mag, phase, accum) per ch × 2 ch | ~30k | 22 M |
| OLA + output ring | ~3k | 2.3 M |
| **Total** | | **~117 M cycles/sec** |
| **% of F4 @ 168 MHz** | | **~70%** |

That's tight — likely too tight to coexist with the rest of the Modulator's overhead (oversampling, CV smoothing, UI). Mitigation options ranked by impact:

1. **Drop to mono PV with stereo passthrough** (process mid channel only, copy to L/R) → halves to ~35%. Sacrifices stereo width on the wet signal.
2. **Process M/S separately, but downsample to 24 kHz for PV** → halves FFT cost again; high-frequency content above 12 kHz aliases.
3. **N=128, hop=32** → halves FFT cost, but pitch resolution becomes ~375 Hz/bin (poor for bass content).
4. **Skip every other analysis frame and reuse spectrum** → 50% CPU savings, light artifact increase.

**Recommended:** start with N=256, dual-mono full-rate, validate audibly, then apply (1) or (4) only if hardware shows CPU overruns. CMSIS FFT is fast enough on F4 that the published numbers tend to be conservative.

## 3. Phased build plan

### Phase 0 — Triage the existing naive shifter (30 min) — *macros now defined*
- ✅ Define `RESERVE_SIZE`/`ENGINE_SIZE` in [../dsp/fx/pitch_shifter.h](../dsp/fx/pitch_shifter.h).
- Temporarily uncomment the `ProcessPitchShifter` wiring in [../dsp/modulator.h](../dsp/modulator.h) and [../dsp/modulator.cc](../dsp/modulator.cc) (the `ProcessPitchShifter` body is wrapped in `/* ... */` around line 454; the dispatch is commented around line 919).
- Audition a +12 / -12 sine test through the host test harness ([../test/warps_test.cc](../test/warps_test.cc)).
- **Decision gate:** if the wobble is gone and ratios are correct, the PV may not be needed. If wobble persists on melodic material, proceed.

### Phase 1 — Reference implementation off-board (1-2 days)
- Write the PV in Python with NumPy/SciPy (or in Faust — but Python is easier for debugging phase math).
- Test signals: pure sine sweeps (verify exact ratio), two-sine polyphonic (verify both shift correctly), click trains (verify transient handling), a vocal sample (verify naturalness).
- Tune the window/hop/interpolation choices against artifacts you actually hear.
- This is where you lock in the algorithm before any STM32 work. Mistakes here cost minutes in Python and hours on hardware.

### Phase 2 — CMSIS-DSP integration (half day)
- Add the required CMSIS-DSP source files to Warps' `PACKAGES` list in [../makefile](../makefile). For RFFT you need at minimum:
  - `DSP_Lib/TransformFunctions/arm_rfft_fast_f32.c`
  - `DSP_Lib/TransformFunctions/arm_cfft_f32.c` and its radix dependencies
  - `DSP_Lib/CommonTables/arm_common_tables.c` (twiddle tables)
- These live at [../../stmlib/third_party/STM/CMSIS/DSP_Lib/](../../stmlib/third_party/STM/CMSIS/DSP_Lib/).
- Verify a 256-pt RFFT/IRFFT roundtrip in the existing `warps_test` host build harness — confirm SNR > 120 dB to rule out integration errors.
- **Compile-size check:** CMSIS adds ~30 KB to flash. Warps already uses `APPLICATION_LARGE = TRUE`, so headroom should be there, but `make size` after to confirm.

### Phase 3 — Embedded port (3-5 days)
- New file `warps/dsp/fx/phase_vocoder.h` (don't repurpose `pitch_shifter.h` — keep the naive one as a fallback/comparison voicing).
- Class `PhaseVocoder` with:
  - `void Init()` — set up FFT instance, zero buffers
  - `void Process(float* left, float* right, size_t size)` — same signature as the existing `Reverb::Process` for consistency
  - `void set_ratio(float ratio)` — accepts linear ratio; caller does `SemitonesToRatio` upstream
  - Internal hop scheduler: accumulator counts incoming samples; trigger a PV step every 64 samples.
- Two `PhaseVocoder` instances (L, R) in `Modulator`, sharing a single static Hann LUT in `resources/`.

### Phase 4 — UI & mode wiring (half day)
- Decision: new `FEATURE_MODE_PITCH_SHIFTER`, or replace the naive `ProcessPitchShifter` body inside the existing freq-shifter slot? **Recommend a new mode** — keeps the existing freq shifter intact and makes the user-facing distinction clear (freq shift vs. pitch shift are sonically very different).
- Add `FEATURE_MODE_PITCH_SHIFTER` to [../dsp/parameters.h](../dsp/parameters.h) (the `FeatureMode` enum around lines 44–54) and a `case` in the dispatch switch in [../dsp/modulator.cc](../dsp/modulator.cc) (around lines 892–944).
- Control mapping (mirrors the commented `ProcessPitchShifter` body around lines 467–475 of [../dsp/modulator.cc](../dsp/modulator.cc)):
  - `raw_algorithm` → semitones (−12 .. +13 range, 25 steps)
  - `carrier_shape` (OSC SHAPE) → continuous / 1-semitone quantize / octave quantize
  - `raw_level_pot[0]` → wet/dry
  - `raw_level_pot[1]` → fine detune (±50 cents) or formant/spectral tilt — pick whichever you find more useful in testing
  - `modulation_parameter` → unused or reserved for spectral gate threshold

### Phase 5 — Validation & polish (2-3 days)
- Host test: drive the PV through `warps_test` with the same Python reference signals; bit-compare output where possible.
- Hardware test: scope CPU utilization (the existing modulator has a meter / overrun detection — verify no drops at any combination of ratio / fine-detune / wet-mix).
- Transient handling: if vocal/percussive material sounds metallic on transients, add **phase locking** (a.k.a. "rigid phase locking", Laroche & Dolson 1999) — when a bin's magnitude is much higher than its neighbors, lock the neighbor phases to it. This is ~30 lines of code, ~5% CPU, and a huge perceptual win on real-world material.
- Optional: stereo-aware processing (process M/S instead of L/R) for better stereo image preservation on stereo input.

## 4. Risks and exit ramps

- **CPU overrun at full quality.** If Phase 3 hardware tests show drops, the fastest mitigations are: mono PV (Phase 4 fallback), then frame-skipping (process every other analysis frame), then N=128 if absolutely required. Decision point: end of Phase 3.
- **Flash size overflow.** CMSIS-DSP plus the FFT tables plus Warps' existing code may not fit. If `make size` after Phase 2 shows < 4 KB free, strip CMSIS to only the FFT functions actually called (manual `.c` file selection) instead of pulling all of `TransformFunctions`. Decision point: end of Phase 2.
- **Algorithm bugs at the embedded port.** Phase math is the #1 source of subtle PV bugs. The Phase 1 Python reference is your oracle — if hardware output diverges from Python output for the same input, the bug is in the port, not the algorithm. Don't try to debug a PV from first principles on hardware.
- **Latency** (~7 ms) may be perceptible when dry/wet mixing into a fast modular patch. If a near-zero-latency option is wanted, keep the naive shifter as a separate `carrier_shape` voicing for transient-heavy material.

## 5. Embedded constraints reference

For convenience when implementing:

| Constraint | Value | Source |
|---|---|---|
| Per-`Process()` block size | 96 samples (`kMaxBlockSize`) | [../dsp/modulator.h](../dsp/modulator.h) line 53 |
| FX-mode native sample rate | 48 kHz | [../dsp/fx/reverb.h](../dsp/fx/reverb.h) LFO init |
| Available float scratch | 3 × 96 floats (`buffer_[3][kMaxBlockSize]`) | [../dsp/modulator.h](../dsp/modulator.h) line 404 |
| Shared FX delay buffer | 32k × 16-bit (used by reverb/ensemble) | [../dsp/fx/reverb.h](../dsp/fx/reverb.h) line 255 |
| CPU | STM32F407 @ 168 MHz, FPv4 hard float | repo-root [../../CLAUDE.md](../../CLAUDE.md) |
| CMSIS-DSP location | [../../stmlib/third_party/STM/CMSIS/DSP_Lib/](../../stmlib/third_party/STM/CMSIS/DSP_Lib/) | vendored, not currently in Warps' `PACKAGES` |
