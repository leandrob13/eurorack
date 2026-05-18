# Porting an ErbeVerb-flavored reverb to Warps

## What we already have working in our favor

Warps Symbiote already ships a fully-built reverb infrastructure that's structurally very close to what an ErbeVerb-style algorithm needs — we wouldn't be building from zero:

- **A generic delay-network DSL**: [../dsp/fx/fx_engine.h](../dsp/fx/fx_engine.h) (lines 104–300) implements `FxEngine<size, FORMAT_16_BIT>` — a compile-time-reserved delay buffer, a `Context` with `Read` / `Write` / `WriteAllPass` / `Interpolate` / `Lp` / `Hp` primitives, and two free LFOs. This is the same DSL Mutable used in Clouds, Rings, Elements and Plaits. An FDN reverb maps onto it almost directly.
- **A 32k × 16-bit shared buffer** allocated for FX use ([../dsp/fx/reverb.h](../dsp/fx/reverb.h) line 255 — `FxEngine<32768, FORMAT_16_BIT>`), passed in by the Modulator at `Init` ([../dsp/modulator.cc](../dsp/modulator.cc) line 62). That's 64 KB of contiguous q15 delay memory, reused by `reverb` and `ensemble`. The current Griesinger reverb only consumes ~21,657 samples of it — there's room.
- **A pluggable mode dispatch**: [../dsp/modulator.cc](../dsp/modulator.cc) lines 892–944 switches on `feature_mode_`, with `FEATURE_MODE_REVERB` already routed through `ProcessReverb`. Adding a new mode is two lines in [../dsp/parameters.h](../dsp/parameters.h) (lines 44–54) plus a `case` in the switch.
- **The osc-shape selector inside reverb mode already multiplexes algorithms**: the current `ProcessReverb` uses `parameters_.carrier_shape` to pick between Caveman / Rings / Clouds / Elements voicings. An "Erbe" voicing could slot in there, or we could split it into its own top-level mode.

## What's publicly known about ErbeVerb

The actual Make Noise / SoundHack algorithm is proprietary — but Tom Erbe has published enough that the *flavor* is reproducible from first principles:

- It's a **feedback delay network (FDN)**, almost certainly 8-line with a Hadamard or Householder mixing matrix, with Schroeder allpass diffusers nested inside each delay line. This is the topology Erbe describes in his "Reverb Topologies and Design" notes and the one used in SoundHack +tverb.
- The signature characteristic is **continuous, glitch-free SIZE morphing** — sweeping SIZE produces an audible doppler/pitch-bend on the tail because the delay reads are fractional and scale together. This is a deliberate sonic feature.
- Controls on the hardware: **SIZE, DECAY, ABSORB (HF damp in the FB path), TILT (output coloration), PREDELAY, MIX**, plus an INFINITY freeze.
- The wet output goes through a tilt EQ (low-shelf + high-shelf with opposing gains) rather than a fixed lowpass.

What is *not* public: exact delay lengths, the matrix variant, the diffuser coefficients, the exact pre/post EQ curves, and the freeze implementation. Anyone porting is rebuilding the topology in the same family, not duplicating bits.

**Closest open-source reference implementations** (much better starting points than guessing):

- **`zita-rev1`** (Fons Adriaensen, GPL): documented 8-line FDN with Schroeder diffusion. Almost directly maps onto our `FxEngine`.
- **Faust `freeverb` / `jpverb` / `zita_rev1`** in `faust-stk`: easy to prototype in, we can iterate on coefficients in seconds.
- Jot's 1991 paper on FDN reverb; Sean Costello's CCRMA writeups.

## How it maps onto Warps specifically

The non-obvious design decisions:

1. **SIZE morphing fits the FxEngine cleanly.** Reserve each FDN line at its *maximum* length at compile time, then read via `c.Interpolate(line, size_param * max_len, scale)` — the engine already does fractional reads (see how `del2` and `del1` are LFO-modulated in the current reverb at [../dsp/fx/reverb.h](../dsp/fx/reverb.h) lines 127–134). The pitch-bend-on-sweep behavior comes for free.

2. **Memory budget at 16-bit q15 is comfortable.** Eight delays sized roughly `{1607, 1949, 2381, 2837, 3463, 4111, 4729, 5417}` samples + 4 input diffuser APs (~150–600 samples each) + a 2–3k pre-delay tap fits in ~27k samples, well inside the existing 32k buffer.

3. **CPU is the tight constraint, not RAM.** An 8-line FDN with one nested allpass per line + Hadamard mixing + per-line LP damping + tilt EQ runs roughly 2× the Griesinger reverb's op count. F4 @ 168 MHz with hard-float and the 16-bit compressed reads should handle it at 48 kHz stereo, but we should benchmark before committing. If it doesn't fit, drop to FDN-4 with deeper allpass nesting per line — sonically very close.

4. **Don't shoehorn it into the existing `Reverb` class.** That class is structurally a Griesinger/Dattorro topology with the `ReverbType` enum already adding a lot of `if` switching ([../dsp/fx/reverb.h](../dsp/fx/reverb.h) lines 107–114). An FDN is a different shape — separate file (`fdn_reverb.h`), separate `ProcessFdnReverb`, either a new `FEATURE_MODE_FDN_REVERB` enum value or extend `ReverbType` with an `ERBE` variant. We'd lean toward a new top-level mode for clean control mapping.

5. **Control mapping suggestion** (matches Warps' three knobs + algorithm + osc-shape conventions):
   - LEVEL1 → MIX
   - LEVEL2 → TILT (center = flat, CCW = dark, CW = bright)
   - ALGO → SIZE
   - MOD → DECAY (RT60)
   - OSC SHAPE button → cycles ABSORB / PREDELAY / FREEZE secondary roles, or selects flavor variants
   - The existing OSC SHAPE selector inside reverb mode already encodes flavor — same trick.

6. **The 16-bit compressed delay storage is audible on long, quiet tails.** ErbeVerb on real hardware is 24-bit. The existing reverbs hide this with diffusion and short-ish tails; an FDN with high decay and infinite-feedback freeze will expose quantization more. Consider `FORMAT_32_BIT` for at least the longest 2 delay lines if we can afford the doubled memory — we'd lose the freeze quality otherwise.

## Validation path

Warps has a test harness ([../test/warps_test.cc](../test/warps_test.cc)) — we can drive `ProcessReverb` with impulse and noise inputs from a host build, capture the wet output to a wav, and inspect the impulse response (decay envelope, modal density, spectral flatness, freeze stability) before touching hardware. **Prototype the whole thing in Faust first** — coefficient tuning in Faust takes minutes vs. hours of firmware iteration.
