// Copyright 2024 Symbiote.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Compact time-domain pitch shifter (2-tap crossfading delay-line / SOLA).
//
// Reuses the shared 32k uint16_t FX buffer that backs Reverb/Ensemble. Only
// one feature mode is active at a time, so the runtime collision is moot;
// the shifter uses the lower half (16384 cells = 32 KB) directly via a
// hand-rolled circular buffer to keep its per-instance .bss footprint
// minimal (no FxEngine LFOs / oscillators).
//
// Topology:
//   - Input is written into a circular line at unit speed.
//   - Two read taps walk through the line; the tap-to-write delay updates
//       offset += (1 - ratio)         per output sample
//     so ratio > 1 (up-shift) shrinks the delay and ratio < 1 (down-shift)
//     grows it. Each tap wraps inside [0, kWindow) to stay near the head.
//   - The two taps are offset by kWindow/2; their Hann envelopes
//       sin^2(pi * pos / kWindow)
//     are complementary, giving a constant-power crossfade that hides the
//     wrap discontinuity.
//
// See docs/pitch_shifter_plan.md.

#ifndef WARPS_DSP_FX_PITCH_SHIFTER_H_
#define WARPS_DSP_FX_PITCH_SHIFTER_H_

#include <algorithm>

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"

#include "warps/resources.h"

namespace warps {

class PitchShifter {
 public:
  PitchShifter() { }
  ~PitchShifter() { }

  enum Voicing {
    VOICING_DETUNE = 0,     // L = +detune, R = -detune (coarse ignored)
    VOICING_CONTINUOUS,     // ±1 octave from coarse, fine from detune
    VOICING_QUANTIZED,      // coarse snaps to {-12,-7,-5,0,+5,+7,+12} st
    VOICING_SHIMMER         // +12 st, regenerative feedback w/ LP in the loop
  };

  void Init(uint16_t* buffer) {
    buffer_ = buffer;
    voicing_ = VOICING_CONTINUOUS;
    ratio_l_ = ratio_r_ = 1.0f;
    target_ratio_l_ = target_ratio_r_ = 1.0f;
    mix_ = feedback_amount_ = 0.0f;
    tone_ = 0.5f;
    Clear();
  }

  void Clear() {
    std::fill(&buffer_[0], &buffer_[kBufferSize], 0);
    write_ptr_ = 0;
    tap0_l_ = 0.0f;
    tap1_l_ = kWindow * 0.5f;
    tap0_r_ = 0.0f;
    tap1_r_ = kWindow * 0.5f;
    fb_l_ = fb_r_ = 0.0f;
  }

  inline void set_mix(float mix) { mix_ = mix; }
  inline void set_feedback(float fb) { feedback_amount_ = fb; }
  // Damping in the regenerative feedback path. Only consulted in
  // VOICING_SHIMMER; non-shimmer voicings always use a unit-coefficient
  // (pass-through) feedback.
  inline void set_tone(float tone) { tone_ = tone; }
  inline void set_voicing(int32_t v) {
    CONSTRAIN(v, 0, 3);
    voicing_ = static_cast<uint8_t>(v);
  }

  // coarse_semitones: continuous coarse pitch in [-12, +12].
  // detune_semitones: additive fine offset (callers scale to taste).
  void set_pitch(float coarse_semitones, float detune_semitones) {
    switch (voicing_) {
      case VOICING_DETUNE: {
        // Coarse folded into a small ±1-semitone spread; L/R get opposite
        // signs for a thickener / chorus character.
        float st = coarse_semitones * (1.0f / 12.0f) + detune_semitones;
        target_ratio_l_ = SemitonesToRatioClipped_(st);
        target_ratio_r_ = SemitonesToRatioClipped_(-st);
        break;
      }
      case VOICING_QUANTIZED: {
        // Chromatic snap across the full ±12 semitone range (25 steps).
        // Detune still acts as a ±50 cent vernier on top of the snapped pitch.
        int32_t st_i = static_cast<int32_t>(
            coarse_semitones + (coarse_semitones >= 0.0f ? 0.5f : -0.5f));
        CONSTRAIN(st_i, -12, 12);
        float st = static_cast<float>(st_i) + detune_semitones;
        target_ratio_l_ = target_ratio_r_ = SemitonesToRatioClipped_(st);
        break;
      }
      case VOICING_SHIMMER: {
        // Algo controls the shimmer pitch around +12 st (octave up at
        // centre). Detune is intentionally ignored: the mod knob is
        // repurposed as a tone/damping control via set_tone().
        float st = 12.0f + coarse_semitones;
        target_ratio_l_ = target_ratio_r_ = SemitonesToRatioClipped_(st);
        break;
      }
      case VOICING_CONTINUOUS:
      default: {
        float st = coarse_semitones + detune_semitones;
        target_ratio_l_ = target_ratio_r_ = SemitonesToRatioClipped_(st);
        break;
      }
    }
  }

  void Process(float* left, float* right, size_t size) {
    stmlib::ParameterInterpolator ratio_l(&ratio_l_, target_ratio_l_, size);
    stmlib::ParameterInterpolator ratio_r(&ratio_r_, target_ratio_r_, size);

    // ~0.17 cents — well below audibility but large enough to catch the
    // exact-1.0 ratio used by the chromatic-0 snap of the quantized voicing.
    const float kUnisonEps = 1.0e-4f;
    const float mix = mix_;
    const float fa = feedback_amount_;
    // Damping LP coefficient in the feedback path; 1.0 = pass-through.
    // tone_ ∈ [0,1]: 0 = heavy damping (dark), 1 = no damping (bright).
    const float k_lp = (voicing_ == VOICING_SHIMMER)
        ? (0.05f + 0.95f * tone_) : 1.0f;

    float fb_l = fb_l_, fb_r = fb_r_;
    float t0l = tap0_l_, t1l = tap1_l_;
    float t0r = tap0_r_, t1r = tap1_r_;
    int32_t wp = write_ptr_;

    while (size--) {
      const float rl = ratio_l.Next();
      const float rr = ratio_r.Next();

      const float xl = *left  + fb_l * fa;
      const float xr = *right + fb_r * fa;

      // Advance write head and store left/right in the two halves.
      wp = (wp + 1) & kBufferMask;
      buffer_[wp] = Compress_(xl);
      buffer_[wp + kHalf] = Compress_(xr);

      // Advance taps; offset grows for down-shift, shrinks for up-shift.
      t0l = WrapWindow_(t0l + 1.0f - rl);
      t1l = WrapWindow_(t1l + 1.0f - rl);
      t0r = WrapWindow_(t0r + 1.0f - rr);
      t1r = WrapWindow_(t1r + 1.0f - rr);

      // Hann amplitudes from each tap's in-window position.
      const float w0l = HannLut_(t0l);
      const float w1l = HannLut_(t1l);
      const float w0r = HannLut_(t0r);
      const float w1r = HannLut_(t1r);

      // Bypass the shifter math when the per-channel ratio is effectively
      // 1.0 — the two-tap mix would otherwise comb-filter the dry signal
      // on the chromatic-0 snap of the quantized voicing.
      const bool unison_l = (rl > 1.0f - kUnisonEps) & (rl < 1.0f + kUnisonEps);
      const bool unison_r = (rr > 1.0f - kUnisonEps) & (rr < 1.0f + kUnisonEps);
      const float yl = unison_l ? *left
          : (ReadInterp_(wp, t0l + kMinOffset) * w0l
           + ReadInterp_(wp, t1l + kMinOffset) * w1l);
      const float yr = unison_r ? *right
          : (ReadInterp_(wp + kHalf, t0r + kMinOffset) * w0r
           + ReadInterp_(wp + kHalf, t1r + kMinOffset) * w1r);

      // 1-pole LP in the feedback loop (active in shimmer; k_lp=1 elsewhere
      // collapses this to fb = y, preserving non-shimmer behaviour).
      fb_l += k_lp * (yl - fb_l);
      fb_r += k_lp * (yr - fb_r);

      *left  += (yl - *left)  * mix;
      *right += (yr - *right) * mix;
      ++left;
      ++right;
    }

    write_ptr_ = wp;
    tap0_l_ = t0l; tap1_l_ = t1l;
    tap0_r_ = t0r; tap1_r_ = t1r;
    fb_l_ = fb_l; fb_r_ = fb_r;
  }

 private:
  // Lower half of the shared 32k FX buffer (32 KB). L in [0, kHalf),
  // R in [kHalf, 2*kHalf).
  static const int32_t kHalf = 8192;
  static const int32_t kBufferSize = kHalf * 2;
  static const int32_t kBufferMask = kHalf - 1;

  // ~43 ms window @ 48 kHz. Long enough to hide the wrap on pads, short
  // enough to keep the flutter rate inaudible on transients.
  static const int32_t kWindow = 2048;

  // Hermite reads 4 samples: one *newer* than the integer offset and two
  // *older*. With a minimum integer offset of 2, the newer tap lands at
  // offset 1 (the just-previous write), which is safe.
  static const int32_t kMinOffset = 2;


  static inline uint16_t Compress_(float v) {
    return static_cast<uint16_t>(
        stmlib::Clip16(static_cast<int32_t>(v * 32768.0f)));
  }

  static inline float Decompress_(uint16_t v) {
    return static_cast<float>(static_cast<int16_t>(v)) * (1.0f / 32768.0f);
  }

  static inline float SemitonesToRatioClipped_(float st) {
    CONSTRAIN(st, -24.0f, 24.0f);
    return stmlib::SemitonesToRatio(st);
  }

  static inline float WrapWindow_(float x) {
    const float w = static_cast<float>(kWindow);
    while (x < 0.0f) x += w;
    while (x >= w) x -= w;
    return x;
  }

  // Hann window via the 1024-entry global sine LUT.
  // Indices 0..511 of lut_sin span sin(0)..sin(pi).
  static inline float HannLut_(float pos) {
    float idx = pos * (512.0f / static_cast<float>(kWindow));
    int32_t i = static_cast<int32_t>(idx);
    CONSTRAIN(i, 0, 511);
    const float s = lut_sin[i];
    return s * s;
  }

  // 4-point Hermite (Catmull-Rom) interpolated read at `offset` samples
  // behind `write_ptr`, confined to the half-buffer addressed by
  // `base` ∈ {0, kHalf}. Hermite preserves HF content much better than
  // linear interpolation, especially on down-shifts where linear aliases.
  inline float ReadInterp_(int32_t base_plus_wp, float offset) const {
    MAKE_INTEGRAL_FRACTIONAL(offset);
    const int32_t bank = base_plus_wp & ~kBufferMask;
    // Four taps walking older as the offset increases: ym = one *newer*
    // than the integer offset, y0 = the integer offset itself, y1/y2 = one
    // and two samples *older*. With kMinOffset=2, ym always lands at
    // offset ≥ 1 — safely behind the freshly-written sample.
    const int32_t im = (base_plus_wp - offset_integral + 1) & kBufferMask;
    const int32_t i0 = (base_plus_wp - offset_integral)     & kBufferMask;
    const int32_t i1 = (base_plus_wp - offset_integral - 1) & kBufferMask;
    const int32_t i2 = (base_plus_wp - offset_integral - 2) & kBufferMask;
    const float ym = Decompress_(buffer_[bank | im]);
    const float y0 = Decompress_(buffer_[bank | i0]);
    const float y1 = Decompress_(buffer_[bank | i1]);
    const float y2 = Decompress_(buffer_[bank | i2]);
    const float t = offset_fractional;
    const float c0 = y0;
    const float c1 = 0.5f * (y1 - ym);
    const float c2 = ym - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    const float c3 = 0.5f * (y2 - ym) + 1.5f * (y0 - y1);
    return ((c3 * t + c2) * t + c1) * t + c0;
  }

  uint16_t* buffer_;
  int32_t write_ptr_;
  float tap0_l_, tap1_l_;
  float tap0_r_, tap1_r_;
  float fb_l_, fb_r_;
  float ratio_l_, ratio_r_;
  float target_ratio_l_, target_ratio_r_;
  float mix_, feedback_amount_;
  float tone_;
  uint8_t voicing_;

  DISALLOW_COPY_AND_ASSIGN(PitchShifter);
};

}  // namespace warps

#endif  // WARPS_DSP_FX_PITCH_SHIFTER_H_
