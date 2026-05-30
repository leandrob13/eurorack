// Copyright 2026 Symbiote.
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
// Stereo tremolo: one LFO with selectable shape (sine / square / saw / S&H)
// and continuous stereo phase offset. Self-contained, mirrors the phaser.

#ifndef WARPS_DSP_FX_TREMOLO_H_
#define WARPS_DSP_FX_TREMOLO_H_

#include <math.h>

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/units.h"

#include "warps/resources.h"

namespace warps {

class Tremolo {
 public:
  Tremolo() { }
  ~Tremolo() { }

  void Init(float sample_rate) {
    sample_rate_ = sample_rate;
    rate_ = 0.3f;
    depth_ = 0.5f;
    amount_ = 1.0f;
    stereo_phase_ = 0.0f;
    shape_ = 0;
    // ~150 Hz one-pole on the output gain to absorb square/S&H steps.
    gain_smooth_a_ = 1.0f - expf(-2.0f * 3.14159265f * 200.0f / sample_rate_);
    rng_state_ = 0x1234abcdu;
    Reset();
  }

  void Reset() {
    lfo_phase_ = 0;
    prev_phase_l_ = 0;
    prev_phase_r_ = 0;
    sh_l_ = 0.0f;
    sh_r_ = 0.0f;
    gain_smooth_l_ = 1.0f;
    gain_smooth_r_ = 1.0f;
  }

  inline void set_rate(float rate) { rate_ = rate; }
  inline void set_depth(float depth) { depth_ = depth; }
  inline void set_amount(float amount) { amount_ = amount; }
  inline void set_stereo_phase(float p) {
    if (p < 0.0f) p = 0.0f;
    if (p > 0.5f) p = 0.5f;
    stereo_phase_ = p;
  }
  inline void set_shape(int32_t s) {
    if (s < 0) s = 0;
    if (s > 3) s = 3;
    shape_ = s;
  }

  void Process(float* left, float* right, size_t size) {
    // 0.1 Hz .. 30 Hz, exp-mapped. log2(300) * 12 ~= 99.5.
    float lfo_freq = 0.1f * stmlib::SemitonesToRatio(rate_ * 99.5f);
    uint32_t phase_inc = static_cast<uint32_t>(
        lfo_freq / sample_rate_ * 4294967296.0f);

    uint32_t r_offset = static_cast<uint32_t>(stereo_phase_ * 4294967296.0f);

    float depth = depth_;
    float amount = amount_;
    int32_t shape = shape_;
    float a = gain_smooth_a_;

    uint32_t phase = lfo_phase_;
    uint32_t prev_l = prev_phase_l_;
    uint32_t prev_r = prev_phase_r_;
    float sh_l = sh_l_;
    float sh_r = sh_r_;
    float gs_l = gain_smooth_l_;
    float gs_r = gain_smooth_r_;

    for (size_t i = 0; i < size; ++i) {
      uint32_t phase_l = phase;
      uint32_t phase_r = phase + r_offset;

      // Wrap detection per channel for S&H re-trigger.
      bool wrap_l = phase_l < prev_l;
      bool wrap_r = phase_r < prev_r;
      prev_l = phase_l;
      prev_r = phase_r;

      float lfo_l, lfo_r;
      switch (shape) {
        case 1: {  // Square (50% duty)
          lfo_l = (phase_l < 0x80000000u) ? 1.0f : -1.0f;
          lfo_r = (phase_r < 0x80000000u) ? 1.0f : -1.0f;
          break;
        }
        case 2: {  // Sawtooth: +1 -> -1 over the cycle.
          lfo_l = 1.0f - static_cast<float>(phase_l) * (2.0f / 4294967296.0f);
          lfo_r = 1.0f - static_cast<float>(phase_r) * (2.0f / 4294967296.0f);
          break;
        }
        case 3: {  // Sample & Hold
          if (wrap_l) sh_l = NextRandomBipolar();
          if (wrap_r) sh_r = NextRandomBipolar();
          lfo_l = sh_l;
          lfo_r = sh_r;
          break;
        }
        case 0:
        default: {  // Sine
          lfo_l = lut_sin[phase_l >> (32 - 10)];
          lfo_r = lut_sin[phase_r >> (32 - 10)];
          break;
        }
      }

      // gain = 1 - depth * (0.5 - 0.5 * lfo) ; lfo in [-1,+1]
      float g_target_l = 1.0f - depth * (0.5f - 0.5f * lfo_l);
      float g_target_r = 1.0f - depth * (0.5f - 0.5f * lfo_r);

      gs_l += a * (g_target_l - gs_l);
      gs_r += a * (g_target_r - gs_r);

      float dry_l = *left;
      float dry_r = *right;
      float wet_l = dry_l * gs_l;
      float wet_r = dry_r * gs_r;

      *left++ = dry_l + (wet_l - dry_l) * amount;
      *right++ = dry_r + (wet_r - dry_r) * amount;

      phase += phase_inc;
    }

    lfo_phase_ = phase;
    prev_phase_l_ = prev_l;
    prev_phase_r_ = prev_r;
    sh_l_ = sh_l;
    sh_r_ = sh_r;
    gain_smooth_l_ = gs_l;
    gain_smooth_r_ = gs_r;
  }

 private:
  inline float NextRandomBipolar() {
    // xorshift32
    uint32_t x = rng_state_;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state_ = x;
    return static_cast<float>(x) * (2.0f / 4294967296.0f) - 1.0f;
  }

  float sample_rate_;
  float rate_;
  float depth_;
  float amount_;
  float stereo_phase_;
  int32_t shape_;

  float gain_smooth_a_;
  uint32_t lfo_phase_;
  uint32_t prev_phase_l_;
  uint32_t prev_phase_r_;
  float sh_l_, sh_r_;
  float gain_smooth_l_, gain_smooth_r_;
  uint32_t rng_state_;

  DISALLOW_COPY_AND_ASSIGN(Tremolo);
};

}  // namespace warps

#endif  // WARPS_DSP_FX_TREMOLO_H_
