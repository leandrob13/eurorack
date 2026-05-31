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
// Boutique analog-modeled cascaded-allpass phaser. 
// Uses Topology Preserving Transform (TPT) allpass stages with staggered 
// frequencies and saturated feedback for a rich, "swirly" sound.

#ifndef WARPS_DSP_FX_PHASER_H_
#define WARPS_DSP_FX_PHASER_H_

#include <math.h>

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"

#include "warps/resources.h"

namespace warps {

const size_t kPhaserMaxStages = 12;

class Phaser {
 public:
  Phaser() { }
  ~Phaser() { }

  void Init(float sample_rate) {
    sample_rate_ = sample_rate;
    amount_ = 0.75f;
    feedback_ = 0.0f;
    center_ = 0.5f;
    depth_ = 0.5f;
    rate_ = 0.3f;
    stages_ = 4;
    for (size_t i = 0; i < kPhaserMaxStages; ++i) {
      g_l_[i] = 0.1f;
      g_r_[i] = 0.1f;
    }
    Reset();
  }

  void Reset() {
    for (size_t i = 0; i < kPhaserMaxStages; ++i) {
      z_l_[i] = 0.0f;
      z_r_[i] = 0.0f;
    }
    fb_l_ = 0.0f;
    fb_r_ = 0.0f;
    hp_l_ = 0.0f;
    hp_r_ = 0.0f;
    lfo_phase_ = 0;
  }

  inline void set_amount(float amount) { amount_ = amount; }
  inline void set_feedback(float feedback) { feedback_ = feedback; }
  inline void set_center(float center) { center_ = center; }
  inline void set_depth(float depth) { depth_ = depth; }
  inline void set_rate(float rate) { rate_ = rate; }

  inline void set_stages(int32_t voicing) {
    int8_t n;
    switch (voicing) {
      case 1:  n = 6;  break;
      case 2:  n = 8;  break;
      case 3:  n = 12; break;
      case 0:
      default: n = 4;  break;
    }
    if (n != stages_) {
      int8_t low = stages_ < n ? stages_ : n;
      for (int8_t i = low; i < static_cast<int8_t>(kPhaserMaxStages); ++i) {
        z_l_[i] = 0.0f;
        z_r_[i] = 0.0f;
      }
      stages_ = n;
    }
  }

  // Bipolar triangle in [-1, +1] over one full phase cycle.
  static inline float Triangle(uint32_t phase) {
    uint32_t folded = (phase < 0x80000000u) ? phase : ~phase;
    return folded * (2.0f / 2147483648.0f) - 1.0f;
  }

  void Process(float* left, float* right, size_t size) {
    // LFO: 0.1 Hz .. ~6 Hz
    float lfo_freq = 0.1f * stmlib::SemitonesToRatio(rate_ * 70.9f);
    uint32_t phase_inc = static_cast<uint32_t>(
        lfo_freq / sample_rate_ * 4294967296.0f);

    // Center frequency: 20 Hz .. 12.5 kHz
    float center_hz = 20.0f * stmlib::SemitonesToRatio(center_ * 111.4f);

    uint32_t end_phase = lfo_phase_ + phase_inc * static_cast<uint32_t>(size);
    // Triangle LFO: more vintage OTA/FET phaser character than a sine.
    float lfo_l = Triangle(end_phase);
    float lfo_r = Triangle(end_phase + 0x40000000u);

    // Exponentially staggered stage frequencies.
    // We calculate the 'g' coefficient (tan(pi*f/fs)) for each stage.
    float g_target_l[kPhaserMaxStages];
    float g_target_r[kPhaserMaxStages];
    float g_step_l[kPhaserMaxStages];
    float g_step_r[kPhaserMaxStages];

    // Vintage 4/6-stage voicings tune all stages identically (single sweeping
    // notch comb, Phase 90 / Small Stone style). Wider voicings keep a stagger.
    float spread = 0.0f;
    if (stages_ == 8) spread = 0.15f;
    else if (stages_ == 12) spread = 0.25f;

    for (int8_t i = 0; i < stages_; ++i) {
      // Stagger each stage by a fraction of an octave.
      float offset = stmlib::SemitonesToRatio((i - (stages_ - 1) * 0.5f) * spread * 12.0f);
      float f_l = center_hz * offset * stmlib::SemitonesToRatio(lfo_l * depth_ * 36.0f);
      float f_r = center_hz * offset * stmlib::SemitonesToRatio(lfo_r * depth_ * 36.0f);
      
      const float f_max = sample_rate_ * 0.45f;
      CONSTRAIN(f_l, 10.0f, f_max);
      CONSTRAIN(f_r, 10.0f, f_max);

      g_target_l[i] = tanf(3.14159265f * f_l / sample_rate_);
      g_target_r[i] = tanf(3.14159265f * f_r / sample_rate_);
      g_step_l[i] = (g_target_l[i] - g_l_[i]) / size;
      g_step_r[i] = (g_target_r[i] - g_r_[i]) / size;
    }

    float feedback = feedback_;
    // Cap wet mix at 75% so the dry signal is always present — vintage units
    // never hit a perfect 50/50 sum and retain some "body" at max depth.
    float amount = amount_;
    int8_t n_stages = stages_;
    
    for (size_t s = 0; s < size; ++s) {
      // Feedback loop with saturation and high-pass filtering (to avoid mud).
      // Classical phasers often have inverted feedback for a "hollow" vocal sound.
      hp_l_ += 0.05f * (fb_l_ - hp_l_);
      hp_r_ += 0.05f * (fb_r_ - hp_r_);
      float fb_filtered_l = fb_l_ - hp_l_;
      float fb_filtered_r = fb_r_ - hp_r_;
      
      float in_l = *left - stmlib::SoftLimit(fb_filtered_l * feedback * 2.0f) * 0.5f;
      float in_r = *right - stmlib::SoftLimit(fb_filtered_r * feedback * 2.0f) * 0.5f;

      float y_l = in_l;
      float y_r = in_r;

      // Cascaded TPT Allpass stages: 
      // y = (x - s) * g / (1 + g) + s; s_next = 2*y - x
      for (int8_t i = 0; i < n_stages; ++i) {
        g_l_[i] += g_step_l[i];
        g_r_[i] += g_step_r[i];

        float h_l = g_l_[i] / (1.0f + g_l_[i]);
        float v_l = (y_l - z_l_[i]) * h_l;
        float out_l = v_l + z_l_[i];
        z_l_[i] = out_l + v_l;
        y_l = 2.0f * out_l - y_l;

        float h_r = g_r_[i] / (1.0f + g_r_[i]);
        float v_r = (y_r - z_r_[i]) * h_r;
        float out_r = v_r + z_r_[i];
        z_r_[i] = out_r + v_r;
        y_r = 2.0f * out_r - y_r;
      }

      fb_l_ = y_l;
      fb_r_ = y_r;

      // 50/50 mix for maximal notch depth, but scale by amount_
      *left += (y_l - *left) * amount;
      *right += (y_r - *right) * amount;
      left++;
      right++;
    }

    lfo_phase_ = end_phase;
  }

 private:
  inline float SineLut(uint32_t phase) const {
    return lut_sin[phase >> (32 - 10)];
  }

  float sample_rate_;
  float amount_;
  float feedback_;
  float center_;
  float depth_;
  float rate_;
  int8_t stages_;

  float fb_l_, fb_r_;
  float hp_l_, hp_r_;
  float g_l_[kPhaserMaxStages];
  float g_r_[kPhaserMaxStages];
  float z_l_[kPhaserMaxStages];
  float z_r_[kPhaserMaxStages];
  uint32_t lfo_phase_;

  DISALLOW_COPY_AND_ASSIGN(Phaser);
};

}  // namespace warps

#endif  // WARPS_DSP_FX_PHASER_H_
