// Copyright 2016 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
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
// 8x8x3 wave terrain.

#ifndef TIDES2_MODULATORS_ENGINE_WAVETABLE_ENGINE_H_
#define TIDES2_MODULATORS_ENGINE_WAVETABLE_ENGINE_H_

//#include "plaits/dsp/engine/engine.h"
//#include "plaits/dsp/oscillator/wavetable_oscillator.h"
//#include "stmlib/stmlib.h"
#include "stmlib/utils/buffer_allocator.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/units.h"
#include "stmlib/dsp/filter.h"
#include "tides2/io_buffer.h"
#include "tides2/poly_slope_generator.h"

namespace tides {
using namespace stmlib;

const float a0 = (440.0f / 8.0f) / kSampleRate;

class Differentiator {
 public:
  Differentiator() { }
  ~Differentiator() { }

  void Init() {
    previous_ = 0.0f;
    lp_ = 0.0f;
  }
  
  float Process(float coefficient, float s) {
    ONE_POLE(lp_, s - previous_, coefficient);
    previous_ = s;
    return lp_;
  }
 private:
  float lp_;
  float previous_;

  DISALLOW_COPY_AND_ASSIGN(Differentiator);
};

class WavetableEngine {
 public:
  WavetableEngine() { }
  ~WavetableEngine() { }
  
  virtual void Init();
  virtual void Render(const Parameters& parameters,
    float f0,
    PolySlopeGenerator::OutputSample* out,
    int channels,
    size_t size);

  inline float fold(float bipolar, float fold_amount) {
    //float bipolar = 2.0f * unipolar - 1.0f;
    float folded = fold_amount > 0.0f ? stmlib::Interpolate(
        lut_bipolar_fold,
        0.5f + bipolar * (0.03f + 0.46f * fold_amount),
        1024.0f) : 0.0f;
    return 5.0f * (bipolar + (folded - bipolar) * fold_amount);
  }

  inline void filter(float frequency, float smoothness, PolySlopeGenerator::OutputSample* out, int channels, size_t size) {
    if (smoothness < 0.5f) {
      float filter_frequency = smoothness * 2.0f;
      float cutoff = 0.2f * frequency * SemitonesToRatio(120.0f * filter_frequency);
      
      float o1;
      for (int channel = 0; channel < channels; channel++) {
        filter_[channel].set_f_q<FREQUENCY_FAST>(cutoff, 0.3f);

        for (size_t i = 0; i < size; ++i) {
          o1 = filter_[channel].Process<FILTER_MODE_LOW_PASS>(out[i].channel[channel]);
          out[i].channel[channel] = filter_[channel].Process<FILTER_MODE_LOW_PASS>(o1);
        }
      }
    }
  }

  
 private:
  //float ReadWave(int x, int y, int z, int phase_i, float phase_f);
   
  float phase_;
  
  float x_pre_lp_;
  float y_pre_lp_;
  float z_pre_lp_;
  
  float x_lp_;
  float y_lp_;
  float z_lp_;

  float previous_x_;
  float previous_y_;
  float previous_z_;
  float previous_f0_;
  float fold_;

  Differentiator diff_outs_[4];
  Svf filter_[4];
  
  DISALLOW_COPY_AND_ASSIGN(WavetableEngine);
};

inline float InterpolateWaveHermite(
    const int16_t* table,
    int32_t index_integral,
    float index_fractional) {
  const float xm1 = table[index_integral];
  const float x0 = table[index_integral + 1];
  const float x1 = table[index_integral + 2];
  const float x2 = table[index_integral + 3];
  const float c = (x1 - xm1) * 0.5f;
  const float v = x0 - x1;
  const float w = c + v;
  const float a = w + v + (x2 - x0) * 0.5f;
  const float b_neg = w + a;
  const float f = index_fractional;
  return (((a * f) - b_neg) * f + c) * f + x0;
}

inline float InterpolateWave(
    const int16_t* table,
    int32_t index_integral,
    float index_fractional) {
  float a = static_cast<float>(table[index_integral]);
  float b = static_cast<float>(table[index_integral + 1]);
  float t = index_fractional;
  return a + (b - a) * t;
}

}  // namespace plaits

#endif  // TIDES2_MODULATORS_ENGINE_WAVETABLE_ENGINE_H_
