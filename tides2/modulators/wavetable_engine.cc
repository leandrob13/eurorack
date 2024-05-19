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

#include "tides2/modulators/wavetable_engine.h"

#include <algorithm>

#include "tides2/resources.h"
#include "stmlib/dsp/parameter_interpolator.h"

namespace tides {

using namespace std;
using namespace stmlib;

//const int kNumBanks = 4;
//const int kNumWavesPerBank = 64;
const size_t table_size = 128;
const float table_size_f = float(table_size);
//const int kNumWaves = 192;

//const size_t kTableSize = 128;
//const float kTableSizeF = float(kTableSize);

void WavetableEngine::Init() {
  phase_ = 0.0f;

  x_lp_ = 0.0f;
  y_lp_ = 0.0f;
  z_lp_ = 0.0f;
  
  x_pre_lp_ = 0.0f;
  y_pre_lp_ = 0.0f;
  z_pre_lp_ = 0.0f;

  previous_x_ = 0.0f;
  previous_y_ = 0.0f;
  previous_z_ = 0.0f;
  previous_f0_ = a0;
  fold_ = 0.0f;
  lp_ = 0.0f;

  diff_out_.Init();
  for (int i = 0; i < 4; i++) {
    filter_[i].Init();
    phases_[i] = 0.0f;
  }
}

inline float ReadWave(
    int x,
    int y,
    int z,
    int randomize,
    int phase_integral,
    float phase_fractional) {
  int wave = ((x + y * 8 + z * 64) * randomize) % 192;
  return InterpolateWaveHermite(
      wav_integrated_waves + wave * (table_size + 4),
      phase_integral,
      phase_fractional);
}

inline float Clamp(float x, float amount) {
  x = x - 0.5f;
  x *= amount;
  CONSTRAIN(x, -0.5f, 0.5f);
  x += 0.5f;
  return x;
}

void WavetableEngine::Render(
    const Parameters& parameters,
    float f0,
    PolySlopeGenerator::OutputSample* out,
    int channels,
    size_t size) {
  
  ONE_POLE(x_pre_lp_, parameters.shape * 6.9999f, 0.2f);
  ONE_POLE(y_pre_lp_, parameters.slope * 6.9999f, 0.2f);
  ONE_POLE(z_pre_lp_, parameters.shift * 2.9999f, 0.05f);
  
  const float x = x_pre_lp_;
  const float y = y_pre_lp_;
  const float z = z_pre_lp_;
  
  const float quantization = min(max(z - 3.0f, 0.0f), 1.0f);
  const float lp_coefficient = min(
      max(2.0f * f0 * (4.0f - 3.0f * quantization), 0.01f), 0.1f);
  
  MAKE_INTEGRAL_FRACTIONAL(x);
  MAKE_INTEGRAL_FRACTIONAL(y);
  MAKE_INTEGRAL_FRACTIONAL(z);
  
  x_fractional += quantization * (Clamp(x_fractional, 16.0f) - x_fractional);
  y_fractional += quantization * (Clamp(y_fractional, 16.0f) - y_fractional);
  z_fractional += quantization * (Clamp(z_fractional, 16.0f) - z_fractional);
  
  ParameterInterpolator x_modulation(
      &previous_x_, static_cast<float>(x_integral) + x_fractional, size);
  ParameterInterpolator y_modulation(
      &previous_y_, static_cast<float>(y_integral) + y_fractional, size);
  ParameterInterpolator z_modulation(
      &previous_z_, static_cast<float>(z_integral) + z_fractional, size);

  ParameterInterpolator f0_modulation(&previous_f0_, f0, size);

  ParameterInterpolator fold_modulation(&fold_, max(2.0f * (parameters.smoothness - 0.5f), 0.0f), size);
  
  for (size_t index = 0; index < size; index++) {
    const float f0 = f0_modulation.Next();
    
    const float gain = (1.0f / (f0 * 131072.0f)) * (0.95f - f0);
    const float cutoff = min(table_size_f * f0, 1.0f);
    
    ONE_POLE(x_lp_, x_modulation.Next(), lp_coefficient);
    ONE_POLE(y_lp_, y_modulation.Next(), lp_coefficient);
    ONE_POLE(z_lp_, z_modulation.Next(), lp_coefficient);
    
    const float x = x_lp_;
    const float y = y_lp_;
    const float z = z_lp_;

    MAKE_INTEGRAL_FRACTIONAL(x);
    MAKE_INTEGRAL_FRACTIONAL(y);
    MAKE_INTEGRAL_FRACTIONAL(z);

    //phase_ += f0;
    //if (phase_ >= 1.0f) {
    //  phase_ -= 1.0f;
    //}
    
    //const float p = phase_ * kTableSizeF;
    //MAKE_INTEGRAL_FRACTIONAL(p);

    int x0 = x_integral;
    int x1 = x_integral + 1;
    int y0 = y_integral;
    int y1 = y_integral + 1;

    for (int channel = 0; channel < channels; channel++){
      phases_[channel] += f0;
      if (phases_[channel] >= 1.0f) {
        phases_[channel] -= 1.0f;
      }

      const float p = phases_[channel] * table_size_f;
      MAKE_INTEGRAL_FRACTIONAL(p);

      int z0 = z_integral + channel;
      int z1 = z0 + 1;

      if (z0 >= 4) {
        z0 -= 4;
      }
      if (z1 >= 4) {
        z1 -= 4;
      }

      int r0 = z0 == 3 ? 101 : 1;
      int r1 = z1 == 3 ? 101 : 1;
      
      float x0y0z0 = ReadWave(x0, y0, z0, r0, p_integral, p_fractional);
      float x1y0z0 = ReadWave(x1, y0, z0, r0, p_integral, p_fractional);
      float xy0z0 = x0y0z0 + (x1y0z0 - x0y0z0) * x_fractional;

      float x0y1z0 = ReadWave(x0, y1, z0, r0, p_integral, p_fractional);
      float x1y1z0 = ReadWave(x1, y1, z0, r0, p_integral, p_fractional);
      float xy1z0 = x0y1z0 + (x1y1z0 - x0y1z0) * x_fractional;

      float xyz0 = xy0z0 + (xy1z0 - xy0z0) * y_fractional;

      float x0y0z1 = ReadWave(x0, y0, z1, r1, p_integral, p_fractional);
      float x1y0z1 = ReadWave(x1, y0, z1, r1, p_integral, p_fractional);
      float xy0z1 = x0y0z1 + (x1y0z1 - x0y0z1) * x_fractional;

      float x0y1z1 = ReadWave(x0, y1, z1, r1, p_integral, p_fractional);
      float x1y1z1 = ReadWave(x1, y1, z1, r1, p_integral, p_fractional);
      float xy1z1 = x0y1z1 + (x1y1z1 - x0y1z1) * x_fractional;
      
      float xyz1 = xy0z1 + (xy1z1 - xy0z1) * y_fractional;

      float mix = xyz0 + (xyz1 - xyz0) * z_fractional;
      mix = diff_outs_[channel].Process(cutoff, mix * gain);
      //ONE_POLE(lp_, mix, cutoff);
      //out[index].channel[0] = mix; 
      out[index].channel[channel] = fold(mix, fold_modulation.Next());
    }
  }
  filter(f0, parameters.smoothness, out, channels, size);
}

}  // namespace tides
