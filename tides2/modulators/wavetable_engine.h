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
#include "tides2/io_buffer.h"

namespace tides {

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
  virtual void Reset();
  void LoadUserData();
  virtual void Render(const Parameters& parameters,
    float f0,
    float* out,
    float* aux,
    int8_t channel,
    size_t size);

  float channel(int index) {
    return channels_[index];
  }
  
 private:
  float ReadWave(int x, int y, int z, int phase_i, float phase_f);
   
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

  float channels_[4];
  //size_t size_;
  
  // Maps a (bank, X, Y) coordinate to a waveform index.
  // This allows all waveforms to be reshuffled by the user to create new maps.
  const int16_t** wave_map_;
  
  Differentiator diff_out_;
  
  DISALLOW_COPY_AND_ASSIGN(WavetableEngine);
};

template<typename T>
inline float InterpolateWaveHermite(
    const T* table,
    int32_t index_integral,
    float index_fractional) {
    const float xm1 = static_cast<float>(table[index_integral]);
    const float x0 = static_cast<float>(table[index_integral + 1]);
    const float x1 = static_cast<float>(table[index_integral + 2]);
    const float x2 = static_cast<float>(table[index_integral + 3]);
    const float c = (x1 - xm1) * 0.5f;
    const float v = x0 - x1;
    const float w = c + v;
    const float a = w + v + (x2 - x0) * 0.5f;
    const float b_neg = w + a;
    const float f = index_fractional;
    return (((a * f) - b_neg) * f + c) * f + x0;
}

inline float NoteToFrequency(float midi_note) {
  midi_note -= 9.0f;
  CONSTRAIN(midi_note, -128.0f, 127.0f);
  return a0 * 0.25f * stmlib::SemitonesToRatio(midi_note);
}

}  // namespace plaits

#endif  // TIDES2_MODULATORS_ENGINE_WAVETABLE_ENGINE_H_