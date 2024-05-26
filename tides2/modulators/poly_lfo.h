// Copyright 2013 Emilie Gillet.
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
// Poly LFO.

#ifndef TIDES_POLY_LFO_H_
#define TIDES_POLY_LFO_H_

#include <algorithm>

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "tides2/resources.h"
#include "tides2/modulators/wavetable_engine.h"



namespace tides {
using namespace stmlib;
using namespace std;

const uint8_t kNumChannels = 4;
const uint8_t num_waves = 17;

class PolyLfo {
 public:
  PolyLfo() { }
  ~PolyLfo() { }
  
  void Init();
  void Render(int32_t frequency);

  inline void set_shape(uint16_t shape) {
    shape_ = shape;
  }
  inline void set_shape_spread(uint16_t shape_spread) {
    shape_spread_ = static_cast<int16_t>(shape_spread - 32768) >> 1;
  }
  inline void set_spread(uint16_t spread) {
    if (spread < 32768) {
      int32_t x = spread - 32768;
      int32_t scaled = -(x * x >> 15);
      spread_ = (x + 3 * scaled) >> 2;
    } else {
      spread_ = spread - 32768;
    }
  }
  inline void set_coupling(uint16_t coupling) {
    int32_t x = coupling - 32768;
    int32_t scaled = x * x >> 15;
    scaled = x > 0 ? scaled : - scaled;
    scaled = (x + 3 * scaled) >> 2;
    coupling_ = (scaled >> 4) * 10;
    
  }
  
  inline uint16_t dac_code(uint8_t index) const {
    return dac_code_[index];
  }
  static uint32_t FrequencyToPhaseIncrement(int32_t frequency);

 private:

  uint16_t shape_;
  int16_t shape_spread_;
  int32_t spread_;
  int16_t coupling_;

  int16_t value_[kNumChannels];
  uint32_t phase_[kNumChannels];
  uint16_t dac_code_[kNumChannels];

  DISALLOW_COPY_AND_ASSIGN(PolyLfo);
};

inline float InterpolateWave(
    const uint8_t* table,
    int32_t index_integral,
    float index_fractional) {
  float a = static_cast<float>(table[index_integral]);
  float b = static_cast<float>(table[index_integral + 1]);
  float t = index_fractional;
  return a + (b - a) * t;
}

const uint16_t wavetable_size = 257;

#define WAVE(wave) &wt_lfo_waveforms[wave * 257]

const uint8_t* const wavetable[] = {
  WAVE(0),
  WAVE(1),
  WAVE(2),
  WAVE(3),
  WAVE(4),
  WAVE(5),
  WAVE(6),
  WAVE(7),
  WAVE(8),
  WAVE(9),
  WAVE(10),
  WAVE(11),
  WAVE(12),
  WAVE(13),
  WAVE(14),
  WAVE(15),
  WAVE(16)
};

class PolyLfo2 {
 public:
  PolyLfo2() { }
  ~PolyLfo2() { }
  
  void Init() {
    spread_ = 0.0f;
    shape_ = 0.0f;
    shape_spread_ = 0.0f;
    coupling_ = 0.0f;
    lp_ = 0.0f;
    
    fill(&value_[0], &value_[kNumChannels], 0.0f);
    for (int i = 0; i < kNumChannels; i++) {
      phase_[i] = 0.0f;
    }
  }
  void Render(float frequency, PolySlopeGenerator::OutputSample* out) {

    const float cutoff = min(float(wavetable_size) * frequency, 1.0f);  
  // Advance phasors.
    if (spread_ >= 0.5f) {
      float phase_difference = (spread_ - 0.5f);

      for (size_t i = 0; i < kNumChannels; i++) {
        phase_[i] = i == 0 ? phase_[i] + frequency : phase_[i - 1] + phase_difference;
        if (phase_[i] >= 1.0f) {
          phase_[i] -= 1.0f;
        }
      }
      
    } else {
      float spread = 2.0f * (1.0f - spread_);
      //float x = (0.5 - spread_);
      //float scaled = (x + 2.0f * x * x) * 0.5f;
      //float spread = 2.0f * (0.5f + scaled);
      for (uint8_t i = 0; i < kNumChannels; ++i) {
        phase_[i] += frequency;
        if (phase_[i] >= 1.0f) {
          phase_[i] -= 1.0f;
        }
        frequency *= spread;
      }
    }
  
  //const uint8_t* sine = &wt_lfo_waveforms[17 * 257];
  
  /*uint16_t wavetable_index = shape_;
  // Wavetable lookup
  for (uint8_t i = 0; i < kNumChannels; ++i) {
    
    float phase = phase_[i];

    if (coupling_ > 0.0f) {
      phase += value_[(i + 1) % kNumChannels] * coupling_;
    } else {
      phase += value_[(i + kNumChannels - 1) % kNumChannels] * -coupling_;
    }
    if (phase >= 1.0f) {
      phase -= 1.0f;
    }
    const float p = phase * 257;
    MAKE_INTEGRAL_FRACTIONAL(p);
    const uint8_t* a = &wt_lfo_waveforms[(wavetable_index >> 12) * 257];
    const uint8_t* b = a + 257;
    float value_a = InterpolateWave(a, p_integral, p_fractional);
    float value_b = InterpolateWave(b, p_integral, p_fractional);

    //int16_t value = Crossfade(value_a, value_b, p_fractional);
    //value_[i] = InterpolateWave(sine, p_integral, p_fractional);
    out_[i] = Crossfade(value_a, value_b, p_fractional);//static_cast<uint16_t>(value + 32768);
    //wavetable_index += shape_spread_;
  }*/
  ///////////////////////////////////////

  float max_index = float(num_waves - 1);
  float waveform = shape_ * max_index;
    
  
  for (uint8_t i = 0; i < kNumChannels; ++i) {
    MAKE_INTEGRAL_FRACTIONAL(waveform);
    
    const float p = phase_[i] * float(wavetable_size);
    MAKE_INTEGRAL_FRACTIONAL(p);
    
    const float x0 = InterpolateWave(
        wavetable[waveform_integral], p_integral, p_fractional);
    const float x1 = InterpolateWave(
        wavetable[waveform_integral + 1], p_integral, p_fractional);
    
    const float s = (x0 + (x1 - x0) * waveform_fractional);
    ONE_POLE(lp_, s, cutoff);
    out[0].channel[i] = lp_; 

    waveform += shape_spread_ * max_index;
    if (waveform > max_index) {
      waveform -= max_index;
    } else if (waveform < 0) {
      waveform += max_index;
    }
  }
}

  inline void set_shape(float shape) {
    shape_ = shape;
  }
  inline void set_shape_spread(float shape_spread) {
    shape_spread_ = (shape_spread - 0.5f) / 2.0f;
  }
  inline void set_spread(float spread) {
    spread_ = spread;
  }
  inline void set_coupling(float coupling) {
    //int32_t x = coupling - 32768;
    //int32_t scaled = x * x >> 15;
    //scaled = x > 0 ? scaled : - scaled;
    //scaled = (x + 3 * scaled) >> 2;
    //coupling_ = (scaled >> 4) * 10;
    coupling_ = coupling - 0.5f;
    
  }

 private:

  float shape_;
  float shape_spread_;
  float spread_;
  float coupling_;

  float lp_;

  float value_[kNumChannels];
  float phase_[kNumChannels];

  DISALLOW_COPY_AND_ASSIGN(PolyLfo2);
};

}  // namespace tides

#endif  // TIDES_POLY_LFO_H_
