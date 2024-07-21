// Copyright 2014 Emilie Gillet.
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
// Ensemble FX.

#ifndef RINGS_DSP_FX_DELAY_H_
#define RINGS_DSP_FX_DELAY_H_

#include "stmlib/stmlib.h"

#include "stmlib/dsp/dsp.h"

#include "rings/dsp/fx/fx_engine.h"
#include "rings/resources.h"

namespace rings {

const size_t buffer_size = 32768 / 2;

template<typename T, size_t max_delay>
class DelayLine {
 public:
  DelayLine() { }
  ~DelayLine() { }
  
  void Init(T* buffer) {
    line_ = buffer;
    Reset();
  }

  void Reset() {
    std::fill(&line_[0], &line_[max_delay], T(0));
    delay_ = 1;
    write_ptr_ = 0;
  }
  
  inline void set_delay(size_t delay) {
    delay_ = delay;
  }

  inline void Write(const T sample) {
    line_[write_ptr_] = sample;
    write_ptr_ = (write_ptr_ - 1 + max_delay) % max_delay;
  }
  
  inline const T Allpass(const T sample, size_t delay, const T coefficient) {
    T read = line_[(write_ptr_ + delay) % max_delay];
    T write = sample + coefficient * read;
    Write(write);
    return -write * coefficient + read;
  }

  inline const T WriteRead(const T sample, float delay) {
    Write(sample);
    return Read(delay);
  }
  
  inline const T Read() const {
    return line_[(write_ptr_ + delay_) % max_delay];
  }
  
  inline const T Read(int32_t delay) const {
    return line_[(write_ptr_ + delay) % max_delay];
  }

  inline const T Read(float delay) const {
    MAKE_INTEGRAL_FRACTIONAL(delay)
    const T a = line_[(write_ptr_ + delay_integral) % max_delay];
    const T b = line_[(write_ptr_ + delay_integral + 1) % max_delay];
    return a + (b - a) * delay_fractional;
  }
  
  inline const T ReadHermite(float delay) const {
    MAKE_INTEGRAL_FRACTIONAL(delay)
    int32_t t = (write_ptr_ + delay_integral + max_delay);
    const T xm1 = line_[(t - 1) % max_delay];
    const T x0 = line_[(t) % max_delay];
    const T x1 = line_[(t + 1) % max_delay];
    const T x2 = line_[(t + 2) % max_delay];
    const float c = (x1 - xm1) * 0.5f;
    const float v = x0 - x1;
    const float w = c + v;
    const float a = w + v + (x2 - x0) * 0.5f;
    const float b_neg = w + a;
    const float f = delay_fractional;
    return (((a * f) - b_neg) * f + c) * f + x0;
  }

  inline float InterpolateCubic(float index) {
    MAKE_INTEGRAL_FRACTIONAL(index)
    // Calculate indices for the points involved in the interpolation
    int xm1 = (write_ptr_ + index_integral - 1 + max_delay) % max_delay;
    int xp1 = (write_ptr_ + index_integral + 1) % max_delay;
    int xp2 = (write_ptr_ + index_integral + 2) % max_delay;

    // Since x is used directly as an index, ensure it is within the bounds of the array
    int32_t ptr = (write_ptr_ + index_integral) % max_delay;

    // Retrieve the values from the table for the points involved in the interpolation
    const float ym1 = line_[xm1];
    const float y0 = line_[ptr];
    const float y1 = line_[xp1];
    const float y2 = line_[xp2];

    // Perform the cubic interpolation
    const float c0 = y0;
    const float c1 = 0.5f * (y1 - ym1);
    const float c2 = ym1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    const float c3 = 0.5f * (y2 - ym1) + 1.5f * (y0 - y1);

    // Calculate the interpolated value
    return ((c3 * index_fractional + c2) * index_fractional + c1) * index_fractional + c0;
}


  inline float Decompress(T value) {
    return static_cast<float>(static_cast<int16_t>(value)) / 32768.0f;
  }
  
  inline T Compress(float value) {
    return static_cast<uint16_t>(
        stmlib::Clip16(static_cast<int32_t>(value * 32768.0f)));
  }

 private:
  size_t write_ptr_;
  size_t delay_;
  T* line_; //[max_delay];
  
  DISALLOW_COPY_AND_ASSIGN(DelayLine);
};

template<size_t max_delay>
class DelayLine16Bits {
 public:
  DelayLine16Bits() { }
  ~DelayLine16Bits() { }
  
  void Init(uint16_t* buffer) {
    line_ = buffer;
    Reset();
  }

  void Reset() {
    std::fill(&line_[0], &line_[max_delay], 0);
    write_ptr_ = 0;
  }
  
  inline void Write(const float sample) {
    int32_t word = static_cast<int32_t>(sample * 32768.0f);
    CONSTRAIN(word, -32768, 32767);
    line_[write_ptr_] = word;
    if (write_ptr_ == 0) {
      line_[max_delay] = word;
      write_ptr_ = max_delay - 1;
    } else {
      --write_ptr_;
    }
  }
  
  inline float Read(float delay) const {
    MAKE_INTEGRAL_FRACTIONAL(delay)
    size_t read_ptr = (write_ptr_ + delay_integral) % max_delay;
    float a = static_cast<float>(line_[read_ptr]) / 32768.0f;
    float b = static_cast<float>(line_[read_ptr + 1]) / 32768.0f;
    return a + (b - a) * delay_fractional;
  }

 private:
  size_t write_ptr_;
  uint16_t* line_; //[max_delay + 1];
  
  DISALLOW_COPY_AND_ASSIGN(DelayLine16Bits);
};

class Delay {
 public:
  Delay() { }
  ~Delay() { }
  
  void Init(uint16_t* buffer) {
    engine_.Init(buffer);
    delay_line_.Init(buffer);
    delay_time_ = 0.0f;
    feedback_ = 0.0f;
    engine_.SetLFOFrequency(LFO_1, 0.3f / 48000.0f);
    engine_.SetLFOFrequency(LFO_2, 0.05f / 48000.0f);
  }
  
  void Process(float* left, float* right, size_t size) {
    const size_t reserved = buffer_size - 1;
    typedef E::Reserve<reserved, E::Reserve<reserved> > Memory;
    E::DelayLine<Memory, 0> line_l;
    E::DelayLine<Memory, 1> line_r;
    E::Context c;

    while (size--) {
      engine_.Start(&c);
      float del_out = 0.0f;
      float mix_l = 0.0f;
      float mix_r = 0.0f;
     
      c.Load(0.0f);
      c.Interpolate(line_l, static_cast<float>(reserved) * delay_time_, LFO_1, 0.5f, 0.5f);
      //c.ReadHermite(line_l, static_cast<float>(reserved) * delay_time_, 0.5f);
      c.Write(del_out);
      c.Read(*left, 0.5f);
      c.Write(mix_l, 0.0f);

      c.Load(del_out * feedback_);
      c.Read(*left, 0.5f);
      c.Write(line_l, 1.0f);
      
      *left = mix_l;

      c.Load(0.0f);
      c.Interpolate(line_r, static_cast<float>(reserved) * delay_time_, LFO_1, 0.5f, 0.5f);
      //c.ReadHermite(line_r, static_cast<float>(reserved) * delay_time_, 0.5f);
      c.Write(del_out);
      c.Read(*right, 0.5f);
      c.Write(mix_r, 0.0f);

      c.Load(del_out * feedback_);
      c.Read(*right, 0.5f);
      c.Write(line_r, 1.0f);
      
      *right = mix_r;

      left++;
      right++;
    }
  }

  void Process2(float* left, float* right, size_t size) {

    float del_out, mix_l;

    while (size--) {
      //del_out = delay_line_.Decompress(delay_line_.Read((32767.0f / 2.0f) * delay_time_)) * 0.5f;
      del_out = delay_line_.Decompress(delay_line_.Read(static_cast<int32_t>(float(buffer_size - 1) * delay_time_))) * 0.5f;
      mix_l = del_out + ((*left + *right) * 0.25f);

      //c.Load(del_out * feedback_);
      float feedback = del_out * feedback_;
      delay_line_.Write(delay_line_.Compress(feedback + ((*left + *right) * 0.25f)));
      
      *left = mix_l;
      *right = mix_l;

      left++;
      right++;
    }
  }

  void Process3(float* left, float* right, size_t size) {
    const size_t reserved = 24576; //(buffer_size * 2) - 1;
    typedef E::Reserve<reserved> Memory;
    E::DelayLine<Memory, 0> line_l;
    E::Context c;

    while (size--) {
      engine_.Start(&c);
      float del_out = 0.0f;
      float mix_l = 0.0f;
     
      c.Load(0.0f);
      c.Interpolate(line_l, static_cast<float>(reserved) * delay_time_, LFO_2, 0.5f, 0.5f);
      //c.ReadHermite(line_l, static_cast<float>(reserved) * delay_time_, 0.5f);
      c.Write(del_out);
      c.Read(*left, 0.25f);
      c.Read(*right, 0.25f);
      c.Write(mix_l, 0.0f);

      c.Load(del_out * feedback_);
      c.Read(*left, 0.25f);
      c.Read(*right, 0.25f);
      c.Write(line_l, 1.0f);
      
      *left = mix_l;
      *right = mix_l;

      left++;
      right++;
    }
  }
  
  inline void set_delay_time(float delay) {
    delay_time_ = delay;
  }
  
  inline void set_feedback(float feedback) {
    feedback_ = feedback;
  }

  inline void Clear() {
    engine_.Clear();
    delay_line_.Reset();
  }
  
 private:
  typedef FxEngine<32768, FORMAT_16_BIT> E;
  E engine_;
  
  float delay_time_;
  float feedback_;
  DelayLine<uint16_t, buffer_size> delay_line_;
  
  DISALLOW_COPY_AND_ASSIGN(Delay);
};

}  // namespace rings

#endif  // RINGS_DSP_FX_ENSEMBLE_H_