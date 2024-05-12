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

class Delay {
 public:
  Delay() { }
  ~Delay() { }
  
  void Init(uint16_t* buffer) {
    engine_.Init(buffer);
    delay_time_ = 0.0f;
    feedback_ = 0.0f;
  }
  
  void Process(float* left, float* right, size_t size) {
    typedef E::Reserve<32767> Memory;
    //typedef E::Reserve<16383, E::Reserve<16383> > Memory;
    E::DelayLine<Memory, 0> line_l;
    //E::DelayLine<Memory, 1> line_r;
    E::Context c;

    float del_out, mix_l;

    while (size--) {
      engine_.Start(&c);
     
      c.Load(0.0f);
      c.ReadHermite(line_l, 32768.0f * delay_time_, 1.0f);
      c.Write(del_out);
      c.Read(*left, 0.5f);
      c.Read(*right, 0.5f);
      c.Write(mix_l);

      c.Load((del_out * feedback_ * 0.85f));
      c.Read(*left, 0.5f);
      c.Read(*right, 0.5f);
      c.Write(line_l, 1.0f);
      
      *left = mix_l;
      *right = mix_l;

      /*c.Load(0.0f);
      c.Interpolate(line_r, 16384.0f * delay_time_, 1.0f);
      c.Write(del_out);
      c.Read(*right);
      c.Write(mix_r);
      c.Load((del_out * feedback_ * 0.85f) + *right);
      c.Write(line_r, 1.0f);*/

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
  }
  
 private:
  typedef FxEngine<32768, FORMAT_16_BIT> E;
  E engine_;
  
  float delay_time_;
  float feedback_;
  
  DISALLOW_COPY_AND_ASSIGN(Delay);
};

}  // namespace rings

#endif  // RINGS_DSP_FX_ENSEMBLE_H_