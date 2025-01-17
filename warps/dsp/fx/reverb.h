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
// Reverb.

#ifndef RINGS_DSP_FX_REVERB_H_
#define RINGS_DSP_FX_REVERB_H_

#include "stmlib/stmlib.h"

#include "warps/dsp/fx/fx_engine.h"

namespace warps {

enum ReverbType {
  CAVEMAN,
  RINGS,
  CLOUDS,
  ELEMENTS
};

class Reverb {
 public:
  Reverb() { }
  ~Reverb() { }
  
  void Init(uint16_t* buffer) {
    engine_.Init(buffer);
    engine_.SetLFOFrequency(LFO_1, 0.5f / 48000.0f);
    engine_.SetLFOFrequency(LFO_2, 0.3f / 48000.0f);
    lp_ = 0.7f;
    diffusion_ = 0.625f;
    input_gain_ = 0.2f;
    frozen_ = false;
    feedback_ = 0.0f;
    type_ = CAVEMAN;
  }
  
  void Process(float* left, float* right, size_t size) {
    // This is the Griesinger topology described in the Dattorro paper
    // (4 AP diffusers on the input, then a loop of 2x 2AP+1Delay).
    // Modulation is applied in the loop of the first diffuser AP for additional
    // smearing; and to the two long delays for a slow shimmer/chorus effect.
    typedef E::Reserve<150,
      E::Reserve<214,
      E::Reserve<319,
      E::Reserve<527,
      E::Reserve<2182,
      E::Reserve<2690,
      E::Reserve<4501,
      E::Reserve<2525,
      E::Reserve<2197,
      E::Reserve<6312> > > > > > > > > > Memory;
    E::DelayLine<Memory, 0> ap1;
    E::DelayLine<Memory, 1> ap2;
    E::DelayLine<Memory, 2> ap3;
    E::DelayLine<Memory, 3> ap4;
    E::DelayLine<Memory, 4> dap1a;
    E::DelayLine<Memory, 5> dap1b;
    E::DelayLine<Memory, 6> del1;
    E::DelayLine<Memory, 7> dap2a;
    E::DelayLine<Memory, 8> dap2b;
    E::DelayLine<Memory, 9> del2;
    E::Context c;

    const float kap = diffusion_;
    const float klp = lp_;
    const float krt = reverb_time_;
    const float amount = amount_;
    const float gain = input_gain_;

    float lp_1 = lp_decay_1_;
    float lp_2 = lp_decay_2_;
    bool is_clouds = type_ == CLOUDS;
    bool is_elements = type_ == ELEMENTS;

    while (size--) {
      float wet;
      float apout = 0.0f;
      engine_.Start(&c);
      
      if (is_clouds || is_elements) {
        // Smear AP1 inside the loop.
        c.Interpolate(ap1, 10.0f, LFO_1, is_clouds ? 60.0f : 80.0f, 1.0f);
        c.Write(ap1, 100, 0.0f);
      }
      
      c.Read(*left, gain);

      // Diffuse through 4 allpasses.
      c.Read(ap1 TAIL, kap);
      c.WriteAllPass(ap1, -kap);
      c.Read(ap2 TAIL, kap);
      c.WriteAllPass(ap2, -kap);
      c.Read(ap3 TAIL, kap);
      c.WriteAllPass(ap3, -kap);
      c.Read(ap4 TAIL, kap);
      c.WriteAllPass(ap4, -kap);
      c.Write(apout);
      
      // Main reverb loop.
      c.Load(apout);
      if (is_clouds || is_elements) {
        c.Interpolate(del2, is_clouds ? 4680.0f : 6211.0f, LFO_2, 100.0f, krt);
      } else {
        c.Interpolate(del2, 6261.0f, LFO_2, 50.0f, krt);
      }
      c.Lp(lp_1, klp);
      c.Read(dap1a TAIL, -kap);
      c.WriteAllPass(dap1a, kap);
      c.Read(dap1b TAIL, kap);
      c.WriteAllPass(dap1b, -kap);
      c.Write(del1, 2.0f);
      c.Write(wet, 0.0f);

      *left += (wet - *left) * amount;

      c.Read(*right, gain);

      // Diffuse through 4 allpasses.
      c.Read(ap1 TAIL, kap);
      c.WriteAllPass(ap1, -kap);
      c.Read(ap2 TAIL, kap);
      c.WriteAllPass(ap2, -kap);
      c.Read(ap3 TAIL, kap);
      c.WriteAllPass(ap3, -kap);
      c.Read(ap4 TAIL, kap);
      c.WriteAllPass(ap4, -kap);
      c.Write(apout);

      c.Load(apout);
      if (is_clouds || is_elements) {
        c.Read(del1 TAIL, krt);
      } else {
        c.Interpolate(del1, 4460.0f, LFO_1, 40.0f, krt);
      }
      c.Lp(lp_2, klp);
      c.Read(dap2a TAIL, kap);
      c.WriteAllPass(dap2a, -kap);
      c.Read(dap2b TAIL, -kap);
      c.WriteAllPass(dap2b, kap);
      c.Write(del2, 2.0f);
      c.Write(wet, 0.0f);

      *right += (wet - *right) * amount;
      
      ++left;
      ++right;
    }
    
    lp_decay_1_ = lp_1;
    lp_decay_2_ = lp_2;
  }
  
  inline void set_amount(float amount) {
    switch (type_)
    {
      case CAVEMAN:
        amount_ = 0.75f * amount;
        break;
      case RINGS:
        amount_ = 0.1f + amount * 0.5f;
        break;
      case CLOUDS:
        {
          float reverb_amount = amount * 0.95f;
          reverb_amount += feedback_ * (2.0f - feedback_);
          CONSTRAIN(reverb_amount, 0.0f, 1.0f);
          amount_ = 0.54f * reverb_amount;
        }
        break;
      case ELEMENTS:
        set_input_gain(amount <= 0.2f ? 0.2f : amount);
        break;
      default:
        break;
    }
  }
  
  inline void set_input_gain(float input_gain) {
    input_gain_ = (frozen_ && type_ == ELEMENTS) ? 0.0f : input_gain;
  }

  inline void set_time(float reverb_time) {
    frozen_ = false;
    switch (type_)
    {
      case CAVEMAN:
        reverb_time_ = 0.5f + 0.49f * reverb_time;
        break;
      case RINGS:
        reverb_time_ = 0.35f + 0.63f * reverb_time;
        break;
      case CLOUDS:
        reverb_time_ = 0.35f + 0.63f * reverb_time;
        break;
      case ELEMENTS:  
        {
          frozen_ = reverb_time >= 0.95f;
          amount_ = reverb_time >= 0.4f ? 0.4f : reverb_time;
          reverb_time_ = frozen_ ? 1.0f : 0.35f + 1.2f * amount_;
        }
        break;
      default:
        break;
    }
  }
  
  inline void set_diffusion(float diffusion) {
    diffusion_ = frozen_ ? 0.625f : diffusion;
  }
  
  inline void set_lp(float lp) {
    switch (type_) {
      case RINGS:
        lp_ = 0.3f + lp * 0.6f;
        break;
      case ELEMENTS:
        lp_ = frozen_ ? 1.0f : lp;
        break;
      case CLOUDS:
        {
          feedback_ = lp;
          lp_ = 0.6f + 0.37f * feedback_;
        }
        break;
      default:
        lp_ = lp;
        break;
    }
  }

  inline void set_type(ReverbType type) {
    type_ = type;
  }
  
  inline void Clear() {
    engine_.Clear();
  }
  
 private:
  typedef FxEngine<32768, FORMAT_16_BIT> E;
  E engine_;
  
  float amount_;
  float input_gain_;
  float reverb_time_;
  float diffusion_;
  float lp_;
  float feedback_;
  bool frozen_;
  ReverbType type_;
  
  float lp_decay_1_;
  float lp_decay_2_;
  
  DISALLOW_COPY_AND_ASSIGN(Reverb);
};

}  // namespace rings

#endif  // RINGS_DSP_FX_REVERB_H_
