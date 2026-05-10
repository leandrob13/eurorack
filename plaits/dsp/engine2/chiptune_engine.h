// Copyright 2021 Emilie Gillet.
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
// Chiptune waveforms with arpeggiator.

#ifndef PLAITS_DSP_ENGINE_CHIPTUNE_ENGINE_H_
#define PLAITS_DSP_ENGINE_CHIPTUNE_ENGINE_H_

#include "plaits/dsp/chords/chord_bank.h"
#include "plaits/dsp/engine/engine.h"
#include "plaits/dsp/engine2/arpeggiator.h"
#include "plaits/dsp/oscillator/nes_triangle_oscillator.h"
#include "plaits/dsp/oscillator/super_square_oscillator.h"

namespace plaits {

const float periodTable[16] = {
    30.0f, 28.0f, 26.0f, 24.0f, 22.0f, 20.0f, 18.0f, 16.0f, 14.0f, 12.0f, 10.0f, 8.0f, 6.0f, 4.0f, 2.0f, 0.0f
    //4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
};

class NESNoiseChannel {
public:
    NESNoiseChannel() {};

    void Init() {
        lfsr = 1; // Reset LFSR
        timer = 0; // Reset timer
        sample = 0; //0x0cff;
        //shortMode = false; // Reset mode
        //volume = 8; // Reset volume
        //outputBit = false; // Reset output bit
    }

    void RenderNoise(float period, float* out, size_t size) {
      //uint16_t timerPeriod = static_cast<uint16_t>(period * 32.0f) + 2;
      float timerPeriod = stmlib::Interpolate(periodTable, period, 14.0f);
      //uint16_t timerPeriod = periodTable[index_integral % 16];
      uint8_t current_lfsr = lfsr;
      uint8_t current_sample = sample;
      while (size--) {
        
        if (timer == 0) {
          //bool outputBit = false;
          timer = static_cast<uint8_t>(timerPeriod); // Reset timer
          uint8_t tap = current_lfsr >> 1;
          uint8_t random_bit = (current_lfsr ^ tap) & 1;
          current_lfsr >>= 1;
          if (random_bit) {
            current_lfsr |= 0x4000;
            current_sample = 15;//0x0300;
          } else {
            current_sample = 0; //0x0cff;
          }
          // Compute feedback based on mode
          //bool feedback = (lfsr & 1) ^ ((lfsr >> 1) & 1);
          //bool feedback = (lfsr & 1) ^ ((shortMode ? (lfsr >> 6) : (lfsr >> 1)) & 1);
          //lfsr = (lfsr >> 1) | ((lfsr & 1) ^ ((lfsr >> 1) & 1) << 14); // Shift and insert feedback
          //lfsr = (lfsr >> 1) | (feedback << 14); // Shift and insert feedback
          //outputBit = !(lfsr & 1); // Output is the complement of bit 0
        } /*else {
            timer--;
        }*/
        *out++ = static_cast<float>(current_sample); // Output volume or silence
      }
      sample = current_sample;
      lfsr = current_lfsr;
      timer -= 1;
    }

private:
    uint8_t sample;
    uint8_t lfsr;           // 15-bit LFSR, initialized to non-zero
    uint8_t timer;          // Timer counter
};

class ChiptuneEngine : public Engine {
 public:
  ChiptuneEngine() { }
  ~ChiptuneEngine() { }
  
  enum {
    NO_ENVELOPE = 2
  };
  
  virtual void Init(stmlib::BufferAllocator* allocator);
  virtual void Reset();
  virtual void LoadUserData(const uint8_t* user_data) { }
  virtual void Render(const EngineParameters& parameters,
      float* out,
      float* aux,
      size_t size,
      bool* already_enveloped);
  
  inline void set_envelope_shape(float envelope_shape) {
    envelope_shape_ = envelope_shape;
  }
  
 private:
  SuperSquareOscillator voice_[kChordNumVoices];
  NESTriangleOscillator<> bass_;
  //NESNoiseChannel noise_;
  
  ChordBank chords_;
  float chord_;
  Arpeggiator arpeggiator_;
  stmlib::HysteresisQuantizer2 arpeggiator_pattern_selector_;
  
  float envelope_shape_;
  float envelope_state_;
  float aux_envelope_amount_;
  
  DISALLOW_COPY_AND_ASSIGN(ChiptuneEngine);
};

}  // namespace plaits

#endif  // PLAITS_DSP_ENGINE_CHIPTUNE_ENGINE_H_