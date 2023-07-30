// Copyright 2015 Emilie Gillet.
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
// Part for the string synth easter egg.

#ifndef RINGS_DSP_CHORD_ORGAN_H_
#define RINGS_DSP_STRING_SYNTH_PART_H_

#include "stmlib/stmlib.h"

#include "stmlib/dsp/filter.h"

#include "rings/dsp/dsp.h"
#include "rings/dsp/fx/chorus.h"
#include "rings/dsp/fx/ensemble.h"
#include "rings/dsp/fx/reverb.h"
#include "rings/dsp/limiter.h"
#include "rings/dsp/note_filter.h"
#include "rings/dsp/patch.h"
#include "rings/dsp/performance_state.h"
#include "rings/dsp/string_synth_envelope.h"
#include "rings/dsp/string_synth_voice.h"

namespace rings {

const int32_t maxStringSynthPolyphony = 4;
const int32_t stringSynthVoices = 12;
const int32_t maxChordSize = 8;
const int32_t chord_size = 4;
const int32_t numHarmonics = 3;

enum ChordOrganFxType {
  FILTER,
  CHORUS,
  REVERB,
  FILTER_2,
  ENSEMBLE,
  REVERB_2,
  LAST
};

struct Voice {
  float tonic;
  StringSynthEnvelope envelope;
  int16_t chord;
  float chord_transpose;
  int16_t genre;
  bool active_envelope;
  float vca_level; // Damping attenuator
  float vca_cv; // Damping CV
  float filter_frequency; // Position pot
  float filter_amount; // Position attenueverter
  float filter_cv; // Position CV
};

const int32_t kRegistrationTableSize = 11;
const float registrations[kRegistrationTableSize][numHarmonics * 2] = {
  { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f },
  { 1.0f, 0.1f, 0.0f, 0.0f, 1.0f, 0.0f },
  { 1.0f, 0.5f, 1.0f, 0.0f, 1.0f, 0.0f },
  { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
  { 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f },
  { 0.0f, 0.5f, 1.0f, 0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f },
};

class ChordOrgan {
 public:
  ChordOrgan() { }
  ~ChordOrgan() { }
  
  void Init(uint16_t* reverb_buffer);
  
  void Process(
      const PerformanceState& performance_state,
      const Patch& patch,
      const float* in,
      float* out,
      float* aux,
      size_t size);

  
  inline void set_fx(ChordOrganFxType fx_type) {
    if ((fx_type % 3) != (fx_type_ % 3)) {
      clear_fx_ = true;
    }
    fx_type_ = fx_type;
  }

  inline void set_polyphony(int32_t polyphony) {
    int32_t old_polyphony = polyphony_;
    polyphony_ = std::min(polyphony, maxStringSynthPolyphony);
    for (int32_t i = old_polyphony; i < polyphony_; ++i) {
      group_[i].tonic = group_[0].tonic + i * 0.01f;
    }
    if (active_group_ >= polyphony_) {
      active_group_ = 0;
    }
  }
  
 private:
  void ProcessEnvelopes(float shape, uint8_t* flags, float* values);
  
  void ComputeRegistration(
    float gain,
    float registration,
    float* amplitudes) {
  registration *= (kRegistrationTableSize - 1.001f);
  MAKE_INTEGRAL_FRACTIONAL(registration);
  float total = 0.0f;
  for (int32_t i = 0; i < numHarmonics * 2; ++i) {
    float a = registrations[registration_integral][i];
    float b = registrations[registration_integral + 1][i];
    amplitudes[i] = a + (b - a) * registration_fractional;
    total += amplitudes[i];
  }

  float modulation = group_[active_group_].active_envelope ? 
      gain + group_[active_group_].vca_cv * group_[active_group_].vca_level : 
      group_[active_group_].vca_cv + group_[active_group_].vca_level;
  CONSTRAIN(modulation, 0.0f, 1.0f);
  
  for (int32_t i = 0; i < numHarmonics * 2; ++i) {
    amplitudes[i] = modulation * amplitudes[i] / total;
  }
}

  void ProcessFilter(
    float envelope,
    float* out,
    float* aux,
    size_t size) {

  for (size_t i = 0; i < size; ++i) {
    filter_in_buffer_[i] = out[i] + aux[i];
  }

  float exp_freq = exp_amp(group_[active_group_].filter_frequency);    
  float modulation = group_[active_group_].active_envelope ? 
      (envelope + group_[active_group_].filter_cv) * group_[active_group_].filter_amount : 
      group_[active_group_].filter_cv * group_[active_group_].filter_amount;

  float total_mod = exp_freq + modulation;
  CONSTRAIN(total_mod, 0.0f, 1.0f);
  std::fill(&out[0], &out[size], 0.0f);
  std::fill(&aux[0], &aux[size], 0.0f);
  filter_.set_f_q<FREQUENCY_FAST>(total_mod * 0.1f, 1.75f);
  float o1;
  for (size_t i = 0; i < size; ++i) {
    o1 = filter_.Process<FILTER_MODE_LOW_PASS>(filter_in_buffer_[i]);
    filter_out_buffer_[i] = filter_.Process<FILTER_MODE_LOW_PASS>(o1);
  }      
  
  for (size_t j = 0; j < size; ++j) {
    out[j] += filter_out_buffer_[j] * 0.5f;
    aux[j] += filter_out_buffer_[j] * 0.5f;
  }
  
}

  float exp_amp(float x) {
    // Clamp x to the range [0, 1]
    x = fminf(fmaxf(x, 0.0f), 1.0f);
    // Compute the amplification using an exponential function
    return (expf(3.0f * (x - 0.75f)) / 2) - 0.05f;
  }
  
  StringSynthVoice<numHarmonics> voice_[stringSynthVoices];
  Voice group_[maxStringSynthPolyphony];
  
  stmlib::Svf filter_;
  Ensemble ensemble_;
  Reverb reverb_;
  Chorus chorus_;
  Limiter limiter_;

  int32_t num_voices_;
  int32_t active_group_;
  uint32_t step_counter_;
  int32_t polyphony_;
  int32_t acquisition_delay_;
  
  ChordOrganFxType fx_type_;
  
  NoteFilter note_filter_;
  
  float filter_in_buffer_[kMaxBlockSize];
  float filter_out_buffer_[kMaxBlockSize];
  
  bool clear_fx_;
  
  DISALLOW_COPY_AND_ASSIGN(ChordOrgan);
};

}  // namespace rings

#endif  // RINGS_DSP_CHORD_ORGAN_H_
