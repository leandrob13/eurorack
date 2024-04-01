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
#include "rings/dsp/arpeggiator.h"

namespace rings {

const int32_t stringSynthVoices = 12;
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

struct Synth {
  float tonic;
  StringSynthEnvelope envelope;
  StringSynthVoice<numHarmonics> voice[stringSynthVoices];
  Arpeggiator arp;
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

class ChordStringSynth {
 public:
  ChordStringSynth() { }
  ~ChordStringSynth() { }
  
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

  inline void set_bank(int32_t bank) {
    bank_ = bank;
  }
  
 private:
  float ProcessEnvelopes(float shape, uint8_t flag) {
  float decay = shape;
  float attack = 0.0f;
  if (shape < 0.5f) {
    attack = 0.0f;
  } else {
    attack = (shape - 0.5f) * 2.0f;
  }
  
  // Convert the arbitrary values to actual units.
  float period = kSampleRate / kMaxBlockSize;
  float attack_time = SemitonesToRatio(attack * 96.0f) * 0.005f * period;
  // float decay_time = SemitonesToRatio(decay * 96.0f) * 0.125f * period;
  float decay_time = SemitonesToRatio(decay * 84.0f) * 0.180f * period;
  float attack_rate = 1.0f / attack_time;
  float decay_rate = 1.0f / decay_time;
  
  float drone = shape < 0.98f ? 0.0f : (shape - 0.98f) * 55.0f;
  if (drone >= 1.0f) drone = 1.0f;

  synth.envelope.set_ad(attack_rate, decay_rate);
  float value = synth.envelope.Process(flag);
  value = value + (1.0f - value) * drone;
  
  return value;
}
  
void ComputeRegistration(
  float gain,
  float registration,
  float* amplitudes
) {
  registration *= (kRegistrationTableSize - 1.001f);
  MAKE_INTEGRAL_FRACTIONAL(registration);
  float total = 0.0f;
  for (int32_t i = 0; i < numHarmonics * 2; ++i) {
    float a = registrations[registration_integral][i];
    float b = registrations[registration_integral + 1][i];
    amplitudes[i] = a + (b - a) * registration_fractional;
    total += amplitudes[i];
  }

  float modulation = synth.active_envelope ? 
      gain + synth.vca_cv * synth.vca_level : 
      synth.vca_cv + synth.vca_level;
  CONSTRAIN(modulation, 0.0f, 1.0f);
  
  for (int32_t i = 0; i < numHarmonics * 2; ++i) {
    amplitudes[i] = modulation * amplitudes[i] / total;
  }
}

inline float NoteToFrequency(float midi_note) {
  midi_note -= 9.0f;
  CONSTRAIN(midi_note, -128.0f, 127.0f);
  return a3 * 0.25f * SemitonesToRatio(midi_note);
}

template<FilterMode mode>
void ProcessFilter(
  float envelope,
  float* out,
  float* aux,
  size_t size
) {

    for (size_t i = 0; i < size; ++i) {
      filter_in_buffer_[i] = out[i] + aux[i];
    }

    float f0 = NoteToFrequency(synth.tonic - 69.0f);
    float cutoff = 2.0f * f0 * SemitonesToRatio(120.0f * synth.filter_frequency);
    
    float modulation = synth.active_envelope ? 
        (envelope + synth.filter_cv) * synth.filter_amount : 
        synth.filter_cv * synth.filter_amount;

    float total_mod = cutoff + modulation;
    CONSTRAIN(total_mod, 0.0f, 1.0f);
    std::fill(&out[0], &out[size], 0.0f);
    std::fill(&aux[0], &aux[size], 0.0f);

    filter_.set_f_q<FREQUENCY_FAST>(total_mod, 0.25f);
    float o1;
    for (size_t i = 0; i < size; ++i) {
      o1 = filter_.Process<mode>(filter_in_buffer_[i]);
      filter_out_buffer_[i] = mode == FILTER_MODE_LOW_PASS ? filter_.Process<mode>(o1) : o1;
    }
          
    for (size_t j = 0; j < size; ++j) {
      out[j] += filter_out_buffer_[j] * 0.5f;
      aux[j] += filter_out_buffer_[j] * 0.5f;
    }
}

  Synth synth;
  
  stmlib::Svf filter_;
  Ensemble ensemble_;
  Reverb reverb_;
  Chorus chorus_;
  Limiter limiter_;

  int32_t bank_;
  ChordOrganFxType fx_type_;
  
  NoteFilter note_filter_;
  
  float filter_in_buffer_[kMaxBlockSize];
  float filter_out_buffer_[kMaxBlockSize];
  float fnote_;
  bool clear_fx_;
  
  DISALLOW_COPY_AND_ASSIGN(ChordStringSynth);
};

}  // namespace rings

#endif  // RINGS_DSP_CHORD_ORGAN_H_
