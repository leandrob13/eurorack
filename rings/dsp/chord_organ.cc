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
// String synth part.

#include "rings/dsp/chord_organ.h"
#include "rings/dsp/chords.h"
#include "rings/dsp/dsp.h"

namespace rings {

using namespace std;
using namespace stmlib;

void ChordOrgan::Init(uint16_t* reverb_buffer) {
  polyphony_ = 1;
  fx_type_ = FILTER;

  for (int32_t i = 0; i < stringSynthVoices; ++i) {
    voice_[i].Init();
  }
  
  for (int32_t i = 0; i < maxStringSynthPolyphony; ++i) {
    group_[i].tonic = 0.0f;
    group_[i].envelope.Init();
  }
  
  filter_.Init();
  limiter_.Init();
  
  reverb_.Init(reverb_buffer);
  chorus_.Init(reverb_buffer);
  ensemble_.Init(reverb_buffer);
  
  note_filter_.Init(
      kSampleRate / kMaxBlockSize,
      0.001f,  // Lag time with a sharp edge on the V/Oct input or trigger.
      0.005f,  // Lag time after the trigger has been received.
      0.050f,  // Time to transition from reactive to filtered.
      0.004f); // Prevent a sharp edge to partly leak on the previous voice.
}

void ChordOrgan::ProcessEnvelopes(
    float shape,
    uint8_t* flags,
    float* values) {
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
  
  for (int32_t i = 0; i < polyphony_; ++i) {
    float drone = shape < 0.98f ? 0.0f : (shape - 0.98f) * 55.0f;
    if (drone >= 1.0f) drone = 1.0f;

    group_[i].envelope.set_ad(attack_rate, decay_rate);
    float value = group_[i].envelope.Process(flags[i]);
    values[i] = value + (1.0f - value) * drone;
  }
}

struct ChordNote {
  float note;
  float amplitude;
};

void ChordOrgan::Process(
    const PerformanceState& performance_state,
    const Patch& patch,
    const float* in,
    float* out,
    float* aux,
    size_t size) {
  // Assign note to a voice.
  uint8_t envelope_flags[maxStringSynthPolyphony];
  
  fill(&envelope_flags[0], &envelope_flags[polyphony_], 0);
  note_filter_.Process(performance_state.note, performance_state.strum);
  if (performance_state.strum) {
    envelope_flags[active_group_] = ENVELOPE_FLAG_FALLING_EDGE;
    active_group_ = (active_group_ + 1) % polyphony_;
    envelope_flags[active_group_] = ENVELOPE_FLAG_RISING_EDGE;
    acquisition_delay_ = 3;
  }
  
  group_[active_group_].tonic = performance_state.tonic;
  group_[active_group_].chord = static_cast<int16_t>(ceil(note_filter_.note()));
  group_[active_group_].chord_transpose = static_cast<int16_t>(group_[active_group_].chord / 12) * 12.0f;
  group_[active_group_].genre = performance_state.genre;
  group_[active_group_].vca_level = performance_state.vca_level;
  group_[active_group_].vca_cv = performance_state.vca_cv;
  group_[active_group_].filter_frequency = performance_state.filter_frequency;
  group_[active_group_].filter_cv = performance_state.filter_cv;
  group_[active_group_].filter_amount = performance_state.filter_amount;
  group_[active_group_].active_envelope = performance_state.envelope <= 0.98f;
  envelope_flags[active_group_] |= ENVELOPE_FLAG_GATE;

  // Process envelopes.
  float envelope_values[maxStringSynthPolyphony];
  ProcessEnvelopes(performance_state.envelope, envelope_flags, envelope_values);
  
  copy(&in[0], &in[size], &aux[0]);
  copy(&in[0], &in[size], &out[0]);

  for (int32_t group = 0; group < polyphony_; ++group) {
    ChordNote notes[chord_size];
    float harmonics[numHarmonics * 2];
    
    ComputeRegistration(
        envelope_values[group] * 0.25f,
        patch.brightness,
        harmonics);
    
    // Note enough polyphony for smooth transition between chords.
    int16_t chord = group_[group].chord % 12;
    for (int32_t i = 0; i < chord_size; ++i) {
      float n = genre_chords[group_[group].genre][chord][i];
      notes[i].note = n - 12.0f;
      notes[i].amplitude = n >= 0.0f && n <= 17.0f ? 1.0f : 0.7f;
    }

    for (int32_t chord_note = 0; chord_note < chord_size; ++chord_note) {
      float note = 0.0f;
      note += group_[group].tonic; // Freq pot root note transpose
      note += group_[group].chord_transpose; // Chord transpose
      note += performance_state.fm;
      note += notes[chord_note].note; // Chord note
      
      float amplitudes[numHarmonics * 2];
      for (int32_t i = 0; i < numHarmonics * 2; ++i) {
        amplitudes[i] = notes[chord_note].amplitude * harmonics[i];
      }
      
      // Fold truncated harmonics.
      size_t num_harmonics = polyphony_ >= 2 && chord_note < 2
          ? numHarmonics - 1
          : numHarmonics;
      for (int32_t i = num_harmonics; i < numHarmonics; ++i) {
        amplitudes[2 * (num_harmonics - 1)] += amplitudes[2 * i];
        amplitudes[2 * (num_harmonics - 1) + 1] += amplitudes[2 * i + 1];
      }

      float frequency = SemitonesToRatio(note - 69.0f) * a3;
      voice_[group * chord_size + chord_note].Render(
          frequency,
          amplitudes,
          num_harmonics,
          (group + chord_note) & 1 ? out : aux,
          size);
    }
  }
  
  if (clear_fx_) {
    reverb_.Clear();
    clear_fx_ = false;
  }
  
  switch (fx_type_) {
    case FILTER:
      ProcessFilter<FILTER_MODE_LOW_PASS>(envelope_values[active_group_] * 0.25f, out, aux, size);
      break;
      
    case FILTER_2:
      ProcessFilter<FILTER_MODE_HIGH_PASS>(envelope_values[active_group_] * 0.25f, out, aux, size);
      break;

    case CHORUS:
      chorus_.set_amount(patch.position);
      chorus_.set_depth(0.15f + 0.5f * patch.position);
      chorus_.Process(out, aux, size);
      break;
    
    case ENSEMBLE:
      ensemble_.set_amount(patch.position * (2.0f - patch.position));
      ensemble_.set_depth(0.2f + 0.8f * patch.position * patch.position);
      ensemble_.Process(out, aux, size);
      break;
  
    case REVERB:
    case REVERB_2:
      reverb_.set_amount(patch.position * 0.5f);
      reverb_.set_diffusion(0.625f);
      reverb_.set_time(fx_type_ == REVERB
        ? (0.5f + 0.49f * patch.position)
        : (0.3f + 0.6f * patch.position));
      reverb_.set_input_gain(0.2f);
      reverb_.set_lp(fx_type_ == REVERB ? 0.3f : 0.6f);
      reverb_.Process(out, aux, size);
      break;
    
    default:
      break;
  }

  // Prevent main signal cancellation when EVEN gets summed with ODD through
  // normalization.
  for (size_t i = 0; i < size; ++i) {
    aux[i] = -aux[i];
  }
  limiter_.Process(out, aux, size, 1.0f);
}

}  // namespace rings