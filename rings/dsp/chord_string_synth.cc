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

#include "rings/dsp/chord_string_synth.h"
#include "rings/dsp/chords.h"
#include "rings/dsp/dsp.h"

namespace rings {

using namespace std;
using namespace stmlib;

void ChordStringSynth::Init(uint16_t* reverb_buffer) {
  bank_ = 1;
  fx_type_ = FILTER;

  for (int32_t i = 0; i < stringSynthVoices; ++i) {
    synth.voice[i].Init();
  }
  previous_strum = false;
  synth.tonic = 0.0f;
  synth.envelope.Init();
  synth.arp.Init();
  
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
      0.004f); // Prevent a sharp edge to partly leak on the previous synth.
}

struct ChordNote {
  float note;
  float amplitude;
};

void ChordStringSynth::Process(
    const PerformanceState& performance_state,
    const Patch& patch,
    const float* in,
    float* out,
    float* aux,
    size_t size) {
  // Assign note to a synth.
  uint8_t envelope_flag = 0;
  
  note_filter_.Process(performance_state.note, performance_state.strum);

  synth.tonic = performance_state.tonic;
  float fnote = note_filter_.note();
  float hysteresis = fnote - fnote_ > 0.0f ? -0.05f : +0.05f;
  fnote_ = fnote + hysteresis + 0.25f;
  synth.chord = static_cast<int16_t>(floor(fnote_));
  synth.chord_transpose = static_cast<int16_t>(synth.chord / 12) * 12.0f;
  synth.genre = performance_state.genre;
  synth.vca_level = performance_state.vca_level;
  synth.vca_cv = performance_state.vca_cv;
  synth.filter_frequency = performance_state.filter_frequency;
  synth.filter_cv = performance_state.filter_cv;
  synth.filter_amount = performance_state.filter_amount;
  synth.active_envelope = performance_state.envelope <= 0.98f;
  bool arpeggiated = performance_state.arp != 0;

  float envelope = performance_state.envelope;

  if (!arpeggiated) synth.arp.Reset();
  if (performance_state.gate) {
    envelope_flag = previous_strum ? envelope_flag : ENVELOPE_FLAG_RISING_EDGE;
    
    if (arpeggiated && performance_state.strum) {
      int range = performance_state.arp < 0 ? 1 : 2;
      int mode = performance_state.arp < 0 ? -1 * performance_state.arp : performance_state.arp;
      synth.arp.set_mode(ArpeggiatorMode(mode - 1));
      synth.arp.set_range(range);
      synth.arp.Clock(chord_size);
    }
    envelope_flag |= ENVELOPE_FLAG_GATE;  
  } 
  previous_strum = performance_state.gate;
  bool clocked = arpeggiated && !performance_state.internal_strum;

  // Process envelopes.
  float envelope_value = ProcessEnvelopes(envelope, envelope_flag);
  
  copy(&in[0], &in[size], &aux[0]);
  copy(&in[0], &in[size], &out[0]);

  ChordNote notes[chord_size];
  float harmonics[numHarmonics * 2];
  
  ComputeRegistration(
      envelope_value,
      patch.brightness,
      harmonics);
  
  int16_t chord = synth.chord % 12;
  int cn = synth.arp.note();
  for (int32_t i = 0; i < chord_size; ++i) {
    int index = clocked ? cn : i;
    float n = genre_chords[bank_ - 1][synth.genre][chord][index];
    float oct_down = (bank_ == 1 && synth.genre < 3) ? 12.0f : 24.0f; 
    notes[i].note = n - oct_down * (1 - synth.arp.octave());
    notes[i].amplitude = n >= 0.0f && n <= 17.0f ? 1.0f : 0.7f;
  }

  for (int32_t chord_note = 0; chord_note < chord_size; ++chord_note) {
    float note = 0.0f;
    note += synth.tonic; // Freq pot root note transpose
    note += synth.chord_transpose; // Chord transpose
    note += performance_state.fm;
    note += notes[chord_note].note; // Chord note
    
    float amplitudes[numHarmonics * 2];
    for (int32_t i = 0; i < numHarmonics * 2; ++i) {
      amplitudes[i] = notes[chord_note].amplitude * harmonics[i];
    }
    
    // Fold truncated harmonics.
    size_t num_harmonics = numHarmonics;
    for (int32_t i = num_harmonics; i < numHarmonics; ++i) {
      amplitudes[2 * (num_harmonics - 1)] += amplitudes[2 * i];
      amplitudes[2 * (num_harmonics - 1) + 1] += amplitudes[2 * i + 1];
    }

    float frequency = SemitonesToRatio(note - 69.0f) * a3;
    synth.voice[chord_note].Render(
        frequency,
        amplitudes,
        num_harmonics,
        chord_note & 1 ? out : aux,
        size);
  }
  
  if (clear_fx_) {
    reverb_.Clear();
    clear_fx_ = false;
  }
  
  switch (fx_type_) {
    case FILTER:
      ProcessFilter<FILTER_MODE_LOW_PASS>(envelope_value * 0.25f, out, aux, size);
      break;

    case FILTER_2:
      ProcessFilter<FILTER_MODE_HIGH_PASS>(envelope_value * 0.25f, out, aux, size);
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