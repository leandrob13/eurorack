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

#include "rings/dsp/arpeggiator.h"
#include "rings/dsp/dsp.h"
#include "rings/dsp/fx/chorus.h"
#include "rings/dsp/fx/delay.h"
#include "rings/dsp/fx/ensemble.h"
#include "rings/dsp/fx/reverb.h"
#include "rings/dsp/limiter.h"
#include "rings/dsp/note_filter.h"
#include "rings/dsp/patch.h"
#include "rings/dsp/performance_state.h"
#include "rings/dsp/string_synth_envelope.h"
#include "rings/dsp/string_synth_voice.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/hysteresis_quantizer.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/stmlib.h"

namespace rings {

const int32_t stringSynthVoices = 12;
const int32_t chord_size = 4;
const int32_t numHarmonics = 3;
// const size_t max_delay = static_cast<size_t>(32768);
const size_t max_delay = static_cast<size_t>(2048);

enum ChordOrganFxType {
  DELAY,
  CHORUS,
  REVERB,
  DELAY_2,
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
  float vca_level;        // Damping attenuator
  float vca_cv;           // Damping CV
  float filter_frequency; // Position pot
  float filter_amount;    // Position attenueverter
  float filter_cv;        // Position CV
  float filter_resonance; // Modal capture: bank button held + brightness pot
  float delay_time;
  float feedback;
  float registration_amount;
};

const int32_t kRegistrationTableSize = 11;
const float registrations[kRegistrationTableSize][numHarmonics * 2] = {
    {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.1f, 0.0f, 0.0f, 1.0f, 0.0f},
    {1.0f, 0.5f, 1.0f, 0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    {0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f}, {0.0f, 0.5f, 1.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
};

class ChordStringSynth {
public:
  ChordStringSynth() {}
  ~ChordStringSynth() {}

  void Init(uint16_t *reverb_buffer);

  void Process(const PerformanceState &performance_state, const Patch &patch,
               const float *in, float *out, float *aux, size_t size);

  inline void set_fx(ChordOrganFxType fx_type) {
    if ((fx_type % 3) != (fx_type_ % 3)) {
      clear_fx_ = true;
    }
    fx_type_ = fx_type;
  }

  inline void set_bank(int32_t bank) {
    if (bank != bank_) {
      bank_changed_ = true;
    }
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

    float drone = shape < 0.995f ? 0.0f : 1.0f;

    synth.envelope.set_ar(attack_rate, decay_rate);
    float value = synth.envelope.Process(flag);
    value = value + (1.0f - value) * drone;

    return value;
  }

  void ComputeRegistration(float gain, float registration, float *amplitudes) {
    registration *= (kRegistrationTableSize - 1.001f);
    MAKE_INTEGRAL_FRACTIONAL(registration);
    float total = 0.0f;
    for (int32_t i = 0; i < numHarmonics * 2; ++i) {
      float a = registrations[registration_integral][i];
      float b = registrations[registration_integral + 1][i];
      amplitudes[i] = a + (b - a) * registration_fractional;
      total += amplitudes[i];
    }

    float modulation = synth.active_envelope
                           ? gain + synth.vca_cv * synth.vca_level
                           : synth.vca_cv + synth.vca_level;
    CONSTRAIN(modulation, 0.0f, 1.0f);

    for (int32_t i = 0; i < numHarmonics * 2; ++i) {
      amplitudes[i] = modulation * amplitudes[i] / total;
    }
  }

  inline float NoteToFrequency(float midi_note) {
    midi_note -= 9.0f;
    CONSTRAIN(midi_note, -128.0f, 127.0f);
    float a0 = (440.0f / 8.0f) / 48000.0f;
    return a0 * 0.25f * SemitonesToRatio(midi_note);
  }

  // Plaits-style VCF, "subtle" flavor — corresponds to the harmonics 0.5→0
  // path in plaits/dsp/engine2/virtual_analog_vcf_engine.cc: dual-cascaded
  // LP is always engaged (stage2_gain = 1) and the second stage's gentle Q
  // (`0.5 + 0.025·q`) rounds off the resonance peak from the first stage.
  // Drive gain starts at 1.0 with no signal loss at zero resonance and only
  // attenuates as resonance increases, to keep self-oscillation peaks tame.
  //
  // Cutoff math runs entirely in semitone space:
  //   pot   → 120·(filter_frequency − 0.2)  semitones above tonic
  //   env   → envelope · normalized_atten · 60 semitones (≈5 octaves at peak)
  //   CV    → unity-gain add (CV·60 semitones) when envelope is on
  //   no-env → attenuverter goes back on CV (CV · normalized_atten · 60)
  // Attenuverter is normalized by the LAW_QUADRATIC_BIPOLAR peak (≈3.3) so
  // "full attenuverter" maps cleanly to the chosen depth.
  void ComputeFilterTargets(float envelope_value, float *cutoff_target,
                            float *q_target, float *gain_target,
                            float *stage2_target) {
    const float kAttenPeak = 3.3f;            // see LAW_QUADRATIC_BIPOLAR
    const float kInvAttenPeak = 1.0f / kAttenPeak;
    const float kFilterDepthSemitones = 84.0f; // 7 octaves of sweep

    // Linearise the attenuverter: cv_scaler applies LAW_QUADRATIC_BIPOLAR
    // (filter_amount = sign(p)·p²·4·3.3), which squashes response near
    // center — 1/4 of physical travel resolves to only ~6% of depth. We
    // undo the square with a sqrt so depth tracks knob travel linearly.
    float atten_abs = fabsf(synth.filter_amount) * kInvAttenPeak;
    float atten_norm = sqrtf(atten_abs);
    if (synth.filter_amount < 0.0f) atten_norm = -atten_norm;

    float cutoff_semitones = 120.0f * (synth.filter_frequency - 0.2f);
    if (synth.active_envelope) {
      cutoff_semitones += envelope_value * atten_norm * kFilterDepthSemitones;
      cutoff_semitones += synth.filter_cv * kFilterDepthSemitones;
    } else {
      cutoff_semitones += synth.filter_cv * atten_norm * kFilterDepthSemitones;
    }

    float f0 = NoteToFrequency(synth.tonic);
    float cutoff = f0 * SemitonesToRatio(cutoff_semitones);
    CONSTRAIN(cutoff, 0.0f, 1.0f);
    *cutoff_target = cutoff;

    float resonance = synth.filter_resonance;
    CONSTRAIN(resonance, 0.0f, 1.0f);
    float resonance_sqr = resonance * resonance;
    *q_target = resonance_sqr * resonance_sqr * 48.0f;

    // Always-on stage-2 cascade — defines the subtle flavor.
    *stage2_target = 1.0f;

    // Linear taper that preserves loudness at r=0 and compensates downward
    // for the resonance peak as r approaches 1.
    *gain_target = 1.0f - 0.5f * resonance;
  }

  void MaybeResetFilterState(float cutoff_target, float q_target,
                             float gain_target, float stage2_target) {
    if (bank_changed_) {
      previous_cutoff_ = cutoff_target;
      previous_q_ = q_target;
      previous_gain_ = gain_target;
      previous_stage2_gain_ = stage2_target;
      svf_[0].Reset();
      svf_[1].Reset();
      bank_changed_ = false;
    }
  }

  void ProcessFilterLP(float envelope, float *out, float *aux, size_t size) {
    float cutoff_target, q_target, gain_target, stage2_target;
    ComputeFilterTargets(envelope, &cutoff_target, &q_target, &gain_target,
                         &stage2_target);
    MaybeResetFilterState(cutoff_target, q_target, gain_target, stage2_target);

    stmlib::ParameterInterpolator cutoff_mod(&previous_cutoff_, cutoff_target, size);
    stmlib::ParameterInterpolator q_mod(&previous_q_, q_target, size);
    stmlib::ParameterInterpolator gain_mod(&previous_gain_, gain_target, size);
    stmlib::ParameterInterpolator stage2_mod(&previous_stage2_gain_, stage2_target, size);

    for (size_t i = 0; i < size; ++i) {
      // Plaits clamps cutoff to 0.25 (Nyquist/2 = 12 kHz at 48 kHz) — Rings'
      // chord-string filter is expected to "fully open" past the audible
      // range, so we only guard against going over Nyquist (set_f_q's tan
      // approximation degrades sharply past ~0.5).
      const float f = std::min(cutoff_mod.Next(), 0.49f);
      const float q = q_mod.Next();
      const float g = gain_mod.Next();
      const float s2 = stage2_mod.Next();

      svf_[0].set_f_q<stmlib::FREQUENCY_FAST>(f, 0.5f + q);
      svf_[1].set_f_q<stmlib::FREQUENCY_FAST>(f, 0.5f + 0.025f * q);

      const float in_sample = stmlib::SoftClip((out[i] + aux[i]) * g);
      float lp = svf_[0].Process<stmlib::FILTER_MODE_LOW_PASS>(in_sample);
      lp = stmlib::SoftClip(lp * g);
      lp += s2 * (stmlib::SoftClip(svf_[1].Process<stmlib::FILTER_MODE_LOW_PASS>(lp)) - lp);

      out[i] = lp * 0.5f;
      aux[i] = lp * 0.5f;
    }
  }

  void ProcessFilterBP(float envelope, float *out, float *aux, size_t size) {
    float cutoff_target, q_target, gain_target, stage2_target;
    ComputeFilterTargets(envelope, &cutoff_target, &q_target, &gain_target,
                         &stage2_target);
    MaybeResetFilterState(cutoff_target, q_target, gain_target, stage2_target);

    stmlib::ParameterInterpolator cutoff_mod(&previous_cutoff_, cutoff_target, size);
    stmlib::ParameterInterpolator q_mod(&previous_q_, q_target, size);
    stmlib::ParameterInterpolator gain_mod(&previous_gain_, gain_target, size);

    for (size_t i = 0; i < size; ++i) {
      const float f = std::min(cutoff_mod.Next(), 0.49f);
      const float q = q_mod.Next();
      const float g = gain_mod.Next();

      // Q baseline 2.0 matches the old fixed-Q BP — Plaits' 0.5 baseline
      // is fine for LP cascading but produces no audible peak on a BP and
      // collapses the band to a notch-like attenuation.
      svf_[0].set_f_q<stmlib::FREQUENCY_FAST>(f, 2.0f + q);

      const float in_sample = stmlib::SoftClip((out[i] + aux[i]) * g);
      float bp = svf_[0].Process<stmlib::FILTER_MODE_BAND_PASS>(in_sample);
      bp = stmlib::SoftClip(bp * g);

      out[i] = bp * 0.5f;
      aux[i] = bp * 0.5f;
    }
  }

  void ProcessFilterHP(float envelope, float *out, float *aux, size_t size) {
    float cutoff_target, q_target, gain_target, stage2_target;
    ComputeFilterTargets(envelope, &cutoff_target, &q_target, &gain_target,
                         &stage2_target);
    MaybeResetFilterState(cutoff_target, q_target, gain_target, stage2_target);

    stmlib::ParameterInterpolator cutoff_mod(&previous_cutoff_, cutoff_target, size);
    stmlib::ParameterInterpolator q_mod(&previous_q_, q_target, size);
    stmlib::ParameterInterpolator gain_mod(&previous_gain_, gain_target, size);

    for (size_t i = 0; i < size; ++i) {
      const float f = std::min(cutoff_mod.Next(), 0.49f);
      const float q = q_mod.Next();
      const float g = gain_mod.Next();

      svf_[0].set_f_q<stmlib::FREQUENCY_FAST>(f, 2.0f + q);

      const float in_sample = stmlib::SoftClip((out[i] + aux[i]) * g);
      float hp = svf_[0].Process<stmlib::FILTER_MODE_HIGH_PASS>(in_sample);
      hp = stmlib::SoftClip(hp * g);

      out[i] = hp * 0.5f;
      aux[i] = hp * 0.5f;
    }
  }

  Synth synth;
  Delay delay_;
  stmlib::Svf svf_[2];
  // 12-step hysteresis quantizer driving arpeggiator mode×range selection,
  // ported from plaits/dsp/engine2/chiptune_engine: pattern/3 → mode,
  // 1<<(pattern%3) → range (1, 2, or 4 octaves).
  stmlib::HysteresisQuantizer2 arp_pattern_selector_;
  Ensemble ensemble_;
  Reverb reverb_;
  Chorus chorus_;
  Limiter limiter_;

  int32_t bank_;
  ChordOrganFxType fx_type_;

  NoteFilter note_filter_;

  float previous_cutoff_;
  float previous_q_;
  float previous_gain_;
  float previous_stage2_gain_;
  float fnote_;
  bool clear_fx_;
  bool bank_changed_;
  bool previous_strum;

  DISALLOW_COPY_AND_ASSIGN(ChordStringSynth);
};

} // namespace rings

#endif // RINGS_DSP_CHORD_ORGAN_H_
