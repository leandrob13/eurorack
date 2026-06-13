// Copyright 2024 Symbiote.
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
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
//
// -----------------------------------------------------------------------------
//
// Symbiote "Mono" synth voice implementation.

#include "stages/synth_voice.h"

#include <algorithm>
#include <cmath>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

namespace stages {

using namespace stmlib;

namespace {

const float kSr = 31250.0f;
const float kNyquist = kSr * 0.5f;
// One Process() call always covers a full kBlockSize block (8 samples), so the
// control rate (used for the envelope and LFO) is fixed.
const float kBlockRate = kSr / float(kBlockSize);
const float kMiddleC = 261.6255f;

// Standard cubic soft clipper: x - x^3/3, output bounded to +/- 2/3.
inline float SoftClip(float x) {
  if (x < -1.0f) return -2.0f / 3.0f;
  if (x > 1.0f) return 2.0f / 3.0f;
  return x - x * x * x * (1.0f / 3.0f);
}

// Map a 0..1 cutoff parameter to Hz, exponentially over ~20 Hz .. ~10 kHz.
inline float CutoffToHz(float cutoff) {
  CONSTRAIN(cutoff, 0.0f, 1.0f);
  return 20.0f * SemitonesToRatio(cutoff * 9.0f * 12.0f);
}

inline float RenderOscWave(
    Oscillator* osc, int wave, float f, float shape, float* out, size_t size) {
  // shape -> PW for the square wave; ignored by the others.
  float pw = 0.05f + 0.9f * shape;
  switch (wave) {
    case SYNTH_OSC_WAVE_SQUARE:
      osc->Render<OSCILLATOR_SHAPE_SQUARE>(f, pw, out, size);
      break;
    case SYNTH_OSC_WAVE_TRIANGLE:
      osc->Render<OSCILLATOR_SHAPE_TRIANGLE>(f, 0.5f, out, size);
      break;
    case SYNTH_OSC_WAVE_SINE:
      osc->Render<OSCILLATOR_SHAPE_SINE>(f, 0.5f, out, size);
      break;
    case SYNTH_OSC_WAVE_SAW:
    default:
      osc->Render<OSCILLATOR_SHAPE_SAW>(f, 0.5f, out, size);
      break;
  }
  return pw;
}

}  // namespace

void SynthVoice::Init() {
  osc1_.Init();
  osc2_.Init();
  eg_.Init();
  svf_.Init();

  std::fill(&ladder_stage_[0], &ladder_stage_[4], 0.0f);
  std::fill(&ladder_delay_[0], &ladder_delay_[4], 0.0f);

  lfo_phase_ = 0.0f;
  lfo_value_ = 0.0f;
  lfo_sh_value_ = 0.0f;
  lfo_fade_level_ = 0.0f;

  env_value_ = 0.0f;
  previous_env_ = 0.0f;
  previous_amp_ = 0.0f;
}

float SynthVoice::RenderLfo(const SynthPatch& patch) {
  // Exponential rate: ~0.05 Hz .. ~40 Hz.
  float hz = 0.05f * SemitonesToRatio(patch.lfo_rate * 9.5f * 12.0f);
  lfo_phase_ += hz / kBlockRate;
  if (lfo_phase_ >= 1.0f) {
    lfo_phase_ -= 1.0f;
    lfo_sh_value_ = 2.0f * Random::GetFloat() - 1.0f;
  }

  float raw;
  switch (patch.lfo_wave) {
    case SYNTH_LFO_SAW:
      raw = 1.0f - 2.0f * lfo_phase_;
      break;
    case SYNTH_LFO_SQUARE:
      raw = lfo_phase_ < 0.5f ? 1.0f : -1.0f;
      break;
    case SYNTH_LFO_SAMPLE_HOLD:
      raw = lfo_sh_value_;
      break;
    case SYNTH_LFO_TRIANGLE:
    default:
      raw = lfo_phase_ < 0.5f
          ? -1.0f + 4.0f * lfo_phase_
          : 3.0f - 4.0f * lfo_phase_;
      break;
  }

  // Fade-in: lfo_fade maps to a time constant; depth scales the result.
  float fade_inc = (patch.lfo_fade < 0.01f)
      ? 1.0f
      : 1.0f / (patch.lfo_fade * patch.lfo_fade * 4.0f * kBlockRate + 1.0f);
  lfo_fade_level_ += fade_inc * (1.0f - lfo_fade_level_);

  lfo_value_ = raw * patch.lfo_depth * lfo_fade_level_;
  return lfo_value_;
}

void SynthVoice::RenderFilter(
    const SynthPatch& patch, float cutoff_mod, const float* in, float* out,
    size_t size) {
  float cutoff = patch.cutoff + cutoff_mod;
  float fc = CutoffToHz(cutoff);

  if (patch.filter_mode == SYNTH_FILTER_LADDER) {
    // Stilson/Smith 4-pole ladder (Paul Kellett's variation), with drive in
    // the feedback path for a saturating resonance.
    float f = fc / kNyquist * 1.16f;
    CONSTRAIN(f, 0.0f, 0.99f);
    float fb = patch.resonance * 4.0f * (1.0f - 0.15f * f * f);
    float drive_gain = 1.0f + patch.drive * 3.0f;
    float in_gain = 0.35013f * (f * f) * (f * f);

    for (size_t i = 0; i < size; ++i) {
      float x = in[i] * drive_gain;
      x = SoftClip(x) * 1.5f;
      x -= ladder_stage_[3] * fb;
      x *= in_gain;
      ladder_stage_[0] = x + 0.3f * ladder_delay_[0] +
          (1.0f - f) * ladder_stage_[0];
      ladder_delay_[0] = x;
      ladder_stage_[1] = ladder_stage_[0] + 0.3f * ladder_delay_[1] +
          (1.0f - f) * ladder_stage_[1];
      ladder_delay_[1] = ladder_stage_[0];
      ladder_stage_[2] = ladder_stage_[1] + 0.3f * ladder_delay_[2] +
          (1.0f - f) * ladder_stage_[2];
      ladder_delay_[2] = ladder_stage_[1];
      ladder_stage_[3] = ladder_stage_[2] + 0.3f * ladder_delay_[3] +
          (1.0f - f) * ladder_stage_[3];
      ladder_delay_[3] = ladder_stage_[2];
      out[i] = ladder_stage_[3] * 4.0f;
    }
    return;
  }

  // State-variable filter modes. Resonance maps to Q ~0.7 .. ~18.
  float fc_norm = fc / kSr;
  CONSTRAIN(fc_norm, 0.0005f, 0.24f);
  float q = 0.7f + patch.resonance * patch.resonance * 17.0f;
  svf_.set_f_q<FREQUENCY_DIRTY>(fc_norm, q);

  switch (patch.filter_mode) {
    case SYNTH_FILTER_BP:
      svf_.Process<FILTER_MODE_BAND_PASS>(in, out, size);
      break;
    case SYNTH_FILTER_HP:
      svf_.Process<FILTER_MODE_HIGH_PASS>(in, out, size);
      break;
    case SYNTH_FILTER_LP:
    default:
      svf_.Process<FILTER_MODE_LOW_PASS>(in, out, size);
      break;
  }

  if (patch.drive > 0.001f) {
    float drive_gain = 1.0f + patch.drive * 3.0f;
    for (size_t i = 0; i < size; ++i) {
      out[i] = SoftClip(out[i] * drive_gain) * 1.5f;
    }
  }
}

void SynthVoice::Render(
    const SynthPatch& patch,
    const GateFlags* gate,
    const GateFlags* sync_gate,
    const float* fm,
    const SynthOutputs& out,
    size_t size) {
  // --- Envelope ----------------------------------------------------------
  eg_.SetDelayLength(0.0f);
  eg_.SetAttackLength(patch.attack);
  eg_.SetHoldLength(0.0f);
  eg_.SetDecayLength(patch.decay);
  eg_.SetSustainLevel(patch.sustain);
  eg_.SetReleaseLength(patch.release);
  eg_.SetAttackCurve(patch.attack_curve);
  eg_.SetDecayCurve(patch.decrel_curve);
  eg_.SetReleaseCurve(patch.decrel_curve);

  bool any_high = false;
  for (size_t i = 0; i < size; ++i) {
    any_high = any_high || (gate[i] & GATE_FLAG_HIGH);
  }
  bool g = any_high;
  if (patch.loop) {
    // Free-running AD: retrigger from idle, fall back to release at sustain.
    EnvelopeStage st = eg_.CurrentStage();
    if (st == IDLE) {
      g = true;
    } else if (st == SUSTAIN) {
      g = false;
    } else {
      g = (st == ATTACK || st == HOLD || st == DECAY);
    }
  }
  eg_.Gate(g);
  env_value_ = eg_.Value();

  // --- LFO ---------------------------------------------------------------
  float lfo = RenderLfo(patch);
  float lfo_pitch = patch.lfo_dest == SYNTH_LFO_DEST_PITCH ? lfo * 12.0f : 0.0f;
  float lfo_pwm = patch.lfo_dest == SYNTH_LFO_DEST_PWM ? lfo * 0.4f : 0.0f;
  float lfo_cutoff = patch.lfo_dest == SYNTH_LFO_DEST_CUTOFF ? lfo * 0.5f : 0.0f;

  // --- Pitch -------------------------------------------------------------
  float env_pitch = patch.env_to_pitch * env_value_ * 24.0f;
  float osc1_semi =
      patch.base_pitch + patch.osc1_coarse + patch.osc1_fine +
      env_pitch + lfo_pitch;
  float osc2_semi =
      patch.base_pitch + patch.osc2_coarse + patch.osc2_fine +
      env_pitch + lfo_pitch;

  float f1 = kMiddleC * SemitonesToRatio(osc1_semi) / kSr;
  float f2 = kMiddleC * SemitonesToRatio(osc2_semi) / kSr;
  CONSTRAIN(f1, kMinFrequency, kMaxFrequency);
  CONSTRAIN(f2, kMinFrequency, kMaxFrequency);

  // --- Oscillators -------------------------------------------------------
  float shape1 = patch.osc1_shape + lfo_pwm;
  float shape2 = patch.osc2_shape + lfo_pwm;
  CONSTRAIN(shape1, 0.0f, 1.0f);
  CONSTRAIN(shape2, 0.0f, 1.0f);

  RenderOscWave(&osc1_, patch.osc1_wave, f1, shape1, osc1_buffer_, size);

  // Hard sync: reset osc2 phase on a rising edge of the ch1 gate.
  if (sync_gate) {
    bool rising = false;
    for (size_t i = 0; i < size; ++i) {
      rising = rising || (sync_gate[i] & GATE_FLAG_RISING);
    }
    if (rising) {
      osc2_.Init();
    }
  }

  if (fm) {
    // Linear through-zero FM into osc2 (ch1 CV).
    for (size_t i = 0; i < size; ++i) {
      fm_buffer_[i] = fm[i];
    }
    float pw2 = 0.05f + 0.9f * shape2;
    switch (patch.osc2_wave) {
      case SYNTH_OSC_WAVE_SQUARE:
        osc2_.Render<OSCILLATOR_SHAPE_SQUARE>(f2, pw2, fm_buffer_, osc2_buffer_, size);
        break;
      case SYNTH_OSC_WAVE_TRIANGLE:
        osc2_.Render<OSCILLATOR_SHAPE_TRIANGLE>(f2, 0.5f, fm_buffer_, osc2_buffer_, size);
        break;
      case SYNTH_OSC_WAVE_SINE:
        osc2_.Render<OSCILLATOR_SHAPE_SINE>(f2, 0.5f, fm_buffer_, osc2_buffer_, size);
        break;
      case SYNTH_OSC_WAVE_SAW:
      default:
        osc2_.Render<OSCILLATOR_SHAPE_SAW>(f2, 0.5f, fm_buffer_, osc2_buffer_, size);
        break;
    }
  } else {
    RenderOscWave(&osc2_, patch.osc2_wave, f2, shape2, osc2_buffer_, size);
  }

  // --- Mixer -------------------------------------------------------------
  float g1 = 1.0f - patch.mix;
  float g2 = patch.mix;
  for (size_t i = 0; i < size; ++i) {
    mix_buffer_[i] = osc1_buffer_[i] * g1 + osc2_buffer_[i] * g2;
  }

  // --- Filter ------------------------------------------------------------
  float key = (patch.base_pitch + patch.osc1_coarse) / 96.0f;
  CONSTRAIN(key, -1.0f, 1.0f);
  float cutoff_mod =
      patch.cutoff_cv +
      patch.env_to_filter * env_value_ +
      patch.key_track * key +
      lfo_cutoff +
      patch.accent * 0.2f;
  RenderFilter(patch, cutoff_mod, mix_buffer_, mix_buffer_, size);

  // --- VCA + outputs -----------------------------------------------------
  float amp_target = env_value_ * (0.6f + 0.4f * patch.accent);
  ParameterInterpolator amp(&previous_amp_, amp_target, size);
  ParameterInterpolator env_cv(&previous_env_, env_value_, size);

  const float kOutGain = 0.6f;
  for (size_t i = 0; i < size; ++i) {
    float a = amp.Next();
    out.main[i] = mix_buffer_[i] * a * kOutGain;
    out.osc1[i] = osc1_buffer_[i] * kOutGain;
    out.osc2[i] = osc2_buffer_[i] * kOutGain;
    out.filter[i] = mix_buffer_[i] * kOutGain;
    out.env[i] = env_cv.Next() * 0.8f;
    out.lfo[i] = lfo * 0.8f;
  }
}

}  // namespace stages
