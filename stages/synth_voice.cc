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

#include "stages/bipolar_fold_lut.h"

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
// One Process() call always covers a full kBlockSize block (8 samples), so the
// control rate (used for the envelope and LFO) is fixed.
const float kBlockRate = kSr / float(kBlockSize);
// Base oscillator frequency with no CV patched and the coarse slider centred:
// C3 (one octave below middle C) for a deeper default voice.
const float kBaseFreq = 130.81278f;
// Linear-FM index for the ch1 CV -> osc2 path. Calibrated cv is ~0.0625 per
// volt, so this makes ~+4V roughly double osc2's frequency (+1 octave).
const float kFmDepth = 4.0f;

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

// Triangle wavefolder using Warps' smooth fold table (kBipolarFoldLut, lifted
// from warps/resources.cc). Warps' fold curve already folds at its centre, so
// a bare "drive = amount" never yields a clean triangle; instead we both push
// further into the table as the amount rises (4 -> 10 folds) and crossfade from
// the dry triangle to the folded signal, so the pot sweeps clean -> heavily
// folded. Indexed bipolarly via `+ 2048`, exactly like Warps' ALGORITHM_FOLD;
// the 2000 scale keeps the window inside the 4097-entry table with margin.
inline void ApplyFold(float* buf, float amount, size_t size) {
  if (amount < 0.001f) {
    return;  // clean triangle
  }
  // Warps fold (output * -0.8, the exact curve from warps/dsp/modulator.cc).
  // The table already folds at its centre, so jumping straight from a clean
  // triangle to the fold output is an audible step. To avoid that we crossfade
  // the dry triangle into the folded signal across the bottom of the pot only
  // (gone by ~25%); above that it is the pure Warps fold. The fold drive
  // (0.02 -> 1.02) still scales over the whole range -- at the top x*1.02*2000
  // reaches the table edge for the full fold.
  const float drive = 0.02f + amount;
  float dry = 1.0f - amount * 4.0f;  // dry path gone by amount = 0.25
  if (dry < 0.0f) dry = 0.0f;
  for (size_t i = 0; i < size; ++i) {
    float x = buf[i];
    CONSTRAIN(x, -1.0f, 1.0f);
    const float folded =
        Interpolate(kBipolarFoldLut + 2048, x * drive, 2000.0f) * -0.8f;
    buf[i] = x * dry + folded * (1.0f - dry);
  }
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
  osc1_saw_[0].Init();
  osc1_saw_[1].Init();
  osc1_sub_.Init();
  eg_.Init();
  svf_[0].Init();
  svf_[1].Init();

  lfo_phase_ = 0.0f;
  lfo_value_ = 0.0f;
  lfo_sh_value_ = 0.0f;
  lfo_fade_level_ = 0.0f;
  noise_lp_ = 0.0f;

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

void SynthVoice::RenderSuperSaw(
    float frequency, float detune, float* out, size_t size) {
  // Centre saw, always at full level.
  osc1_.Render<OSCILLATOR_SHAPE_SAW>(frequency, 0.5f, out, size);

  // The two detuned satellites fade in *with* the detune amount. The cores have
  // independent (and unsynchronised) phase, so at detune 0 three same-frequency
  // saws would comb-filter into a hollow tone rather than a clean saw; fading
  // the satellites to silence there guarantees a clean single saw at shape 0.
  if (detune < 0.001f) {
    return;
  }
  // Satellite *level* ramps up quickly over the first part of the pot (so only
  // the very bottom stays a clean single saw, free of same-frequency combing),
  // then holds. The detune *width* keeps widening across the whole range, so
  // the effect gets more pronounced the further the pot is turned.
  float blend = detune * 6.667f;  // reaches full level by detune ~0.15
  if (blend > 1.0f) blend = 1.0f;
  const float d = detune * 0.06f;            // up to ~+/-0.06 ratio (~1 semi/side)
  const float s_gain = 0.45f * blend;        // satellites
  const float c_gain = 1.0f - 0.3f * blend;  // centre eases back a touch

  osc1_saw_[0].Render<OSCILLATOR_SHAPE_SAW>(
      frequency * (1.0f + d), 0.5f, super_buffer_, size);
  for (size_t i = 0; i < size; ++i) {
    out[i] = out[i] * c_gain + super_buffer_[i] * s_gain;
  }
  osc1_saw_[1].Render<OSCILLATOR_SHAPE_SAW>(
      frequency * (1.0f - d), 0.5f, super_buffer_, size);
  for (size_t i = 0; i < size; ++i) {
    out[i] += super_buffer_[i] * s_gain;
  }
}

void SynthVoice::RenderNoise(float tone, float* out, size_t size) {
  // White noise through a one-pole low-pass whose cutoff tracks the tone pot
  // (dark rumble -> near-white). Compensate the level the low-pass removes so
  // the perceived loudness stays roughly constant across the pot.
  const float coeff = 0.02f + tone * tone * 0.95f;
  const float comp = 1.0f + (1.0f - tone) * 2.0f;
  for (size_t i = 0; i < size; ++i) {
    const float white = 2.0f * Random::GetFloat() - 1.0f;
    noise_lp_ += coeff * (white - noise_lp_);
    out[i] = noise_lp_ * comp;
  }
}

void SynthVoice::RenderFilter(
    const SynthPatch& patch, float cutoff_mod, const float* in, float* out,
    size_t size) {
  float cutoff = patch.cutoff + cutoff_mod;
  float fc = CutoffToHz(cutoff);
  float fc_norm = fc / kSr;
  CONSTRAIN(fc_norm, 0.0005f, 0.24f);

  // Band-pass / high-pass: a single 2-pole SVF stage with optional post-drive,
  // as before.
  if (patch.filter_mode == SYNTH_FILTER_BP ||
      patch.filter_mode == SYNTH_FILTER_HP) {
    float q = 0.7f + patch.resonance * patch.resonance * 17.0f;
    svf_[0].set_f_q<FREQUENCY_DIRTY>(fc_norm, q);
    if (patch.filter_mode == SYNTH_FILTER_BP) {
      svf_[0].Process<FILTER_MODE_BAND_PASS>(in, out, size);
    } else {
      svf_[0].Process<FILTER_MODE_HIGH_PASS>(in, out, size);
    }
    if (patch.drive > 0.001f) {
      float drive_gain = 1.0f + patch.drive * 3.0f;
      for (size_t i = 0; i < size; ++i) {
        out[i] = SoftClip(out[i] * drive_gain) * 1.5f;
      }
    }
    return;
  }

  // Low-pass: virtual-analog cascade calibrated like Plaits' VA-with-VCF engine
  // (plaits/dsp/engine2/virtual_analog_vcf_engine.cc). Two 2-pole SVF stages
  // with soft clipping in the signal path give a saturating, MS-20-ish
  // resonance. The resonance follows a quartic taper (gentle until the top),
  // and the second stage's Q is heavily damped so it only adds slope, not peak.
  //   green / LP_AGGRESSIVE : full 4-pole, high Q, hot drive  -> screams.
  //   off   / LP_GENTLE     : single 2-pole, tame Q, clean    -> smooth.
  const bool aggressive = patch.filter_mode == SYNTH_FILTER_LP_AGGRESSIVE;

  const float res = patch.resonance;
  const float res_sqr = res * res;
  const float q = res_sqr * res_sqr * (aggressive ? 48.0f : 14.0f);
  // Input drive into the soft clipper; aggressive mode runs hotter so the
  // resonance saturates. The drive pot pushes both modes further.
  const float gain = (aggressive ? 1.0f : 0.8f) + patch.drive * 2.0f;

  svf_[0].set_f_q<FREQUENCY_DIRTY>(fc_norm, 0.5f + q);
  svf_[1].set_f_q<FREQUENCY_DIRTY>(fc_norm, 0.5f + 0.025f * q);

  for (size_t i = 0; i < size; ++i) {
    float lp = svf_[0].Process<FILTER_MODE_LOW_PASS>(SoftClip(in[i] * gain));
    lp = SoftClip(lp * gain);
    if (aggressive) {
      // Blend in the second pole pair -> saturating 4-pole.
      lp = SoftClip(svf_[1].Process<FILTER_MODE_LOW_PASS>(lp));
    }
    out[i] = lp * 1.4f;
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
  float osc1_semi =
      patch.base_pitch + patch.osc1_coarse + patch.osc1_fine + lfo_pitch;
  float osc2_semi =
      patch.base_pitch + patch.osc2_coarse + patch.osc2_fine + lfo_pitch;

  float f1 = kBaseFreq * SemitonesToRatio(osc1_semi) / kSr;
  float f2 = kBaseFreq * SemitonesToRatio(osc2_semi) / kSr;
  CONSTRAIN(f1, kMinFrequency, kMaxFrequency);
  CONSTRAIN(f2, kMinFrequency, kMaxFrequency);

  // --- Oscillators -------------------------------------------------------
  // ch0 hold+pot routes the envelope to the oscillator shapes (saw detune /
  // square PWM / triangle fold) with this depth; 0 = off. Applied to both
  // oscillators, like the LFO's shape (PWM) destination.
  float shape_env = patch.env_to_shape * env_value_;
  float shape1 = patch.osc1_shape + lfo_pwm + shape_env;
  float shape2 = patch.osc2_shape + lfo_pwm + shape_env;
  CONSTRAIN(shape1, 0.0f, 1.0f);
  CONSTRAIN(shape2, 0.0f, 1.0f);

  // osc1: super-saw on the saw slot, wavefolder on the triangle slot, plain
  // band-limited shape otherwise. (osc1 has no FM / sync input.)
  if (patch.osc1_wave == SYNTH_OSC_WAVE_SAW) {
    RenderSuperSaw(f1, shape1, osc1_buffer_, size);
  } else if (patch.osc1_wave == SYNTH_OSC_WAVE_TRIANGLE) {
    osc1_.Render<OSCILLATOR_SHAPE_TRIANGLE>(f1, 0.5f, osc1_buffer_, size);
    ApplyFold(osc1_buffer_, shape1, size);
  } else {
    RenderOscWave(&osc1_, patch.osc1_wave, f1, shape1, osc1_buffer_, size);
  }

  // Optional sub-oscillator: a square one octave below osc1, summed into the
  // osc1 signal (so it follows osc1 through the mixer / filter / VCA).
  if (patch.sub_osc) {
    osc1_sub_.Render<OSCILLATOR_SHAPE_SQUARE>(
        f1 * 0.5f, 0.5f, super_buffer_, size);
    for (size_t i = 0; i < size; ++i) {
      osc1_buffer_[i] += super_buffer_[i] * 0.5f;
    }
  }

  // Hard sync: reset osc2 phase on a rising edge of the ch1 gate.
  if (sync_gate) {
    bool rising = false;
    for (size_t i = 0; i < size; ++i) {
      rising = rising || (sync_gate[i] & GATE_FLAG_RISING);
    }
    if (rising) {
      osc2_.SyncReset();
    }
  }

  // osc2: the "off" slot is noise (shape = tone); the saw slot stays a single
  // saw; the triangle slot gets the wavefolder. Linear FM (ch1 CV) applies to
  // the pitched waves only.
  if (patch.osc2_wave == SYNTH_OSC_WAVE_SINE) {
    RenderNoise(shape2, osc2_buffer_, size);
  } else {
    if (fm) {
      // Linear through-zero FM into osc2 (ch1 CV). The CV is sampled once per
      // block, so this is a (fast) pitch modulation rather than audio-rate FM.
      for (size_t i = 0; i < size; ++i) {
        fm_buffer_[i] = fm[i] * kFmDepth;
      }
      float pw2 = 0.05f + 0.9f * shape2;
      switch (patch.osc2_wave) {
        case SYNTH_OSC_WAVE_SQUARE:
          osc2_.Render<OSCILLATOR_SHAPE_SQUARE>(f2, pw2, fm_buffer_, osc2_buffer_, size);
          break;
        case SYNTH_OSC_WAVE_TRIANGLE:
          osc2_.Render<OSCILLATOR_SHAPE_TRIANGLE>(f2, 0.5f, fm_buffer_, osc2_buffer_, size);
          break;
        case SYNTH_OSC_WAVE_SAW:
        default:
          osc2_.Render<OSCILLATOR_SHAPE_SAW>(f2, 0.5f, fm_buffer_, osc2_buffer_, size);
          break;
      }
    } else {
      RenderOscWave(&osc2_, patch.osc2_wave, f2, shape2, osc2_buffer_, size);
    }
    if (patch.osc2_wave == SYNTH_OSC_WAVE_TRIANGLE) {
      ApplyFold(osc2_buffer_, shape2, size);
    }
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
      lfo_cutoff;
  RenderFilter(patch, cutoff_mod, mix_buffer_, mix_buffer_, size);

  // --- VCA + outputs -----------------------------------------------------
  // The ch5 CV (accent/level) is added to the envelope so a steady CV can hold
  // the VCA open without a gate (droning).
  float amp_target = env_value_ + patch.accent;
  CONSTRAIN(amp_target, 0.0f, 1.0f);
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
