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
// Symbiote "Mono" synth voice: 2 oscillators -> mixer -> filter -> VCA, with a
// DADSR-style envelope, an LFO and a small modulation matrix. The six Stages
// sections drive one monophonic voice, laid out left->right like a synth block
// diagram (see stages/docs/synth_plan.md).

#ifndef STAGES_SYNTH_VOICE_H_
#define STAGES_SYNTH_VOICE_H_

#include "stmlib/stmlib.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/utils/gate_flags.h"

#include "stages/envelope.h"
#include "stages/io_buffer.h"
#include "stages/oscillator.h"

namespace stages {

enum SynthOscWave {
  SYNTH_OSC_WAVE_SAW,       // osc1: super-saw (shape = detune); osc2: single saw
  SYNTH_OSC_WAVE_SQUARE,    // PWM via shape
  SYNTH_OSC_WAVE_TRIANGLE,  // wavefolder, shape = fold amount (both oscillators)
  SYNTH_OSC_WAVE_SINE,      // osc1: sine; osc2: noise (shape = tone)
  SYNTH_OSC_WAVE_LAST
};

enum SynthFilterMode {
  SYNTH_FILTER_LP_AGGRESSIVE,  // green: MS-20-style saturating 4-pole low-pass
  SYNTH_FILTER_BP,             // orange: band-pass
  SYNTH_FILTER_HP,             // red: high-pass
  SYNTH_FILTER_LP_GENTLE,      // off (unlit): smooth 2-pole low-pass
  SYNTH_FILTER_MODE_LAST
};

enum SynthLfoWave {
  SYNTH_LFO_TRIANGLE,
  SYNTH_LFO_SAW,
  SYNTH_LFO_SQUARE,
  SYNTH_LFO_SAMPLE_HOLD,
  SYNTH_LFO_WAVE_LAST
};

enum SynthLfoDest {
  SYNTH_LFO_DEST_PITCH,
  SYNTH_LFO_DEST_PWM,
  SYNTH_LFO_DEST_CUTOFF,
  SYNTH_LFO_DEST_LAST
};

// Everything the voice needs for one block. Continuous values are normalized
// 0..1 unless noted; the host (stages.cc) reads panel + jacks and fills this in.
struct SynthPatch {
  // Pitch (semitones). base_pitch is the calibrated V/oct from ch0-in.
  float base_pitch;
  float osc1_coarse;   // semitones
  float osc1_fine;     // semitones
  float osc1_shape;    // 0..1 (PW / morph)
  int osc1_wave;
  float osc2_coarse;   // semitones (interval)
  float osc2_fine;     // semitones
  float osc2_shape;    // 0..1
  int osc2_wave;
  float mix;           // 0 = osc1 only, 1 = osc2 only
  bool sub_osc;        // osc1 sub-oscillator (square, one octave down) on/off

  // Filter
  int filter_mode;
  float cutoff;        // 0..1
  float resonance;     // 0..1
  float drive;         // 0..1
  float key_track;     // 0..1
  float cutoff_cv;     // additive, in 0..1 cutoff units (ch2-in)

  // Envelope (0..1 panel units)
  float attack;
  float decay;
  float sustain;
  float release;
  float attack_curve;  // 0..1
  float decrel_curve;  // 0..1
  bool loop;
  float env_to_filter; // -1..1
  float env_to_shape;  // 0..1: envelope -> osc shape (detune / PWM / fold) depth

  // LFO
  float lfo_rate;      // 0..1
  float lfo_depth;     // 0..1
  int lfo_wave;
  int lfo_dest;
  float lfo_fade;      // 0..1 (fade-in time)

  // Performance
  float accent;        // 0..1 (ch5-in -> VCA + cutoff)
};

// Per-block render outputs (one buffer per panel channel, size <= kBlockSize).
struct SynthOutputs {
  float* main;     // ch5
  float* env;      // ch4
  float* lfo;      // ch3
  float* filter;   // ch2
  float* osc2;     // ch1
  float* osc1;     // ch0
};

class SynthVoice {
 public:
  SynthVoice() { }
  ~SynthVoice() { }

  void Init();

  // gate     : ch4 envelope gate (per sample, size flags).
  // sync_gate: ch1 hard-sync gate for osc2 (per sample), may be NULL.
  // fm       : ch1 linear-FM CV into osc2 (per sample), may be NULL.
  void Render(
      const SynthPatch& patch,
      const stmlib::GateFlags* gate,
      const stmlib::GateFlags* sync_gate,
      const float* fm,
      const SynthOutputs& out,
      size_t size);

  inline float env_value() const { return env_value_; }
  inline float lfo_value() const { return lfo_value_; }

 private:
  float RenderLfo(const SynthPatch& patch);
  void RenderFilter(
      const SynthPatch& patch, float cutoff_mod, const float* in, float* out,
      size_t size);
  // osc1 super-saw: 3 detuned saw cores, spread set by the shape pot.
  void RenderSuperSaw(float frequency, float detune, float* out, size_t size);
  // osc2 "off" slot: white noise tilted dark->bright by the shape pot.
  void RenderNoise(float tone, float* out, size_t size);

  Oscillator osc1_;
  Oscillator osc2_;
  // Two extra detuned saw cores for osc1's super-saw (osc1_ is the centre core).
  Oscillator osc1_saw_[2];
  // Sub-oscillator one octave below osc1 (square), toggled by a medium press.
  Oscillator osc1_sub_;
  Envelope eg_;
  // Two cascaded 2-pole SVF stages for the virtual-analog low-pass (Plaits-VCF
  // style); svf_[0] alone also serves the band-pass / high-pass modes.
  stmlib::Svf svf_[2];

  float lfo_phase_;
  float lfo_value_;
  float lfo_sh_value_;
  float lfo_fade_level_;

  float noise_lp_;  // one-pole state for the osc2 noise tone control

  float env_value_;
  float previous_env_;
  float previous_amp_;

  // Smoothed audio-path scratch.
  float mix_buffer_[kBlockSize];
  float osc1_buffer_[kBlockSize];
  float osc2_buffer_[kBlockSize];
  float fm_buffer_[kBlockSize];
  float super_buffer_[kBlockSize];  // scratch for super-saw satellite cores

  DISALLOW_COPY_AND_ASSIGN(SynthVoice);
};

}  // namespace stages

#endif  // STAGES_SYNTH_VOICE_H_
