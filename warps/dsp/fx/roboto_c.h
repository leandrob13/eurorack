// Copyright 2026 Symbiote.
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
// Roboto Plan C: faithful HT8950 model.
//   Mode 0 PITCH         : 7-step SOLA pitch shifter (steps -9..+9 st in 3 st).
//   Mode 1 ROBOT         : fixed-period grain replay — forces output fundamental
//                          to a fixed frequency regardless of input pitch.
//   Mode 2 PITCH_VIBRATO : mode 0 + 8 Hz vibrato (±1 st on the shift ratio).
//   Mode 3 ROBOT_VIBRATO : mode 1 + 8 Hz vibrato on the grain rate.
//
// Robot mode is the literal HT8950 "lock the read clock" architecture: input is
// continuously written into a small buffer; two overlapping voices replay
// length-T grains every T seconds with a sin^2 window. Output fundamental =
// 1/T, independent of input pitch. Formants survive because each grain replays
// the input spectrum locally.

#ifndef WARPS_DSP_FX_ROBOTO_C_H_
#define WARPS_DSP_FX_ROBOTO_C_H_

#include <math.h>

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/parameter_interpolator.h"

#include "warps/dsp/fx/pitch_shifter.h"

namespace warps {

class RobotoC {
 public:
  RobotoC() { }
  ~RobotoC() { }

  enum Mode {
    MODE_PITCH = 0,
    MODE_ROBOT,
    MODE_PITCH_VIBRATO,
    MODE_ROBOT_VIBRATO,
  };

  // Grain buffer: power of two for cheap masking. 4096 samples ~ 85 ms @ 48 k,
  // so minimum robot fundamental ~24 Hz (= sample_rate * 2 / kGrainBufSize)
  // before grains alias against the buffer. Smaller buffer = snappier formant
  // tracking (read region refreshes every kGrainBufSize/grain_len cycles).
  // Stored as int16_t in the unused upper half of the shared 32k FX buffer
  // (PitchShifter only touches entries [0, 16384) — see pitch_shifter.h kHalf).
  static const int32_t kGrainBufSize = 4096;
  static const int32_t kGrainBufMask = kGrainBufSize - 1;
  static const int32_t kGrainBufOffset = 16384;  // start past PitchShifter region

  void Init(uint16_t* buffer, float sample_rate) {
    sample_rate_ = sample_rate;
    grain_buf_ = reinterpret_cast<int16_t*>(buffer) + kGrainBufOffset;
    pitch_shifter_.Init(buffer);
    pitch_shifter_.set_voicing(PitchShifter::VOICING_CONTINUOUS);
    pitch_shifter_.set_feedback(0.0f);
    pitch_shifter_.set_mix(1.0f);

    dc_in_.Init();
    dc_in_.set_f<stmlib::FREQUENCY_FAST>(30.0f / sample_rate_);
    dc_out_.Init();
    dc_out_.set_f<stmlib::FREQUENCY_FAST>(15.0f / sample_rate_);

    vib_phase_ = 0.0f;
    vib_inc_ = 8.0f / sample_rate_;

    for (int32_t i = 0; i < kGrainBufSize; ++i) grain_buf_[i] = 0;
    grain_write_ = 0;
    voice_phase_[0] = 0.0f;
    voice_phase_[1] = 0.5f;
    robot_freq_ = 100.0f;
    freeze_ = false;

    bit_levels_ = 255.0f;
    sr_phase_ = 0.0f;
    sr_ratio_ = 1.0f;
    held_ = 0.0f;

    mix_ = 1.0f;
    mix_target_ = 1.0f;
    mode_ = MODE_PITCH;
    pitch_semitones_ = 0.0f;
  }

  void Clear() {
    pitch_shifter_.Clear();
    dc_in_.Reset();
    dc_out_.Reset();
    vib_phase_ = 0.0f;
    sr_phase_ = 0.0f;
    held_ = 0.0f;
    for (int32_t i = 0; i < kGrainBufSize; ++i) grain_buf_[i] = 0;
    grain_write_ = 0;
    voice_phase_[0] = 0.0f;
    voice_phase_[1] = 0.5f;
    freeze_ = false;
  }

  // LEVEL 1 CV -> grain-buffer freeze gate. High CV (above 0.7) halts writes
  // in robot mode; the read keeps cycling through the last buffer contents,
  // holding whatever vowel/grain was loaded. Threshold chosen above the
  // unpatched cv_scaler default (~0.6) so unpatched jack = no freeze. Pitch
  // modes ignore this — they have no grain buffer.
  inline void set_freeze(float cv01) {
    freeze_ = (cv01 > 0.7f);
  }

  // PITCH: caller passes (parameters_.note - 60.0f). We recenter and rescale
  // so the full LEVEL 1 pot sweep maps cleanly onto the 7 HT8950-style steps:
  // -9, -6, -3, 0, +3, +6, +9 semitones.
  //
  // With the LEVEL 1 jack unpatched, cv_scaler injects a +24 normalling bump,
  // so `parameters_.note - 60` actually ranges [-24 .. +36] over the pot's
  // [0..1] sweep — pot midpoint = +6 (not 0). We recenter on +6 and rescale
  // by 18/60 so pot=0 -> -9, pot=0.5 -> 0, pot=1 -> +9, before quantizing.
  inline void set_pitch(float raw_note_offset) {
    float centered = (raw_note_offset - 6.0f) * (18.0f / 60.0f);
    if (centered < -10.5f) centered = -10.5f;
    if (centered > 10.5f) centered = 10.5f;
    int32_t step;
    if (centered >= 0.0f) {
      step = static_cast<int32_t>(centered * (1.0f / 3.0f) + 0.5f);
    } else {
      step = -static_cast<int32_t>(-centered * (1.0f / 3.0f) + 0.5f);
    }
    if (step < -3) step = -3;
    if (step > 3) step = 3;
    pitch_semitones_ = static_cast<float>(step * 3);
  }

  // ALGO -> single master-clock emulation. Like the HT8950's external R-set
  // master oscillator: one control drives BOTH the ADC/DAC sample rate (=>
  // ZOH SR-reduce ratio) AND the fixed readback rate that determines the
  // robot fundamental. There is no independent V/oct on robot pitch — that's
  // the authentic chip behavior.
  //
  //   algo01 = 0.0 (CCW) -> clock ~500 Hz   (slowest, robot fund 10 Hz buzz)
  //   algo01 = 0.5       -> clock ~3.2 kHz  (HT8950 territory, ~63 Hz fund)
  //   algo01 = 1.0 (CW)  -> clock ~20 kHz   (fastest, fund ~400 Hz)
  //
  // Range factor 40 (500 Hz -> 20 kHz). Divider 50 + clamp range [10, 400]
  // mean both endpoints sit exactly at the clamps — the entire pot sweep
  // audibly moves both bandwidth AND robot fund with no dead zones.
  inline void set_master_clock(float algo01) {
    if (algo01 < 0.0f) algo01 = 0.0f;
    if (algo01 > 1.0f) algo01 = 1.0f;
    // Exponential from 500 Hz (CCW) up to 20 kHz (CW).
    const float kClockMin = 500.0f;
    const float kClockMax = 20000.0f;
    float clock_hz = kClockMin * powf(kClockMax / kClockMin, algo01);
    // SR-reduce ratio = clock / sample_rate.
    float ratio = clock_hz / sample_rate_;
    if (ratio > 1.0f) ratio = 1.0f;
    sr_ratio_ = ratio;
    // Robot fundamental = master_clock / 50. Range [10, 400] Hz across sweep.
    // At 10 Hz it's subharmonic buzz; at 400 Hz high robot. Both extremes
    // sit at the clamps so there is no dead zone.
    const float kRobotDivider = 50.0f;
    float fund = clock_hz * (1.0f / kRobotDivider);
    if (fund < 10.0f) fund = 10.0f;
    if (fund > 400.0f) fund = 400.0f;
    robot_freq_ = fund;
  }

  // MOD -> bit depth. CCW = chip-faithful 8-bit; CW = 1-bit pulse-train.
  inline void set_bits(float mod01) {
    if (mod01 < 0.0f) mod01 = 0.0f;
    if (mod01 > 1.0f) mod01 = 1.0f;
    float bits = 8.0f - mod01 * 7.0f;  // 8 .. 1
    bit_levels_ = powf(2.0f, bits) - 1.0f;
  }

  inline void set_mix(float pot01) {
    if (pot01 < 0.0f) pot01 = 0.0f;
    if (pot01 > 1.0f) pot01 = 1.0f;
    mix_target_ = pot01;
  }

  inline void set_mode(int32_t shape) {
    if (shape < 0) shape = 0;
    if (shape > 3) shape = 3;
    mode_ = static_cast<Mode>(shape);
  }

  // Process one block. `in` is the engine input (= IN2 audio, DC-blocked in
  // place). `fm_src` is IN1 audio — used in ROBOT modes as an audio-rate FM
  // modulator on the robot fundamental (per-sample bend of grain_inc). PITCH
  // modes ignore `fm_src`. `scratch`, `out_main`, `out_aux` are the usual
  // distinct buffers (out_main may alias fm_src — fm_src is consumed before
  // out_main is written).
  //
  // Signal flow matches the HT8950 silicon: input is quantized + downsampled
  // at the ADC FIRST, then the buffer (pitch shifter / grain replay) sees
  // that already-degraded audio. The DAC reads back at the same lo-fi rate.
  //
  //   in -> DC block -> SoftLimit -> bitcrush -> ZOH SR-reduce
  //                                                   |
  //                                                   v
  //                              [pitch shift OR grain replay]   <- fm_src
  //                                                   |
  //                                                   v
  //                                              DC block -> out
  void Process(float* in, float* fm_src, float* scratch,
               float* out_main, float* out_aux, size_t size) {
    const bool use_robot =
        (mode_ == MODE_ROBOT || mode_ == MODE_ROBOT_VIBRATO);
    const bool use_vibrato =
        (mode_ == MODE_PITCH_VIBRATO || mode_ == MODE_ROBOT_VIBRATO);

    // 1. DC-block input.
    float* engine_src = in;
    for (size_t i = 0; i < size; ++i) {
      engine_src[i] = dc_in_.Process<stmlib::FILTER_MODE_HIGH_PASS>(engine_src[i]);
    }

    // 2. 8 Hz vibrato LFO (block rate is fine — 2 ms at 96 sa/block vs 125 ms
    //    vibrato period).
    float vib_st = 0.0f;
    if (use_vibrato) {
      vib_phase_ += vib_inc_ * static_cast<float>(size);
      while (vib_phase_ >= 1.0f) vib_phase_ -= 1.0f;
      vib_st = sinf(vib_phase_ * 2.0f * 3.14159265358979f);  // ±1 st
    }

    // 3. Stash clean DC-blocked dry in out_aux for the final crossfade.
    for (size_t i = 0; i < size; ++i) {
      out_aux[i] = engine_src[i];
    }

    // 6. ADC emulation: SoftLimit -> bitcrush -> ZOH SR-reduce, IN PLACE on
    //    the engine source. Afterwards engine_src holds the 8-bit /
    //    clock-rate audio that the chip's pitch shifter / robot readback
    //    actually operates on.
    for (size_t i = 0; i < size; ++i) {
      engine_src[i] = stmlib::SoftLimit(engine_src[i]);
    }
    {
      const float levels = bit_levels_;
      const float inv_levels = 1.0f / levels;
      for (size_t i = 0; i < size; ++i) {
        float x = engine_src[i] * levels;
        int32_t q = static_cast<int32_t>(x + (x >= 0.0f ? 0.5f : -0.5f));
        engine_src[i] = static_cast<float>(q) * inv_levels;
      }
    }
    {
      float phase = sr_phase_;
      const float ratio = sr_ratio_;
      float held = held_;
      for (size_t i = 0; i < size; ++i) {
        phase += ratio;
        if (phase >= 1.0f) {
          phase -= 1.0f;
          held = engine_src[i];
        }
        engine_src[i] = held;
      }
      sr_phase_ = phase;
      held_ = held;
    }

    // 7. Snapshot the post-ADC ("what the chip sees") input into scratch for
    //    the OUT2 alt monitor.
    for (size_t i = 0; i < size; ++i) {
      scratch[i] = engine_src[i];
    }

    // 6. Either ROBOT (grain replay) or PITCH (SOLA shifter) — operating on
    //    the already-degraded audio.
    if (use_robot) {
      // Faithful HT8950 robot: read pointer cycles through a SHORT window
      // [0, grain_len) of the buffer at the robot rate, while the write
      // pointer continues sweeping through the FULL buffer in the background.
      //
      // IN1 audio (fm_src) audio-rate-modulates the phase advance per sample
      // -> the robot fundamental bends with IN1 in real time. grain_len_base
      // (the read-window width) stays fixed at the unmodulated value so each
      // cycle reads the same buffer region — only the cycle DURATION varies.
      // That gives clean through-FM character without clobbering content.
      float f = robot_freq_;
      if (use_vibrato) {
        f *= powf(2.0f, vib_st * (1.0f / 12.0f));
      }
      float grain_inc_base = f / sample_rate_;
      const float min_inc = 2.0f / static_cast<float>(kGrainBufSize);
      if (grain_inc_base < min_inc) grain_inc_base = min_inc;
      if (grain_inc_base > 0.25f) grain_inc_base = 0.25f;
      const float grain_len_base = 1.0f / grain_inc_base;
      const float kPi = 3.14159265358979f;
      const float kQ = 32767.0f;
      const float kInvQ = 1.0f / kQ;
      const float kFmDepth = 0.5f;  // ±50% bend at IN1 = ±1.0

      for (size_t i = 0; i < size; ++i) {
        // Per-sample audio-rate FM on grain phase advance.
        float inst_inc = grain_inc_base * (1.0f + fm_src[i] * kFmDepth);
        if (inst_inc < min_inc) inst_inc = min_inc;
        if (inst_inc > 0.25f) inst_inc = 0.25f;

        // Read BEFORE write so the read sees the previous lap's contents,
        // not the sample we're about to write this tick.
        float out = 0.0f;
        for (int32_t v = 0; v < 2; ++v) {
          voice_phase_[v] += inst_inc;
          if (voice_phase_[v] >= 1.0f) voice_phase_[v] -= 1.0f;
          int32_t read_pos =
              static_cast<int32_t>(voice_phase_[v] * grain_len_base) &
              kGrainBufMask;
          float s = static_cast<float>(grain_buf_[read_pos]) * kInvQ;
          // sin^2 window; two voices offset by 0.5 sum to constant 1.
          float w = sinf(voice_phase_[v] * kPi);
          w *= w;
          out += s * w;
        }

        // Write input AFTER the read; advances through the full buffer.
        // When frozen, skip the write — read keeps cycling through the last
        // captured content, holding the current vowel/grain.
        if (!freeze_) {
          float xq = engine_src[i] * kQ;
          if (xq > 32767.0f) xq = 32767.0f;
          if (xq < -32768.0f) xq = -32768.0f;
          grain_buf_[grain_write_ & kGrainBufMask] = static_cast<int16_t>(xq);
          grain_write_ = (grain_write_ + 1) & kGrainBufMask;
        }

        engine_src[i] = out;
      }
    } else {
      // 7-step pitch shift (+ optional vibrato on top of the quantized step).
      // PitchShifter needs a stereo pair; copy engine_src into out_main as the
      // second channel (out_main is about to be overwritten by the crossfade).
      pitch_shifter_.set_pitch(pitch_semitones_ + vib_st, 0.0f);
      for (size_t i = 0; i < size; ++i) out_main[i] = engine_src[i];
      pitch_shifter_.Process(engine_src, out_main, size);
    }

    // 8. DC-block on the way out.
    for (size_t i = 0; i < size; ++i) {
      engine_src[i] = dc_out_.Process<stmlib::FILTER_MODE_HIGH_PASS>(engine_src[i]);
    }

    // 9. Dry/wet crossfade. Dry is the clean DC-blocked IN2 in out_aux;
    //    wet is the engine output in engine_src.
    stmlib::ParameterInterpolator mix(&mix_, mix_target_, size);
    for (size_t i = 0; i < size; ++i) {
      float m = mix.Next();
      float dry = out_aux[i];
      out_main[i] = dry + (engine_src[i] - dry) * m;
    }

    // 9. Repurpose out_aux as the post-ADC snapshot (the "buffer audio" —
    //    8-bit @ master-clock-rate, what the chip's pitch/robot engine reads).
    for (size_t i = 0; i < size; ++i) {
      out_aux[i] = scratch[i];
    }
  }

 private:
  PitchShifter pitch_shifter_;

  stmlib::OnePole dc_in_;
  stmlib::OnePole dc_out_;

  Mode mode_;
  float pitch_semitones_;
  float vib_phase_;
  float vib_inc_;

  // Grain replay (robot) state. Buffer lives in the shared 32k FX buffer.
  // Read cycles through buf[0, grain_len); write cycles through full buffer.
  int16_t* grain_buf_;
  int32_t grain_write_;
  float voice_phase_[2];
  float robot_freq_;
  bool freeze_;

  float bit_levels_;
  float sr_phase_;
  float sr_ratio_;
  float held_;

  float mix_;
  float mix_target_;
  float sample_rate_;

  DISALLOW_COPY_AND_ASSIGN(RobotoC);
};

}  // namespace warps

#endif  // WARPS_DSP_FX_ROBOTO_C_H_
