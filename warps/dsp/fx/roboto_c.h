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

  // Grain buffer: power of two for cheap masking. 2048 samples ~ 43 ms @ 48 k,
  // so minimum robot fundamental ~23 Hz before grains alias against the buffer.
  // Stored as int16_t in the unused upper half of the shared 32k FX buffer
  // (PitchShifter only uses entries [0, 16384) — see pitch_shifter.h kHalf).
  static const int32_t kGrainBufSize = 2048;
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
  //   algo01 = 0.0 (CCW) -> clock ~1.5 kHz  (slowest, robot fund 30 Hz "vader")
  //   algo01 = 0.5       -> clock ~5.5 kHz  (HT8950 territory, ~110 Hz fund)
  //   algo01 = 1.0 (CW)  -> clock ~20 kHz   (fastest, fund ~400 Hz)
  //
  // Range factor (20k/1.5k = ~13.3) is the maximum that keeps both robot-fund
  // clamps (30 .. 400 Hz) from triggering anywhere across the sweep — so every
  // turn of the knob audibly moves both bandwidth AND robot pitch.
  inline void set_master_clock(float algo01) {
    if (algo01 < 0.0f) algo01 = 0.0f;
    if (algo01 > 1.0f) algo01 = 1.0f;
    // Exponential from 1.5 kHz (CCW) up to ~20 kHz (CW).
    const float kClockMin = 1500.0f;
    const float kClockMax = 20000.0f;
    float clock_hz = kClockMin * powf(kClockMax / kClockMin, algo01);
    // SR-reduce ratio = clock / sample_rate.
    float ratio = clock_hz / sample_rate_;
    if (ratio > 1.0f) ratio = 1.0f;
    sr_ratio_ = ratio;
    // Robot fundamental = master_clock / divider. Divider 50 keeps the full
    // sweep inside [30, 400] Hz without clamping (1500/50=30, 20000/50=400).
    const float kRobotDivider = 50.0f;
    float fund = clock_hz * (1.0f / kRobotDivider);
    if (fund < 30.0f) fund = 30.0f;
    if (fund > 400.0f) fund = 400.0f;
    robot_freq_ = fund;
  }

  inline void set_bits(float mod01) {
    if (mod01 < 0.0f) mod01 = 0.0f;
    if (mod01 > 1.0f) mod01 = 1.0f;
    float bits = 8.0f - mod01 * 5.0f;
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

  // Process one block. `in` is the input (DC-blocked in place); `scratch`,
  // `out_main`, `out_aux` must be four DISTINCT, non-aliasing buffers.
  //
  // Signal flow matches the HT8950 silicon: input is quantized + downsampled
  // at the ADC FIRST, then the buffer (pitch shifter / grain replay) sees
  // that already-degraded audio. The DAC reads back at the same lo-fi rate.
  //
  //   in -> DC block -> SoftLimit -> bitcrush -> ZOH SR-reduce
  //                                                   |
  //                                                   v
  //                              [pitch shift OR grain replay]
  //                                                   |
  //                                                   v
  //                                              DC block -> out
  void Process(float* in, float* scratch,
               float* out_main, float* out_aux, size_t size) {
    // 1. DC-block input.
    for (size_t i = 0; i < size; ++i) {
      in[i] = dc_in_.Process<stmlib::FILTER_MODE_HIGH_PASS>(in[i]);
    }

    const bool use_robot =
        (mode_ == MODE_ROBOT || mode_ == MODE_ROBOT_VIBRATO);
    const bool use_vibrato =
        (mode_ == MODE_PITCH_VIBRATO || mode_ == MODE_ROBOT_VIBRATO);

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
      out_aux[i] = in[i];
    }

    // 4. ADC emulation: SoftLimit -> bitcrush -> ZOH SR-reduce, IN PLACE.
    //    After this block `in` is the 8-bit / clock-rate audio that the
    //    chip's pitch shifter / robot readback actually operates on.
    for (size_t i = 0; i < size; ++i) {
      in[i] = stmlib::SoftLimit(in[i]);
    }
    {
      const float levels = bit_levels_;
      const float inv_levels = 1.0f / levels;
      for (size_t i = 0; i < size; ++i) {
        float x = in[i] * levels;
        int32_t q = static_cast<int32_t>(x + (x >= 0.0f ? 0.5f : -0.5f));
        in[i] = static_cast<float>(q) * inv_levels;
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
          held = in[i];
        }
        in[i] = held;
      }
      sr_phase_ = phase;
      held_ = held;
    }

    // 5. Snapshot the post-ADC ("what the chip sees") input into scratch for
    //    the OUT2 alt monitor.
    for (size_t i = 0; i < size; ++i) {
      scratch[i] = in[i];
    }

    // 6. Either ROBOT (grain replay) or PITCH (SOLA shifter) — operating on
    //    the already-degraded audio.
    if (use_robot) {
      // Faithful HT8950 robot: read pointer cycles through a SHORT window
      // [0, grain_len) of the buffer at the robot rate, while the write
      // pointer continues sweeping through the FULL buffer in the background.
      //
      //   output period = grain_len samples => fundamental = sample_rate /
      //                                        grain_len = f_robot.
      //   the read window's contents are refreshed every (kGrainBufSize /
      //   grain_len) robot cycles as the write pointer laps it. Formants
      //   ride the refresh; fundamental stays locked.
      //
      // grain_len <= kGrainBufSize/2 so the write spends > 1 cycle outside
      // the read window (otherwise reads see live writes and the effect
      // collapses to a delayed pass-through).
      float f = robot_freq_;
      if (use_vibrato) {
        f *= powf(2.0f, vib_st * (1.0f / 12.0f));
      }
      float grain_inc = f / sample_rate_;
      const float min_inc = 2.0f / static_cast<float>(kGrainBufSize);
      if (grain_inc < min_inc) grain_inc = min_inc;
      if (grain_inc > 0.25f) grain_inc = 0.25f;
      const float grain_len = 1.0f / grain_inc;
      const float kPi = 3.14159265358979f;
      const float kQ = 32767.0f;
      const float kInvQ = 1.0f / kQ;

      for (size_t i = 0; i < size; ++i) {
        // Read BEFORE write so the read sees the previous lap's contents,
        // not the sample we're about to write this tick.
        float out = 0.0f;
        for (int32_t v = 0; v < 2; ++v) {
          voice_phase_[v] += grain_inc;
          if (voice_phase_[v] >= 1.0f) voice_phase_[v] -= 1.0f;
          int32_t read_pos =
              static_cast<int32_t>(voice_phase_[v] * grain_len) &
              kGrainBufMask;
          float s = static_cast<float>(grain_buf_[read_pos]) * kInvQ;
          // sin^2 window; two voices offset by 0.5 sum to constant 1.
          float w = sinf(voice_phase_[v] * kPi);
          w *= w;
          out += s * w;
        }

        // Write input AFTER the read; advances through the full buffer.
        float xq = in[i] * kQ;
        if (xq > 32767.0f) xq = 32767.0f;
        if (xq < -32768.0f) xq = -32768.0f;
        grain_buf_[grain_write_ & kGrainBufMask] = static_cast<int16_t>(xq);
        grain_write_ = (grain_write_ + 1) & kGrainBufMask;

        in[i] = out;
      }
    } else {
      // 7-step pitch shift (+ optional vibrato on top of the quantized step).
      // PitchShifter needs a stereo pair; reuse scratch as the right channel,
      // but DO NOT overwrite it first — we need scratch to retain the post-ADC
      // snapshot for OUT2. Copy the post-ADC `in` into out_main as the second
      // channel, then run the shifter on (in, out_main). out_main gets clobbered
      // but we're about to overwrite it via the crossfade anyway.
      pitch_shifter_.set_pitch(pitch_semitones_ + vib_st, 0.0f);
      for (size_t i = 0; i < size; ++i) out_main[i] = in[i];
      pitch_shifter_.Process(in, out_main, size);
    }

    // 7. DC-block on the way out.
    for (size_t i = 0; i < size; ++i) {
      in[i] = dc_out_.Process<stmlib::FILTER_MODE_HIGH_PASS>(in[i]);
    }

    // 8. Dry/wet crossfade. Dry is the clean DC-blocked input in out_aux;
    //    wet is the engine output in `in`.
    stmlib::ParameterInterpolator mix(&mix_, mix_target_, size);
    for (size_t i = 0; i < size; ++i) {
      float m = mix.Next();
      float dry = out_aux[i];
      out_main[i] = dry + (in[i] - dry) * m;
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
