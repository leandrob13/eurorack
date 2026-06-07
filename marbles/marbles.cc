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

#include <stm32f4xx_conf.h>

#include "marbles/drivers/clock_inputs.h"
#include "marbles/drivers/dac.h"
#include "marbles/drivers/debug_pin.h"
#include "marbles/drivers/debug_port.h"
#include "marbles/drivers/gate_outputs.h"
#include "marbles/drivers/rng.h"
#include "marbles/drivers/system.h"

#include "marbles/ramp/ramp_extractor.h"
#include "marbles/random/random_generator.h"
#include "marbles/random/random_stream.h"
#include "marbles/random/t_generator.h"
#include "marbles/random/x_y_generator.h"

#include "marbles/clock_self_patching_detector.h"
#include "marbles/cv_reader.h"
#include "marbles/io_buffer.h"
#include "marbles/note_filter.h"
#include "marbles/resources.h"
#include "marbles/scale_recorder.h"
#include "marbles/settings.h"
#include "marbles/tb3po/tb3po_sequencer.h"
#include "marbles/ui.h"

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/hysteresis_quantizer.h"
#include "stmlib/dsp/units.h"

#define PROFILE_INTERRUPT 0
#define PROFILE_RENDER 0

using namespace marbles;
using namespace std;
using namespace stmlib;

const bool test_adc_noise = false;

const int kSampleRate = 32000;
const int kGateDelay = 2;

ClockInputs clock_inputs;
ClockSelfPatchingDetector self_patching_detector[kNumGateOutputs];
CvReader cv_reader;
Dac dac;
DebugPort debug_port;
GateOutputs gate_outputs;
HysteresisQuantizer2 deja_vu_length_quantizer;
IOBuffer io_buffer;
NoteFilter note_filter;
Rng rng;
ScaleRecorder scale_recorder;
Settings settings;
Ui ui;

RandomGenerator random_generator;
RandomStream random_stream;
TGenerator t_generator;
XYGenerator xy_generator;
TB3PoSequencer tb3po;

// Persistent edge-detection state for the TB-3PO seed lifecycle and clock.
uint8_t prev_x_deja_vu = DEJA_VU_OFF;
float prev_grids_ramp = 0.0f;

// External-clock watchdogs (Grids mode). Each counter tracks samples since
// the last rising edge on its clock input. When the gap exceeds ~2× the last
// observed period, TB-3PO's gate is forced off so a stopped upstream sequencer
// can't latch downstream VCAs/ADSRs open.
uint32_t t_clock_silence_samples = 0;
uint32_t t_clock_last_period_samples = kSampleRate / 2;  // 500 ms initial guess
uint32_t x_clock_silence_samples = 0;
uint32_t x_clock_last_period_samples = kSampleRate / 2;

// Default interrupt handlers.
extern "C" {

int __errno;

void NMI_Handler() { }
void HardFault_Handler() { while (1); }
void MemManage_Handler() { while (1); }
void BusFault_Handler() { while (1); }
void UsageFault_Handler() { while (1); }
void SVC_Handler() { }
void DebugMon_Handler() { }
void PendSV_Handler() { }

void SysTick_Handler() {
  IWDG_ReloadCounter();
  ui.Poll();
  if (settings.freshly_baked()) {
    if (debug_port.readable()) {
      uint8_t command = debug_port.Read();
      uint8_t response = ui.HandleFactoryTestingRequest(command);
      debug_port.Write(response);
    }
  }
}

}

IOBuffer::Slice FillBuffer(size_t size) {
  if (PROFILE_INTERRUPT) {
    TIC;
  }
  IOBuffer::Slice s = io_buffer.NextSlice(size);
  
  gate_outputs.Write(s);
  clock_inputs.Read(s, size);

  if (io_buffer.new_block()) {
    cv_reader.Copy(&s.block->adc_value[0]);
    clock_inputs.ReadNormalization(s.block);
  }

  if (rng.readable()) {
    random_stream.Write(rng.data());
  }

  if (PROFILE_INTERRUPT) {
    TOC;
  }
  
  return s;
}

inline uint16_t DacCode(int index, float voltage) {
  CONSTRAIN(voltage, -5.0f, 5.0f);
  const float scale = settings.calibration_data().dac_scale[index];
  const float offset = settings.calibration_data().dac_offset[index];
  return ClipU16(static_cast<int32_t>(voltage * scale + offset));
}

void ProcessTest(IOBuffer::Block* block, size_t size) {
  float parameters[kNumParameters];
  GateFlags hidden_gates[kNumParameters];
  static float phase;
  cv_reader.Process(false, &block->adc_value[0], parameters, hidden_gates);
  for (size_t i = 0; i < size; ++i) {
    phase += 100.0f / static_cast<float>(kSampleRate);
    if (phase >= 1.0f) {
      phase -= 1.0f;
    }
    block->cv_output[0][i] = DacCode(
        0, 4.0 * Interpolate(lut_sine, phase, 256.0f));
    block->cv_output[1][i] = DacCode(
        1, -8.0f * phase + 4.0f);
    block->cv_output[2][i] = DacCode(
        2, (phase < 0.5f ? phase : 1.0f - phase) * 16.0f - 4.0f);
    block->cv_output[3][i] = DacCode(
        3, phase < 0.5f ? -4.0f : 4.0f);

    for (int j = 0; j < 4; ++j) {
      uint16_t dac_code = ui.output_test_forced_dac_code(j);
      if (dac_code) {
        block->cv_output[j][i] = dac_code;
      }
    }
    
    block->gate_output[0][i] = block->input_patched[0]
        ? block->input[0][i]
        : phase < 0.2f;
    block->gate_output[1][i] = phase < 0.5f;
    block->gate_output[2][i] = block->input_patched[1]
        ? block->input[1][i]
        : phase < 0.8f;
  }
}

Ratio y_divider_ratios[] = {
  { 1, 64 },
  { 1, 48 },
  { 1, 32 },
  { 1, 24 },
  { 1, 16 },
  { 1, 12 },
  { 1, 8 },
  { 1, 6 },
  { 1, 4 },
  { 1, 3 },
  { 1, 2 },
  { 1, 1 },
};

int loop_length[] = {
  1,
  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
  4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
  5, 5, 5, 5,
  6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
  7, 7,
  8, 8, 8, 8, 8, 8, 8, 8, 8,
  10, 10, 10,
  12, 12, 12, 12, 12, 12, 12,
  14, 14,
  16
};
GateFlags hidden_gates[kNumParameters];
float parameters[kNumParameters];
float ramp_buffer[kBlockSize * 4];
bool gates[kBlockSize * 2];
bool master_gates[kBlockSize];
float voltages[kBlockSize * 4];
Ramps ramps;
GroupSettings x, y;
bool gate_delay_tail[kNumGateOutputs][kGateDelay];

float SineOscillator(float voltage) {
  static float phase = 0.0f;
  CONSTRAIN(voltage, -5.0f, 5.0f);
  float frequency = stmlib::SemitonesToRatio(voltage * 12.0f) * 220.0f / kSampleRate;
  phase += frequency;
  if (phase >= 1.0f) {
    phase -= 1.0f;
  }
  return 5.0f * Interpolate(lut_sine, phase, 256.0f);
}

void Process(IOBuffer::Block* block, size_t size) {
  if (PROFILE_RENDER) {
    TIC;
  }

  // Filter CV values (3.5%)
  cv_reader.Process(
      settings.explicit_reset(),
      &block->adc_value[0],
      parameters,
      hidden_gates);

  const State& state = settings.state();
  bool grids_mode = (state.t_model == T_GENERATOR_MODEL_GRIDS);

  // In Grids mode the DEJA VU CV jack is repurposed as the reset trigger
  // (see grids_reset below), so its CV must not contaminate the deja_vu /
  // grids_chaos parameter or the UI lock deadband. Use pot-only there.
  float deja_vu = grids_mode
      ? cv_reader.channel(ADC_CHANNEL_DEJA_VU_AMOUNT).pot()
      : parameters[ADC_CHANNEL_DEJA_VU_AMOUNT];
  float deja_vu_raw = deja_vu;
  
  //  Deadband near 12 o'clock for the deja vu parameter.
  const float d = fabsf(deja_vu - 0.5f);
  if (d > 0.03f) {
    ui.set_deja_vu_lock(false);
  } else if (d < 0.02f) {
    ui.set_deja_vu_lock(true);
  }
  if (deja_vu < 0.47f) {
    deja_vu *= 1.06382978723f;
  } else if (deja_vu > 0.53f) {
    deja_vu = 0.5f + (deja_vu - 0.53f) * 1.06382978723f;
  } else {
    deja_vu = 0.5f;
  }
  
  GateFlags* t_clock = block->input[0];
  GateFlags* xy_clock = block->input[1];
  
  // Determine the clock source for the XY section (2%)
  ClockSource xy_clock_source = CLOCK_SOURCE_INTERNAL_T1_T2_T3;
  if (block->input_patched[1]) {
    xy_clock_source = CLOCK_SOURCE_EXTERNAL;
    size_t best_score = 8;
    for (size_t i = 0; i < kNumGateOutputs; ++i) {
      size_t score = self_patching_detector[i].Process(block, size);
      if (score >= best_score) {
        xy_clock_source = ClockSource(CLOCK_SOURCE_INTERNAL_T1 + i);
        best_score = score;
      }
    }
  }

  // Generate gates for T-section (16%).
  ramps.master = &ramp_buffer[0];
  ramps.external = &ramp_buffer[kBlockSize];
  ramps.slave[0] = &ramp_buffer[kBlockSize * 2];
  ramps.slave[1] = &ramp_buffer[kBlockSize * 3];

  // In Grids mode all X outputs follow a single steady clock, never the
  // individual pattern gate outputs.
  if (grids_mode) {
    xy_clock_source = block->input_patched[1]
        ? CLOCK_SOURCE_EXTERNAL
        : CLOCK_SOURCE_INTERNAL_T2;
  }
  int deja_vu_length = deja_vu_length_quantizer.Lookup(
      loop_length,
      parameters[ADC_CHANNEL_DEJA_VU_LENGTH]);

  // TB-3PO seed lifecycle (Grids mode only). x_deja_vu doubles as a "lock"
  // switch: OFF → ON|LOCKED commits the current seed to flash; ON|LOCKED → OFF
  // draws a new seed (auditioning). Edge-triggered to avoid flash thrash.
  if (grids_mode && state.x_deja_vu != prev_x_deja_vu) {
    if (state.x_deja_vu == DEJA_VU_OFF) {
      tb3po.Reseed();
      settings.mutable_state()->tb3po_seed = tb3po.seed();
    } else if (prev_x_deja_vu == DEJA_VU_OFF) {
      settings.mutable_state()->tb3po_seed = tb3po.seed();
      settings.SaveState();
    }
  }
  prev_x_deja_vu = state.x_deja_vu;

  // In Grids mode the DEJA VU CV rising edge resets both T and X sections so
  // drums and bassline restart together. Outside Grids mode the existing
  // T_JITTER-edge / explicit_reset path applies.
  bool grids_reset = grids_mode &&
      (hidden_gates[ADC_CHANNEL_DEJA_VU_AMOUNT] & GATE_FLAG_RISING);

  bool t_section_reset = (settings.explicit_reset() &&
      !grids_mode &&
      (hidden_gates[ADC_CHANNEL_T_JITTER] & GATE_FLAG_RISING)) ||
      grids_reset;
  
  t_generator.set_model(TGeneratorModel(state.t_model));
  t_generator.set_range(TGeneratorRange(state.t_range));
  
  if (state.t_model == T_GENERATOR_MODEL_GRIDS) {
    t_generator.set_rate(cv_reader.channel(ADC_CHANNEL_T_RATE).pot());
    t_generator.set_bias(cv_reader.channel(ADC_CHANNEL_T_BIAS).pot());
    t_generator.set_jitter(cv_reader.channel(ADC_CHANNEL_T_JITTER).pot());

    float bd = cv_reader.channel(ADC_CHANNEL_T_RATE).cv() / 120.0f;
    float sd = cv_reader.channel(ADC_CHANNEL_T_BIAS).cv();
    float hh = cv_reader.channel(ADC_CHANNEL_T_JITTER).cv();
    
    t_generator.set_grids_bd_density(bd);
    t_generator.set_grids_sd_density(sd);
    t_generator.set_grids_hh_density(hh);

    bool euclidean = (state.t_deja_vu != DEJA_VU_OFF);
    t_generator.set_grids_euclidean(euclidean);
    if (euclidean) {
      t_generator.set_grids_euclidean_length(deja_vu_length);
    }

    // DEJA VU knob is bipolar around 12 o'clock in Grids mode:
    //   CCW (< 0.5) → drum-mode chaos and Euclidean-mode T2 fills
    //   CW  (> 0.5) → Euclidean-mode rotation
    //   center      → no effect anywhere
    // tb3po does not read deja_vu_raw, so the X-section is unaffected.
    float ccw = (0.5f - deja_vu_raw) * 2.0f;
    CONSTRAIN(ccw, 0.0f, 1.0f);
    float cw = (deja_vu_raw - 0.5f) * 2.0f;
    CONSTRAIN(cw, 0.0f, 1.0f);
    // Square-root taper on chaos: grids divides perturbation by 4 internally,
    // so a linear ramp feels inert until the last quarter. sqrt(ccw) front-
    // loads the response so small CCW turns already audibly perturb the map.
    t_generator.set_grids_chaos(sqrtf(ccw));
    // Cubic taper + ~1/3 cap on the fill probability: small CCW turns
    // sprinkle hits rather than flood T2, and even fully-CCW stays
    // sub-saturation (~33% per empty step) so the base groove still reads.
    t_generator.set_grids_euclidean_fill(ccw * ccw * ccw * 0.33f);
    t_generator.set_grids_euclidean_rotation(cw);
  } else {
    t_generator.set_rate(parameters[ADC_CHANNEL_T_RATE]);
    t_generator.set_bias(parameters[ADC_CHANNEL_T_BIAS]);
    t_generator.set_jitter(parameters[ADC_CHANNEL_T_JITTER]);
    t_generator.set_pulse_width_mean(float(state.t_pulse_width_mean) / 256.0f);
    t_generator.set_pulse_width_std(float(state.t_pulse_width_std) / 256.0f);
  }
  
  if (!grids_mode) {
    t_generator.set_deja_vu(
        state.t_deja_vu == DEJA_VU_LOCKED
            ? 0.5f
            : (state.t_deja_vu == DEJA_VU_ON ? deja_vu : 0.0f));
    t_generator.set_length(deja_vu_length);
  }

  // TB-3PO per-block parameter feed. Pattern shape (density, scale) goes in
  // here; live transpose/length/lock are also refreshed each block.
  if (grids_mode) {
    int dens_enc = static_cast<int>(
        roundf(parameters[ADC_CHANNEL_X_SPREAD] * 14.0f));
    int dens_cv = static_cast<int>(
        roundf(cv_reader.channel(ADC_CHANNEL_X_SPREAD).cv() * 7.0f));
    tb3po.set_density(dens_enc, dens_cv);

    // BIAS knob: quantized in semitones over a ±18 (3 octave) range.
    // BIAS CV: 1V/oct tracking — X_BIAS uses the default uncalibrated cv
    // scale (-2.0), so cv() runs ~±1 over ±5 V → ×60 yields semitones/V.
    // CV is clamped to ±18 semitones so it spans the same 3-octave range.
    float bias_pot = cv_reader.channel(ADC_CHANNEL_X_BIAS).unscaled_pot();
    int knob_semitones = static_cast<int>(roundf((bias_pot - 0.5f) * 36.0f));
    CONSTRAIN(knob_semitones, -18, 18);
    float cv_semitones = cv_reader.channel(ADC_CHANNEL_X_BIAS).cv() * 60.0f;
    CONSTRAIN(cv_semitones, -18.0f, 18.0f);
    tb3po.set_transpose(static_cast<float>(knob_semitones) + cv_semitones);

    // STEPS knob + CV drives TB-3PO step count, 1..32 (kMaxSteps). The
    // channel's HysteresisFilter (hysteresis=0.02) is wider than one step
    // (1/31 ≈ 0.032), so rounding the combined parameter is stable.
    int tb3po_length = 1 + static_cast<int>(
        roundf(parameters[ADC_CHANNEL_X_STEPS] * 31.0f));
    CONSTRAIN(tb3po_length, 1, TB3PoSequencer::kMaxSteps);
    tb3po.set_length(tb3po_length);

    tb3po.set_lock_seed(state.x_deja_vu != DEJA_VU_OFF);
    tb3po.set_scale(&settings.persistent_data().scale[state.x_scale]);
  }

  t_generator.Process(
      block->input_patched[0],
      &t_section_reset,
      t_clock,
      ramps,
      gates,
      master_gates,
      size);

  // Generate voltages for X-section (40%).
  float note_cv_1 = cv_reader.channel(ADC_CHANNEL_X_SPREAD).scaled_raw_cv();
  float note_cv_2 = cv_reader.channel(ADC_CHANNEL_X_SPREAD_2).scaled_raw_cv();
  float note_cv = 0.5f * (note_cv_1 + note_cv_2);
  float u = note_filter.Process(0.5f * (note_cv + 1.0f));
  
  if (test_adc_noise) {
    static float note_lp = 0.0f;
    float note = note_cv_1;
    ONE_POLE(note_lp, note, 0.0001f);
    float cents = (note - note_lp) * 1200.0f * 5.0f;
    fill(&voltages[0], &voltages[4 * size], cents);
  } else if (ui.recording_scale()) {
    float voltage = (u - 0.5f) * 10.0f;
    for (size_t i = 0; i < size; ++i) {
      GateFlags gate = block->input_patched[1]
          ? block->input[1][i]
          : GATE_FLAG_LOW;
      if (gate & GATE_FLAG_RISING) {
        scale_recorder.NewNote(voltage);
      }
      if (gate & GATE_FLAG_HIGH) {
        scale_recorder.UpdateVoltage(voltage);
      }
      if (gate & GATE_FLAG_FALLING) {
        scale_recorder.AcceptNote();
      }
    }
    fill(&voltages[0], &voltages[4 * size], voltage);
  } else {
    x.control_mode = ControlMode(state.x_control_mode);
    x.voltage_range = VoltageRange(state.x_range % 3);
    x.register_mode = state.x_register_mode;
    x.register_value = u;
    cv_reader.set_attenuverter(
        ADC_CHANNEL_X_SPREAD, state.x_register_mode ? 0.5f : 1.0f);
  
    x.spread = parameters[ADC_CHANNEL_X_SPREAD];
    x.bias = parameters[ADC_CHANNEL_X_BIAS];
    x.steps = parameters[ADC_CHANNEL_X_STEPS];
    x.deja_vu = state.x_deja_vu == DEJA_VU_LOCKED
        ? 0.5f
        : (state.x_deja_vu == DEJA_VU_ON ? deja_vu : 0.0f);
    x.length = deja_vu_length;
    x.ratio.p = 1;
    x.ratio.q = 1;
  
    y.control_mode = CONTROL_MODE_IDENTICAL;
    y.voltage_range = VoltageRange(state.y_range);
    y.register_mode = false;
    y.register_value = 0.0f;
    y.spread = float(state.y_spread) / 256.0f;
    y.bias = float(state.y_bias) / 256.0f;
    y.steps = float(state.y_steps) / 256.0f;
    y.deja_vu = 0.0f;
    y.length = 1;
    y.ratio = y_divider_ratios[
        static_cast<uint16_t>(state.y_divider) * 12 >> 8];
    
    if (settings.dirty_scale_index() != -1) {
      int i = settings.dirty_scale_index();
      xy_generator.LoadScale(i, settings.persistent_data().scale[i]);
      settings.set_dirty_scale_index(-1);
    }
    
    y.scale_index = x.scale_index = state.x_scale;
    
    bool x_section_reset = settings.explicit_reset() && \
        hidden_gates[ADC_CHANNEL_X_STEPS] & GATE_FLAG_RISING;
    if (xy_clock_source != CLOCK_SOURCE_EXTERNAL) {
      x_section_reset |= t_section_reset;
    }

    xy_generator.Process(
        xy_clock_source,
        x,
        y,
        &x_section_reset,
        xy_clock,
        ramps,
        voltages,
        size);
  }
  
  const float* v = voltages;
  const bool* g = gates;
  const bool* mg = master_gates;
  bool tb3po_reset_pending = grids_reset;
  for (size_t i = 0; i < size; ++i) {
    float ramp = ramp_buffer[i];
    if (grids_mode) {
      bool x_ext = (xy_clock_source == CLOCK_SOURCE_EXTERNAL);

      // T-section stall watchdog. Adapts to ~2× the last T-clock period.
      // ForceGateOff is skipped when the X clock jack is driving TB-3PO
      // (x_ext), because T stopping should not latch the bassline gate.
      if (block->input_patched[0]) {
        if (t_clock[i] & GATE_FLAG_RISING) {
          if (t_clock_silence_samples > 0) {
            t_clock_last_period_samples = t_clock_silence_samples;
          }
          t_clock_silence_samples = 0;
        } else {
          uint32_t threshold = t_clock_last_period_samples * 2;
          if (threshold < static_cast<uint32_t>(kSampleRate / 8)) {
            threshold = kSampleRate / 8;
          }
          if (threshold > static_cast<uint32_t>(kSampleRate * 2)) {
            threshold = kSampleRate * 2;
          }
          if (t_clock_silence_samples < threshold) {
            ++t_clock_silence_samples;
            if (t_clock_silence_samples == threshold && !x_ext) {
              tb3po.ForceGateOff();
            }
          }
        }
      } else {
        t_clock_silence_samples = 0;
      }

      if (x_ext) {
        // X clock jack drives TB-3PO: rising edge → new step, falling → gate off.
        if (xy_clock[i] & GATE_FLAG_RISING) {
          if (x_clock_silence_samples > 0) {
            x_clock_last_period_samples = x_clock_silence_samples;
          }
          x_clock_silence_samples = 0;
          tb3po.Tick(tb3po_reset_pending);
          tb3po_reset_pending = false;
        } else {
          if (xy_clock[i] & GATE_FLAG_FALLING) {
            tb3po.TickHalfCycle();
          }
          uint32_t threshold = x_clock_last_period_samples * 2;
          if (threshold < static_cast<uint32_t>(kSampleRate / 8)) {
            threshold = kSampleRate / 8;
          }
          if (threshold > static_cast<uint32_t>(kSampleRate * 2)) {
            threshold = kSampleRate * 2;
          }
          if (x_clock_silence_samples < threshold) {
            ++x_clock_silence_samples;
            if (x_clock_silence_samples == threshold) {
              tb3po.ForceGateOff();
            }
          }
        }
      } else {
        x_clock_silence_samples = 0;
        // ramps.master in Grids mode cycles 0→1 once per 16th note.
        // Downward jump = step boundary (Tick); 0.5 crossing = gate-off (TickHalfCycle).
        bool step_boundary = ramp < prev_grids_ramp - 0.5f;
        bool half_cycle = prev_grids_ramp < 0.5f && ramp >= 0.5f;
        if (step_boundary) {
          tb3po.Tick(tb3po_reset_pending);
          tb3po_reset_pending = false;
        }
        if (half_cycle) {
          tb3po.TickHalfCycle();
        }
      }
      tb3po.StepSlide();
    }
    prev_grids_ramp = ramp;

    // X1 = 5V/0V Grids clock; X2/X3/Y = TB-3PO pitch / gate / accent. The
    // voltages buffer is advanced past all four X slots so xy_generator state
    // remains coherent across mode switches even when its outputs are unused.
    float vx1 = *v++;
    float vx2 = *v++;
    float vx3 = *v++;
    float vy  = *v++;
    float x1 = grids_mode
        ? ((xy_clock_source == CLOCK_SOURCE_EXTERNAL)
           ? (xy_clock[i] & GATE_FLAG_HIGH ? 5.0f : 0.0f)
           : (ramp < 0.5f ? 5.0f : 0.0f))
        : vx1;
    float x2 = grids_mode ? tb3po.pitch_volts()                     : vx2;
    float x3 = grids_mode ? (tb3po.gate()   ? 5.0f : 0.0f)          : vx3;
    float y  = grids_mode ? (tb3po.accent() ? 5.0f : 0.0f)          : vy;

    block->cv_output[1][i] = DacCode(1, x1);
    block->cv_output[2][i] = DacCode(2, x2);
    block->cv_output[3][i] = DacCode(3, x3);
    block->cv_output[0][i] = DacCode(0, y);
    block->gate_output[0][i + kGateDelay] = *g++;
    block->gate_output[1][i + kGateDelay] = *mg++;
    block->gate_output[2][i + kGateDelay] = *g++;
  }
  
  for (size_t i = 0; i < kNumGateOutputs; ++i) {
    for (size_t j = 0; j < kGateDelay; ++j) {
      block->gate_output[i][j] = gate_delay_tail[i][j];
      gate_delay_tail[i][j] = block->gate_output[i][size + j];
    }
  }
  
  if (PROFILE_RENDER) {
    TOC;
  }
}

void Init() {
  System sys;
  sys.Init(true);
  settings.Init();
  
  clock_inputs.Init();
  dac.Init(kSampleRate, 1);
  rng.Init();
  note_filter.Init();
  gate_outputs.Init();
  io_buffer.Init();
    
  deja_vu_length_quantizer.Init(
      sizeof(loop_length) / sizeof(int), 0.25f, false);
  cv_reader.Init(settings.mutable_calibration_data());
  scale_recorder.Init();
  ui.Init(&settings, &cv_reader, &scale_recorder, &clock_inputs, &tb3po);
  
  if (settings.freshly_baked()) {
    settings.ProgramOptionBytes();
    if (PROFILE_INTERRUPT || PROFILE_RENDER) {
      DebugPin::Init();
    } else {
      debug_port.Init();
    }
  }
  
  random_generator.Init(1);
  random_stream.Init(&random_generator);
  t_generator.Init(&random_stream, static_cast<float>(kSampleRate));
  xy_generator.Init(&random_stream, static_cast<float>(kSampleRate));

  for (size_t i = 0; i < kNumScales; ++i) {
    xy_generator.LoadScale(i, settings.persistent_data().scale[i]);
  }

  // Seed the TB-3PO acid sequencer from saved state so a locked pattern
  // survives a power cycle. set_scale() must come before set_seed() because
  // regeneration depends on scale_size.
  tb3po.Init();
  {
    const State& s = settings.state();
    tb3po.set_scale(&settings.persistent_data().scale[s.x_scale]);
    tb3po.set_lock_seed(s.x_deja_vu != DEJA_VU_OFF);
    tb3po.set_seed(s.tb3po_seed);
    prev_x_deja_vu = s.x_deja_vu;
    prev_grids_ramp = 0.0f;
  }
  
  for (size_t i = 0; i < kNumGateOutputs; ++i) {
    self_patching_detector[i].Init(i);
  }
  
  sys.StartTimers();
  dac.Start(&FillBuffer);
}

int main(void) {
  Init();
  while (1) {
    ui.DoEvents();
    io_buffer.Process(ui.output_test_mode() ? &ProcessTest : &Process);
  }
}
