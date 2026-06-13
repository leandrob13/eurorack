// Copyright 2017 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "stages/test/fixtures.h"

#include "stages/braids_quantizer.h"
#include "stages/quantizer.h"
#include "stages/quantizer_scales.h"

using namespace stages;
using namespace stmlib;

const uint32_t kSampleRate = 32000;

void TestADSR() {
  SegmentGeneratorTest t;

  segment::Configuration configuration[6] = {
    { segment::TYPE_RAMP, false },
    { segment::TYPE_RAMP, false },
    { segment::TYPE_RAMP, false },
    { segment::TYPE_HOLD, true },
    { segment::TYPE_RAMP, false },
  };

  t.generator()->Configure(true, configuration, 5);
  t.set_segment_parameters(0, 0.15f, 0.0f);
  t.set_segment_parameters(1, 0.25f, 0.3f);
  t.set_segment_parameters(2, 0.25f, 0.75f);
  t.set_segment_parameters(3, 0.5f, 0.1f);
  t.set_segment_parameters(4, 0.5f, 0.25f);
  t.Render("stages_adsr.wav", ::kSampleRate);
}

void TestTwoStepSequence() {
  SegmentGeneratorTest t;

  segment::Configuration configuration[2] = {
    { segment::TYPE_HOLD, false },
    { segment::TYPE_HOLD, false },
  };

  t.generator()->Configure(true, configuration, 2);
  t.set_segment_parameters(0, 0.2f, 0.3f);
  t.set_segment_parameters(1, -1.0f, 0.5f);
  t.Render("stages_two_step.wav", ::kSampleRate);
}

void TestSingleDecay() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_RAMP, false };

  t.generator()->Configure(true, &configuration, 1);
  t.set_segment_parameters(0, 0.7f, 0.2f);
  t.Render("stages_single_decay.wav", ::kSampleRate);
}

void TestTimedPulse() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_HOLD, false };

  t.generator()->Configure(true, &configuration, 1);
  t.set_segment_parameters(0, -1.0f, 0.4f);
  t.Render("stages_timed_pulse.wav", ::kSampleRate);
}

void TestGate() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_HOLD, true };

  t.generator()->Configure(true, &configuration, 1);
  t.set_segment_parameters(0, 0.5f, 1.0f);
  t.Render("stages_gate.wav", ::kSampleRate);
}

void TestSampleAndHold() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_STEP, false };

  t.generator()->Configure(true, &configuration, 1);
  t.set_segment_parameters(0, -1.0f, 0.5f);
  t.Render("stages_sh.wav", ::kSampleRate);
}

void TestPortamento() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_STEP, false };

  t.generator()->Configure(false, &configuration, 1);
  t.set_segment_parameters(0, -1.0f, 0.7f);
  t.Render("stages_portamento.wav", ::kSampleRate);
}

void TestFreeRunningLFO() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_RAMP, true };

  t.generator()->Configure(false, &configuration, 1);
  t.set_segment_parameters(0, 0.7f, -3.0f);
  t.Render("stages_free_running_lfo.wav", ::kSampleRate);
}

void TestTapLFOAudioRate() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_RAMP, true };

  t.generator()->Configure(true, &configuration, 1);
  t.pulses()->AddFreq(100000, 1001.0f, 0.5f, ::kSampleRate);
  /*
  t.pulses()->AddFreq(20, 90.0f, 0.5f, ::kSampleRate);
  t.pulses()->AddFreq(20, 110.0f, 0.5f, ::kSampleRate);
  t.pulses()->AddFreq(20, 123.0f, 0.5f, ::kSampleRate);
  t.pulses()->AddFreq(20, 1234.0f, 0.5f, ::kSampleRate);
  t.pulses()->AddFreq(20, 5000.0f, 0.5f, ::kSampleRate);
  t.pulses()->AddFreq(20, 7000.0f, 0.5f, ::kSampleRate);
  t.pulses()->AddFreq(20, 6000.0f, 0.5f, ::kSampleRate);
  */
  /*
  t.pulses()->AddPulses(100, 50, 10);
  t.pulses()->AddPulses(50, 5, 20);
  t.pulses()->AddPulses(30, 5, 20);
  t.pulses()->AddPulses(20, 5, 20);
  for (int i = 0; i < 20; ++i) {
    t.pulses()->AddPulses(30, 5, 1);
    t.pulses()->AddPulses(10, 5, 1);
  }
  t.pulses()->AddPulses(100, 5, 10);
  t.pulses()->AddPulses(50, 5, 20);
  */
  t.set_segment_parameters(0, 0.5f, 0.5f);
  t.Render("stages_tap_lfo_audio_rate.wav", ::kSampleRate);
}

void TestTapLFO() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_RAMP, true };

  t.generator()->Configure(true, &configuration, 1);
  t.pulses()->AddPulses(4000, 1000, 20);
  t.pulses()->AddPulses(8000, 7000, 20);
  for (int i = 0; i < 15; ++i) {
    t.pulses()->AddPulses(1500, 500, 6);
    t.pulses()->AddPulses(3000, 500, 2);
  }
  for (int i = 0; i < 100; ++i) {
    int length = (rand() % 1200) + 400;
    t.pulses()->AddPulses(length, length / 4, 1);
  }
  t.pulses()->AddPulses(10, 5, 500);
  t.set_segment_parameters(0, 0.5f, 0.5f);
  t.Render("stages_tap_lfo.wav", ::kSampleRate);
}

void TestRandomSteppedLFO() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_TURING, true };

  t.generator()->Configure(false, &configuration, 1);
  t.set_segment_parameters(0, 0.7f, 0.0f);
  fprintf(stderr, "Rendering stepped LFO\n");
  Random::Seed(0);
  t.Render("stages_random_stepped_lfo.wav", ::kSampleRate);
}

void TestRandomSineLFO() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_TURING, true };

  t.generator()->Configure(false, &configuration, 1);
  t.set_segment_parameters(0, 0.7f, 0.25f);
  Random::Seed(0);
  t.Render("stages_random_sine_lfo.wav", ::kSampleRate);
}

void TestRandomSplineLFO() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_TURING, true };

  t.generator()->Configure(false, &configuration, 1);
  t.set_segment_parameters(0, 0.7f, 0.5f);
  Random::Seed(0);
  t.Render("stages_random_spline_lfo.wav", ::kSampleRate);
}

void TestRandomBrownianLFO() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_TURING, true };

  t.generator()->Configure(false, &configuration, 1);
  t.set_segment_parameters(0, 0.7f, 0.75f);
  Random::Seed(0);
  t.Render("stages_random_brownian_lfo.wav", ::kSampleRate);
}

void TestRandomTapLFO() {
  srand(0);
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_TURING, true };

  t.generator()->Configure(true, &configuration, 1);
  for (int i = 0; i < 1000; ++i) {
    int length = (rand() % 1200) + 400;
    t.pulses()->AddPulses(length, 100, 1);
  }
  t.set_segment_parameters(0, 0.5f, 0.0f);
  t.Render("stages_random_tap_lfo.wav", ::kSampleRate);
}

void TestWhiteNoise() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_TURING, true, false, segment::RANGE_FAST };

  t.generator()->Configure(false, &configuration, 1);
  t.set_segment_parameters(0, 1.0f, 0.0f);
  Random::Seed(0);
  t.Render("stages_random_white_noise.wav", ::kSampleRate);
}

void TestBrownNoise() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_TURING, true, false, segment::RANGE_FAST };

  t.generator()->Configure(false, &configuration, 1);
  t.set_segment_parameters(0, 1.0f, 1.0f);
  Random::Seed(0);
  t.Render("stages_random_brown_noise.wav", ::kSampleRate);
}

void TestDelay() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_HOLD, false };

  t.generator()->Configure(false, &configuration, 1);
  t.set_segment_parameters(0, -1.0f, 0.5f);
  t.Render("stages_delay.wav", ::kSampleRate);
}

void TestClockedSampleAndHold() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_HOLD, true };

  t.generator()->Configure(false, &configuration, 1);
  t.set_segment_parameters(0, -1.0f, 0.5f);
  t.Render("stages_clocked_sh.wav", ::kSampleRate);
}

void TestZero() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_RAMP, false };

  t.generator()->Configure(false, &configuration, 1);
  t.set_segment_parameters(0, -1.0f, 0.05f);
  t.Render("stages_zero.wav", ::kSampleRate);
}


void TestDelayLine() {
  DelayLine16Bits<8> d;
  d.Init();
  for (int i = 0; i < 21; i++) {
    d.Write(i / 22.0f + 0.01f);
    float a = d.Read(size_t(1));
    float b = d.Read(size_t(2));
    float c = d.Read(1.2f);
    printf("%f %f %f %f\n", a, b, c, a + (b - a) * 0.2f);
  }
}

float rand_float() {
  return static_cast<float>(rand()) / RAND_MAX;
}

void TestSmallQuantizer() {
  printf("Testing quantizer\n");
  BraidsQuantizer ref;

  Quantizer quant;

  srand(0);
  int16_t prev_pitch = 0;
  int passed = 0;
  for (size_t ix = 0; ix < 6; ix++) {
    ref.Init();
    ref.Configure(scales[ix]);
    quant.Init();
    quant.Configure(scales[ix]);
    int16_t min = ref.Process(int32_t(-stages::eight_octaves));
    int16_t max = ref.Process(int32_t(stages::eight_octaves));
    for (int16_t step_size = -12 << 8; step_size < 12 << 8; step_size++) {
      if (step_size == 0) continue;
      for (int16_t i = step_size < 0 ? max : min; i <= max && i >= min; i+=step_size) {
        int16_t q = quant.Process(i);
        int16_t r = ref.Process(i);
        prev_pitch = q;
        if (q != r) {
          printf("Quant %d: Expected %d -> %d but got %d; prev pitch = %d; "
                 "scale = %lu\n",
                 i, i, r, q, prev_pitch, ix);
          return;
        }
        passed++;
      }
    }
    /*
    float min = ref.Process(-1.0f);
    float max = ref.Process(1.0f);

    for (float delta_div = 1.0f; delta_div < 100000.0f; delta_div*=10.0f) {
      for (size_t i = 0; i < 1e6; i++) {
        pitch += (rand_float() - 0.5f) / delta_div;
        CONSTRAIN(pitch, min, max);
        float q = quant.Process(pitch);
        float r = ref.Process(pitch);
        if (q != r) {
          printf("Quant %lu: Expected %f -> %f but got %f; prev pitch = %f; scale = %lu\n",
                i, pitch, r, q, prev_pitch, ix);
          return;
        }
        prev_pitch = q;
      }
    }
    */
  }
  printf("Passed %d quantization tests.\n", passed);
}

void TestTuringMachine() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_TURING, false };

  t.generator()->Configure(true, &configuration, 1);
  t.pulses()->AddPulses(8, 4, ::kSampleRate * 20 * 5);
  t.set_segment_parameters(0, 0.5f, 1.0f);
  t.Render("stages_tm_50.wav", ::kSampleRate);
  t.set_segment_parameters(0, 0.25f, 1.0f);
  t.Render("stages_tm_25.wav", ::kSampleRate);
  t.set_segment_parameters(0, 0.05f, 1.0f);
  t.Render("stages_tm_05.wav", ::kSampleRate);
  t.set_segment_parameters(0, 0.0f, 1.0f);
  t.Render("stages_tm_00.wav", ::kSampleRate);
  t.set_segment_parameters(0, 0.5f, 1.0f);
  configuration.quant_scale = 3;
  t.generator()->Configure(true, &configuration, 1);
  t.Render("stages_tm_50_quantized.wav", ::kSampleRate);
}

void TestQuantizeLinear() {
  SegmentGeneratorTest t;

  for (float i=0; i < 101; i++) {
    float x = float(i) / 50.0f - 1.0f;
    //printf("%f -> %f\n", x, 8.0f * t.generator()->QuantizeLinear(0, scales[3], x, 2));
  }
  for (float i=100; i >= 0; i--) {
    float x = float(i) / 50.0f - 1.0f;
    //printf("%f -> %f\n", x, 8.0f * t.generator()->QuantizeLinear(0, scales[3], x, 2));
  }
}

/*
void TestAudioOscillator() {
  SegmentGeneratorTest t;

  segment::Configuration configuration = { segment::TYPE_ALT, true };

  t.generator()->Configure(false, &configuration, 1);
  t.set_segment_parameters(0, 0.5f, 0.7f);
  t.Render("stages_audio_oscillator.wav", ::kSampleRate);
}
*/

int main(void) {
  /*
  TestADSR();
  TestTwoStepSequence();
  TestSingleDecay();
  TestTimedPulse();
  TestGate();
  TestSampleAndHold();
  TestPortamento();
  TestFreeRunningLFO();
  TestTapLFO();
  TestTapLFOAudioRate();
  TestRandomSteppedLFO();
  TestRandomSineLFO();
  TestRandomSplineLFO();
  TestRandomBrownianLFO();
  TestRandomTapLFO();
  TestWhiteNoise();
  TestBrownNoise();
  TestDelay();
  */
  TestSmallQuantizer();
  //TestTuringMachine();

  //TestQuantizeLinear();
  //TestZero();
  // This segment type doesn't exist anymore
  //TestClockedSampleAndHold();
  //TestAudioOscillator();
}
