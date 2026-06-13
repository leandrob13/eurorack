#include <math.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <random>

#include "stages/test/fixtures.h"
#include "stages/quantizer.h"
#include "stages/braids_quantizer.h"
#include "stages/quantizer_scales.h"

using namespace std;
using namespace chrono;
using namespace stages;

using timer = high_resolution_clock;

template <typename T>
void use(T&& t) {
  __asm__ __volatile__("" ::"g"(t));
}

template <typename T>
time_t time(T code, size_t iterations) {
  auto start = timer::now();
  while (iterations--) {
    use(code());
  }
  auto end = timer::now();
  return duration_cast<nanoseconds>(end - start).count();
}

template <typename T>
size_t pick_iters(T code) {
  size_t iterations;
  for (iterations = 1; iterations < 1e9; iterations *= 10) {
    time_t t = time(code, iterations);
    if (t > 10e6) {
      return 10 * iterations;
    }
  }
  return iterations;
}

template <typename T>
void timeit(T code, size_t runs) {
  double min = INT64_MAX;
  double mean = 0;
  size_t iterations = pick_iters(code);
  for (size_t i = 0; i < runs; i++) {
    time_t t = time(code, iterations);
    mean += t;
    if (t < min) {
      min = t;
    }
  }
  min /= iterations;
  mean = mean / iterations / runs;

  if (min < 1e3) {
    printf("%ld runs of %ld iterations; mean %0.3fns, min %0.3fns\n", runs,
           iterations, mean, min);
  } else if (min < 1e6) {
    printf("%ld runs of %ld iterations; mean %0.3fus, min %0.3fus\n", runs,
           iterations, mean / 1e3, min / 1e3);
  } else if (min < 1e9) {
    printf("%ld runs of %ld iterations; mean %0.3fms, min %0.3fms\n", runs,
           iterations, mean / 1e6, min / 1e6);
  } else {
    printf("%ld runs of %ld iterations; mean %0.3fs, min %0.3fs\n", runs,
           iterations, mean / 1e9, min / 1e9);
  }
}

void TimeTapLFO() {
  cout << "Tap LFO" << endl;
  timeit(
      [] {
        SegmentGeneratorTest t;
        segment::Configuration configuration = {segment::TYPE_RAMP, true, false,
                                                segment::RANGE_DEFAULT};
        t.generator()->Configure(true, &configuration, 1);
        for (int i = 0; i < 1000; ++i) {
          t.pulses()->AddPulses(1500, 500, 6);
          t.pulses()->AddPulses(3000, 500, 2);
        }

        t.set_segment_parameters(0, 0.5f, 0.5f);
        const size_t size = 8;
        while (!t.pulses()->empty()) {
          GateFlags flags[size];
          t.pulses()->Render(flags, 8);
          SegmentGenerator::Output out[size];

          t.generator()->Process(flags, out, size);
        }
        return 0;
      },
      7);
}

void TimeRandomBrownianTapLFO() {
  cout << "Random Brownian Tap LFO" << endl;
  timeit(
      [] {
        SegmentGeneratorTest t;
        segment::Configuration configuration = {segment::TYPE_TURING, true};
        t.generator()->Configure(true, &configuration, 1);
        for (int i = 0; i < 1000; ++i) {
          t.pulses()->AddPulses(1500, 500, 6);
          t.pulses()->AddPulses(3000, 500, 2);
        }

        t.set_segment_parameters(0, 0.5f, 0.75f);
        const size_t size = 8;
        while (!t.pulses()->empty()) {
          GateFlags flags[size];
          t.pulses()->Render(flags, 8);
          SegmentGenerator::Output out[size];

          t.generator()->Process(flags, out, size);
        }
        return 0;
      },
      7);
}

void TimeRandomSplineTapLFO() {
  cout << "Random Spline Tap LFO" << endl;
  timeit(
      [] {
        SegmentGeneratorTest t;
        segment::Configuration configuration = {segment::TYPE_TURING, true};
        t.generator()->Configure(true, &configuration, 1);
        for (int i = 0; i < 1000; ++i) {
          t.pulses()->AddPulses(1500, 500, 6);
          t.pulses()->AddPulses(3000, 500, 2);
        }

        t.set_segment_parameters(0, 0.5f, 0.5f);
        const size_t size = 8;
        while (!t.pulses()->empty()) {
          GateFlags flags[size];
          t.pulses()->Render(flags, 8);
          SegmentGenerator::Output out[size];

          t.generator()->Process(flags, out, size);
        }
        return 0;
      },
      7);
}

void TimeRandomSineTapLFO() {
  cout << "Random Sine Tap LFO" << endl;
  timeit(
      [] {
        SegmentGeneratorTest t;
        segment::Configuration configuration = {segment::TYPE_TURING, true};
        t.generator()->Configure(true, &configuration, 1);
        for (int i = 0; i < 1000; ++i) {
          t.pulses()->AddPulses(1500, 500, 6);
          t.pulses()->AddPulses(3000, 500, 2);
        }

        t.set_segment_parameters(0, 0.5f, 0.5f);
        const size_t size = 8;
        while (!t.pulses()->empty()) {
          GateFlags flags[size];
          t.pulses()->Render(flags, 8);
          SegmentGenerator::Output out[size];

          t.generator()->Process(flags, out, size);
        }
        return 0;
      },
      7);
}

void TimeSmallQuantizer() {
  cout << "Small Quantizer" << endl;
  Quantizer quant;
  quant.Init();
  quant.Configure(scales[1]);
  timeit(
    [&quant] {
      return quant.Process(2.0f * static_cast<float>(rand()) / RAND_MAX - 1.0f);
    },
    7
  );
}


void TimeQuantizer() {
  cout << "Quantizer" << endl;
  BraidsQuantizer quant;
  quant.Init();
  quant.Configure(scales[1]);
  timeit(
    [&quant] {
      return quant.Process(2.0f * static_cast<float>(rand()) / RAND_MAX - 1.0f);
    },
    7
  );
}

int main() {
  TimeTapLFO();
  TimeRandomBrownianTapLFO();
  TimeRandomSineTapLFO();
  TimeRandomSplineTapLFO();
}