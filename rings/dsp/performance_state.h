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
// Note triggering state.

#ifndef RINGS_DSP_PERFORMANCE_STATE_H_
#define RINGS_DSP_PERFORMANCE_STATE_H_

namespace rings {

const int32_t kNumChords = 11;
const int32_t kNumGenres = 25;
const int32_t kNumArps = 9;

struct PerformanceState {
  bool strum;
  bool internal_exciter;
  bool internal_strum;
  bool internal_note;

  float tonic;
  float note;
  float fm;
  int32_t chord;

  int16_t genre;
  int16_t arp;
  float envelope; // Damping pot
  float vca_level; // Damping attenuator
  float vca_cv; // Damping CV
  float filter_frequency; // Position pot
  float filter_amount; // Position attenueverter
  float filter_cv; // Position CV
};

}  // namespace rings

#endif  // RINGS_DSP_PERFORMANCE_STATE_H_
