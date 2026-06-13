// Copyright 2015 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
// Re-implemented by Bryan Head to eliminate codebook
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
// Note quantizer
#ifndef STAGES_QUANTIZER_H_
#define STAGES_QUANTIZER_H_

#include "stmlib/stmlib.h"

namespace stages {

struct Scale {
  int16_t span;
  //uint8_t num_notes;
  size_t num_notes;
  int16_t notes[16];
};

const float eight_octaves = static_cast<float>((12 << 7) * 8);

class Quantizer {
 public:
  Quantizer() {}
  ~Quantizer() {}

  void Init();

  // Assumes 0.0 = 0v and 1.0f = 8v
  float Process(float pitch) {
    return Process(static_cast<int16_t>(pitch * eight_octaves)) / eight_octaves;
  }

  int16_t Process(int16_t pitch) { return Process(pitch, 0); }

  int16_t Process(int16_t pitch, int16_t root);

  void Configure(const Scale& scale) {
    notes_ = scale.notes;
    span_ = scale.span;
    num_notes_ = scale.num_notes;
    enabled_ = notes_ != NULL && num_notes_ != 0 && span_ != 0;
  }

 private:
  bool enabled_;
  int16_t codeword_;
  int16_t previous_boundary_;
  int16_t next_boundary_;
  const int16_t* notes_;
  int16_t span_;
  uint8_t num_notes_;

  DISALLOW_COPY_AND_ASSIGN(Quantizer);
};

}  // namespace stages

#endif