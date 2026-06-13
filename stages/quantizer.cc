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

#include "stages/quantizer.h"

#include <cstdlib>

namespace stages {

void Quantizer::Init() {
  enabled_ = true;
  codeword_ = 0;
  previous_boundary_ = 0;
  next_boundary_ = 0;
}

int16_t Quantizer::Process(int16_t pitch, int16_t root) {
  if (!enabled_) {
    return pitch;
  }
  pitch -= root;

  if (pitch >= previous_boundary_ && pitch <= next_boundary_) {
    pitch = codeword_;
  } else {
    int16_t octave = pitch / span_ - (pitch < 0 ? 1 : 0);
    int16_t rel_pitch = pitch - span_ * octave;

    int16_t best_distance = 16384;
    int16_t q = -1;
    for (size_t i = 0; i < num_notes_; i++) {
      int16_t distance = abs(rel_pitch - notes_[i]);
      if (distance < best_distance) {
        best_distance = distance;
        q = i;
      }
    }
    if (abs(pitch - (octave + 1) * span_ - notes_[0]) < best_distance) {
      octave++;
      q = 0;
    } else if (abs(pitch - (octave - 1) * span_ - notes_[num_notes_ - 1]) <= best_distance) {
      octave--;
      q = num_notes_ - 1;
    }
    codeword_ = notes_[q] + octave * span_;
    previous_boundary_ = q == 0
      ? notes_[num_notes_ - 1] + (octave - 1) * span_
      : notes_[q - 1] + octave * span_;
    previous_boundary_ = (9 * previous_boundary_ + 7 * codeword_) >> 4;
    next_boundary_ = q == num_notes_ - 1
      ? notes_[0] + (octave + 1) * span_
      : notes_[q + 1] + octave * span_;
    next_boundary_ = (9 * next_boundary_ + 7 * codeword_) >> 4;
    pitch = codeword_;
  }
  pitch += root;
  return pitch;
}
}  // namespace stages