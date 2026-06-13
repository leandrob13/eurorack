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
// Quantizer scales

#ifndef STAGES_QUANTIZER_SCALES_H_
#define STAGES_QUANTIZER_SCALES_H_

#include "stages/quantizer.h"

namespace stages {

const Scale scales[] = {
  // Off
  { 0, 0, { } },
  // Semitones
  { 12 << 7, 12, { 0, 128, 256, 384, 512, 640, 768, 896, 1024, 1152, 1280, 1408} },
  // Ionian (From midipal/BitT source code)
  { 12 << 7, 7, { 0, 256, 512, 640, 896, 1152, 1408} },
  // Pentatonic major (From midipal/BitT source code)
  { 12 << 7, 5, { 0, 256, 512, 896, 1152} },

  // Just here for testing
  { 12 << 7, 2, { 128, 256 } },
  { 12 << 7, 2, { 1152, 1408 }}
};

}  // namespace stages

#endif  // STAGES_QUANTIZER_SCALES_H_
