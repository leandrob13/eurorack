// Copyright 2021 Emilie Gillet.
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
// Chords: wavetable and divide-down organ/string machine.

#include "plaits/dsp/chords/chord_bank.h"
#include "plaits/dsp/chords/chords.h"

#include "stmlib/dsp/units.h"

namespace plaits {

using namespace stmlib;

void ChordBank::Init(BufferAllocator* allocator) {
  ratios_ = allocator->Allocate<float>(kChordNumChords * kChordNumNotes);
  note_count_ = allocator->Allocate<int>(kChordNumChords);
  sorted_ratios_ = allocator->Allocate<float>(kChordNumNotes);

  chord_index_quantizer_.Init(kChordNumChords, 0.0125f, false);
  genre_quantizer_.Init(16, 0.05f, false);
  genre_ = 0;
  chord_ = 0.0f;
}

void ChordBank::Reset() {
  for (int i = 0; i < kChordNumChords; ++i) {
    int count = 0;
    for (int j = 0; j < kChordNumNotes; ++j) {
      float octave_down = genre_ < 3 ? 24.0f : 36.0f;
      ratios_[i * kChordNumNotes + j] = SemitonesToRatio(genre_chords[genre_][i][j] - octave_down);
      if (genre_chords[genre_][i][j] !=  0.01f && genre_chords[genre_][i][j] !=  7.01f && \
          genre_chords[genre_][i][j] != 11.99f && genre_chords[genre_][i][j] != 12.00f) {
        ++count;
      }
    }
    note_count_[i] = count;
  }
  Sort();
}

int ChordBank::ComputeChordInversion(
    float inversion,
    float* ratios,
    float* amplitudes) {
  const float* base_ratio = this->ratios();
  inversion = inversion * float(kChordNumNotes * kChordNumVoices);

  MAKE_INTEGRAL_FRACTIONAL(inversion);
  
  int num_rotations = inversion_integral / kChordNumNotes;
  int rotated_note = inversion_integral % kChordNumNotes;
  
  const float kBaseGain = 0.25f;
  
  int mask = 0;
  
  for (int i = 0; i < kChordNumNotes; ++i) {
    float transposition = 0.25f * static_cast<float>(
        1 << ((kChordNumNotes - 1 + inversion_integral - i) / kChordNumNotes));
    int target_voice = (i - num_rotations + kChordNumVoices) % kChordNumVoices;
    int previous_voice = (target_voice - 1 + kChordNumVoices) % kChordNumVoices;
    
    if (i == rotated_note) {
      ratios[target_voice] = base_ratio[i] * transposition;
      ratios[previous_voice] = ratios[target_voice] * 2.0f;
      amplitudes[previous_voice] = kBaseGain * inversion_fractional;
      amplitudes[target_voice] = kBaseGain * (1.0f - inversion_fractional);
    } else if (i < rotated_note) {
      ratios[previous_voice] = base_ratio[i] * transposition;
      amplitudes[previous_voice] = kBaseGain;
    } else {
      ratios[target_voice] = base_ratio[i] * transposition;
      amplitudes[target_voice] = kBaseGain;
    }
    
    if (i == 0) {
      if (i >= rotated_note) {
        mask |= 1 << target_voice;
      }
      if (i <= rotated_note) {
        mask |= 1 << previous_voice;
      }
    }
  }
  return mask;
}

}  // namespace plaits
