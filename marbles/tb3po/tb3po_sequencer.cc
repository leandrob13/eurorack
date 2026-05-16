// Copyright 2025 Marbles Grids/TB-3PO port.

#include "marbles/tb3po/tb3po_sequencer.h"

#include <algorithm>
#include <cstdlib>

#include "stmlib/stmlib.h"

#include "marbles/grids/grids_random.h"
#include "marbles/random/quantizer.h"

namespace marbles {

namespace {

// 303-ish portamento time constant at 32 kHz (~25 ms). Tune on hardware.
const float kSlideCoef = 0.003f;

// Octave offset used as the pitch baseline (in scale degrees = num_degrees *
// kOctaveOffset). 4 octaves lands the root around 4 V on a 1V/oct scale.
const int kOctaveOffset = 4;

// Probability that an octave jump fires when a new pitch is drawn. The
// original TB-3PO uses random(200) < 80 — 40% chance, half up / half down.
const int kOctaveJumpProb = 80;
const int kOctaveJumpRange = 200;

// Wrap the LFSR's MSBs into a [0, range) uniform-ish int. Cheap modulo —
// the LFSR has period 65535, the bias is negligible for our parameter ranges.
inline int RandRange(int range) {
  if (range <= 0) return 0;
  return static_cast<int>(GridsRandom::GetWord()) % range;
}

}  // namespace

void TB3PoSequencer::Init() {
  seed_ = 0;
  lock_seed_ = false;
  num_steps_ = 16;
  current_pattern_density_ = 0xff;
  current_pattern_scale_size_ = 0;

  gates_ = 0;
  slides_ = 0;
  accents_ = 0;
  oct_ups_ = 0;
  oct_downs_ = 0;
  std::fill(&notes_[0], &notes_[kMaxSteps], 0);

  density_encoder_ = 7;
  density_cv_ = 0;
  density_ = 7;
  transpose_ = 0.0f;
  scale_ = NULL;
  scale_size_ = 0;
  std::fill(&active_idx_[0], &active_idx_[kMaxScaleDegrees], 0);
  active_count_ = 0;

  step_ = 0;
  gate_ = false;
  accent_ = false;
  gate_off_pending_ = false;

  pitch_volts_ = 0.0f;
  slide_target_ = 0.0f;
  slide_start_ = 0.0f;
}

void TB3PoSequencer::set_density(int encoder, int cv) {
  CONSTRAIN(encoder, 0, 14);
  CONSTRAIN(cv, -7, 7);
  density_encoder_ = encoder;
  density_cv_ = cv;
  int d = encoder + cv;
  CONSTRAIN(d, 0, 14);
  density_ = d;
}

void TB3PoSequencer::set_transpose(float scale_degrees) {
  transpose_ = scale_degrees;
}

void TB3PoSequencer::set_length(int steps) {
  CONSTRAIN(steps, 1, kMaxSteps);
  num_steps_ = static_cast<uint8_t>(steps);
}

void TB3PoSequencer::set_lock_seed(bool locked) {
  lock_seed_ = locked;
}

void TB3PoSequencer::set_scale(const Scale* scale) {
  bool changed = (scale != scale_);
  scale_ = scale;
  if (scale_) {
    int n = scale_->num_degrees;
    if (n <= 0) n = 1;
    if (n > kMaxScaleDegrees) n = kMaxScaleDegrees;
    scale_size_ = static_cast<uint8_t>(n);
  } else {
    scale_size_ = 12;
  }
  BuildActiveDegrees();
  if (changed) {
    // Force a regeneration on the next Tick so the new in-scale set is
    // reflected immediately, even if density and active_count_ happen to
    // collide with the values that produced the current pattern.
    current_pattern_density_ = 0xff;
  }
}

void TB3PoSequencer::BuildActiveDegrees() {
  active_count_ = 0;
  if (!scale_ || scale_size_ == 0) return;

  // For 12-degree weighted presets (the Marbles defaults: C major, Pentatonic,
  // raags, etc.) low-weight cells are chromatic passing tones used by the
  // weight-aware X-section quantizer. TB-3PO doesn't quantize — it walks
  // cells directly — so filter those out using a relative threshold so we
  // keep only the diatonic/in-scale degrees.
  //
  // For smaller scales (Pelog, user-recorded scales), every degree IS the
  // scale; weight just shapes the X-section's selection probability, so we
  // pass them through verbatim.
  uint8_t threshold = 0;
  if (scale_size_ >= 12) {
    uint8_t max_w = 0;
    for (int i = 0; i < scale_size_; ++i) {
      if (scale_->degree[i].weight > max_w) {
        max_w = scale_->degree[i].weight;
      }
    }
    // ~25% of peak. Empirically catches diatonic notes (weight ≥ 64) on the
    // stock 12-degree presets and rejects the 4/8/16/32-weight chromatic
    // passing tones.
    threshold = static_cast<uint8_t>(max_w >> 2);
    if (threshold == 0) threshold = 1;
  }

  for (int i = 0; i < scale_size_; ++i) {
    if (scale_->degree[i].weight >= threshold) {
      active_idx_[active_count_++] = static_cast<uint8_t>(i);
    }
  }
  if (active_count_ == 0) {
    // Pathological fallback (all weights zero): always allow the root.
    active_idx_[active_count_++] = 0;
  }
}

void TB3PoSequencer::set_seed(uint16_t s) {
  seed_ = s;
  // Force regenerate using the new seed.
  current_pattern_density_ = 0xff;
  current_pattern_scale_size_ = 0;
  RegenerateAll();
}

void TB3PoSequencer::Reseed() {
  // Draw a fresh 16-bit seed. Use one extra Update to avoid handing back the
  // current LFSR state verbatim.
  GridsRandom::Update();
  seed_ = GridsRandom::state();
  if (seed_ == 0) seed_ = 1;
  current_pattern_density_ = 0xff;
  current_pattern_scale_size_ = 0;
  RegenerateAll();
}

void TB3PoSequencer::Tick(bool reset) {
  RegenerateIfDirty();

  uint8_t prev_step;
  if (reset) {
    // Reset folds into the first clock here (the original applet kept them
    // separate). Force prev_step to 0 too so a stale slide bit from the last
    // step before reset doesn't carry into step 0.
    step_ = 0;
    prev_step = 0;
  } else {
    prev_step = step_;
    step_ = GetNextStep(step_);
  }

  if (StepIsSlid(prev_step)) {
    // Glide from wherever the IIR happens to be — matches 303 portamento.
    slide_start_ = pitch_volts_;
    slide_target_ = PitchForStep(step_);
  } else if (StepIsGated(step_)) {
    // hold_pitch=true in v1: only jump pitch on gated steps.
    float p = PitchForStep(step_);
    pitch_volts_ = p;
    slide_start_ = p;
    slide_target_ = p;
  }

  if (StepIsGated(step_) || StepIsSlid(prev_step)) {
    gate_ = true;
    accent_ = StepIsAccent(step_);
    gate_off_pending_ = true;
  }
}

void TB3PoSequencer::TickHalfCycle() {
  if (gate_off_pending_) {
    gate_off_pending_ = false;
    if (!StepIsSlid(step_)) {
      gate_ = false;
      accent_ = false;
    }
  }
}

void TB3PoSequencer::StepSlide() {
  if (pitch_volts_ == slide_target_) return;
  pitch_volts_ += kSlideCoef * (slide_target_ - pitch_volts_);
  if (slide_start_ < slide_target_) {
    if (pitch_volts_ > slide_target_) pitch_volts_ = slide_target_;
  } else {
    if (pitch_volts_ < slide_target_) pitch_volts_ = slide_target_;
  }
}

void TB3PoSequencer::RegenerateIfDirty() {
  // current_pattern_scale_size_ tracks the active (in-scale) count, not the
  // full degree count, since active_count_ is what controls the random walk
  // and the pitch lookup table.
  if (density_ != current_pattern_density_ ||
      active_count_ != current_pattern_scale_size_) {
    RegenerateAll();
  }
}

void TB3PoSequencer::RegenerateAll() {
  // Save shared LFSR state so we don't disturb PatternGenerator's perturbation.
  uint16_t saved = GridsRandom::state();
  // A Galois LFSR seeded with 0 stays at 0 forever — substitute a non-zero
  // constant. Happens on first boot before any commit (state.tb3po_seed == 0).
  GridsRandom::Seed(seed_ == 0 ? 0xACE1 : seed_);

  RegeneratePitches();
  ApplyDensity();

  current_pattern_density_ = density_;
  current_pattern_scale_size_ = active_count_;

  GridsRandom::Seed(saved);
}

void TB3PoSequencer::RegeneratePitches() {
  int pitch_change_dens = GetPitchChangeDensity();
  int available_pitches = 0;
  if (active_count_ > 0) {
    if (pitch_change_dens > 7) {
      available_pitches = active_count_ - 1;
    } else if (pitch_change_dens < 2) {
      available_pitches = pitch_change_dens;
    } else {
      int range_from_scale = active_count_ - 3;
      if (range_from_scale < 4) range_from_scale = 4;
      available_pitches = 3 + (pitch_change_dens - 3) * range_from_scale / 4;
      CONSTRAIN(available_pitches, 1, active_count_ - 1);
    }
    // Final safety: tiny scales (e.g. active_count_==1) may have made the
    // formulas above produce out-of-range indices.
    CONSTRAIN(available_pitches, 0, active_count_ - 1);
  }

  oct_ups_ = 0;
  oct_downs_ = 0;

  for (int s = 0; s < kMaxSteps; ++s) {
    int force_repeat_note_prob = 50 - (pitch_change_dens * 6);
    if (s > 0 && RandBit(force_repeat_note_prob)) {
      notes_[s] = notes_[s - 1];
    } else {
      // Store the *rank* into active_idx_[], not the raw degree, so transpose
      // and octave shifts later operate in active-degree units — that's what
      // keeps the line in-scale even at extreme transpose offsets.
      int rank = RandRange(available_pitches + 1);
      notes_[s] = static_cast<uint8_t>(rank);

      oct_ups_ <<= 1;
      oct_downs_ <<= 1;

      int coinflip = RandRange(kOctaveJumpRange);
      if (coinflip < kOctaveJumpProb) {
        if (coinflip & 1) {
          oct_ups_ |= 0x1u;
        } else {
          oct_downs_ |= 0x1u;
        }
      }
    }
  }
}

void TB3PoSequencer::ApplyDensity() {
  uint8_t latest_slide = 0;
  uint8_t latest_accent = 0;

  int on_off_dens = GetOnOffDensity();
  int gate_prob = 10 + on_off_dens * 14;

  gates_ = 0;
  slides_ = 0;
  accents_ = 0;

  for (int i = 0; i < kMaxSteps; ++i) {
    gates_ <<= 1;
    gates_ |= RandBit(gate_prob) ? 1u : 0u;

    slides_ <<= 1;
    latest_slide = RandBit(latest_slide ? 10 : 18) ? 1u : 0u;
    slides_ |= latest_slide;

    accents_ <<= 1;
    latest_accent = RandBit(latest_accent ? 7 : 16) ? 1u : 0u;
    accents_ |= latest_accent;
  }
}

int TB3PoSequencer::GetOnOffDensity() const {
  return std::abs(density_ - 7);
}

int TB3PoSequencer::GetPitchChangeDensity() const {
  int d = density_;
  CONSTRAIN(d, 0, 8);
  return d;
}

bool TB3PoSequencer::RandBit(int prob) {
  return static_cast<int>(GridsRandom::GetWord() % 100u) < prob;
}

int TB3PoSequencer::GetNextStep(int step) const {
  ++step;
  if (step >= num_steps_) return 0;
  return step;
}

float TB3PoSequencer::PitchForStep(int s) const {
  if (!scale_ || active_count_ == 0 || scale_size_ == 0) {
    return 0.0f;
  }
  // Everything below is in *active-rank* units. We collapse to a (octave,
  // degree-in-octave) pair only at the end so transpose and octave shifts
  // always land on a degree that exists in active_idx_.
  int rank = static_cast<int>(notes_[s]);
  int transpose_int =
      static_cast<int>(transpose_ + (transpose_ >= 0.0f ? 0.5f : -0.5f));
  int total = rank + transpose_int + kOctaveOffset * active_count_;
  if (StepIsOctUp(s)) {
    total += active_count_;
  } else if (StepIsOctDown(s)) {
    total -= active_count_;
  }

  // Euclidean division — C++ % is implementation-defined for negatives.
  int octave = total / active_count_;
  int within = total % active_count_;
  if (within < 0) {
    within += active_count_;
    --octave;
  }
  if (octave < 0) octave = 0;
  if (octave > 16) octave = 16;

  int idx = octave * scale_size_ + static_cast<int>(active_idx_[within]);
  return scale_->cell_voltage(idx);
}

}  // namespace marbles
