// Copyright 2025 Marbles Grids/TB-3PO port.
//
// Generative TB-303 / TB-3PO style acid sequencer driving the X-section when
// the T-section is running Grids. Algorithm ported from the O&C / Hemisphere
// TB_3PO applet; UI, hex-seed editing, density automation, and the no_slides /
// hold_pitch toggles are dropped from v1.

#ifndef MARBLES_TB3PO_TB3PO_SEQUENCER_H_
#define MARBLES_TB3PO_TB3PO_SEQUENCER_H_

#include "stmlib/stmlib.h"

namespace marbles {

struct Scale;

class TB3PoSequencer {
 public:
  static const int kMaxSteps = 32;
  // Matches kMaxDegrees in marbles/random/quantizer.h. Kept local so we can
  // forward-declare Scale instead of pulling its header into this one.
  static const int kMaxScaleDegrees = 16;

  TB3PoSequencer() { }
  ~TB3PoSequencer() { }

  void Init();

  // Block-level setters (called from marbles.cc Process()).
  void set_density(int encoder, int cv);     // encoder 0..14, cv -7..+7
  void set_transpose(float scale_degrees);   // bipolar, knob+CV combined
  void set_length(int steps);                // clamped to 1..kMaxSteps
  void set_lock_seed(bool locked);           // from state.x_deja_vu != OFF
  void set_scale(const Scale* scale);        // pointer into settings.persistent_data().scale

  // Persistence.
  uint16_t seed() const { return seed_; }
  void set_seed(uint16_t s);
  void Reseed();   // draw a new 16-bit seed via GridsRandom and regenerate.

  // Clock.
  void Tick(bool reset);          // one Grids "X step" boundary (rising X1 edge)
  void TickHalfCycle();           // half-step (falling X1 edge) — drives gate-off
  // Release the gate immediately. Used by marbles.cc's external-clock
  // watchdog: when ramps.master freezes (T-clock silent) the half-cycle
  // trigger never fires, so gate_ would latch HIGH and downstream VCAs/ADSRs
  // would never release.
  void ForceGateOff();

  // Per-sample slide IIR (called from the audio loop).
  void StepSlide();
  float pitch_volts() const { return pitch_volts_; }
  bool  gate() const        { return gate_; }
  bool  accent() const      { return accent_ && gate_; }

 private:
  bool RandBit(int prob);
  int  GetNextStep(int step) const;
  int  GetOnOffDensity() const;
  int  GetPitchChangeDensity() const;
  void ApplyDensity();
  void RegeneratePitches();
  void RegenerateAll();
  void RegenerateIfDirty();
  void BuildActiveDegrees();
  float PitchForStep(int s) const;

  bool StepIsGated(int s) const   { return gates_    & (1u << s); }
  bool StepIsSlid(int s) const    { return slides_   & (1u << s); }
  bool StepIsAccent(int s) const  { return accents_  & (1u << s); }
  bool StepIsOctUp(int s) const   { return oct_ups_  & (1u << s); }
  bool StepIsOctDown(int s) const { return oct_downs_ & (1u << s); }

  // Pattern.
  uint16_t seed_;
  bool     lock_seed_;
  uint8_t  num_steps_;
  uint8_t  current_pattern_density_;
  uint8_t  current_pattern_scale_size_;

  uint32_t gates_;
  uint32_t slides_;
  uint32_t accents_;
  uint32_t oct_ups_;
  uint32_t oct_downs_;
  uint8_t  notes_[kMaxSteps];

  // Live parameters.
  int   density_encoder_;
  int   density_cv_;
  int   density_;          // clamp(encoder + cv, 0, 14)
  float transpose_;
  const Scale* scale_;
  uint8_t scale_size_;     // full scale->num_degrees (used for octave math)

  // Active (in-scale) degree filter, rebuilt on scale change. notes_[s] holds
  // an index from active_idx_[] so TB-3PO never plays an out-of-scale note
  // even on a weight-based 12-degree preset like C major or Pentatonic.
  uint8_t active_idx_[kMaxScaleDegrees];
  uint8_t active_count_;

  // Playback.
  uint8_t step_;
  bool    gate_;
  bool    accent_;
  bool    gate_off_pending_;

  // Slide IIR.
  float pitch_volts_;
  float slide_target_;
  float slide_start_;

  DISALLOW_COPY_AND_ASSIGN(TB3PoSequencer);
};

}  // namespace marbles

#endif  // MARBLES_TB3PO_TB3PO_SEQUENCER_H_
