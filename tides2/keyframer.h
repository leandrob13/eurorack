// Copyright 2024 Leandro Bolivar.
//
// Frames keyframe interpolator ported to 3 channels for Tides 2 Symbiote.
// Values stored as uint16_t [0,65535] in KeyframeBank (persisted); all
// interpolation runs in float using the M4F FPU.

#ifndef TIDES2_KEYFRAMER_H_
#define TIDES2_KEYFRAMER_H_

#include "stmlib/stmlib.h"

namespace tides {

const uint8_t  kKFNumChannels  = 3;
const uint8_t  kKFMaxKeyframes = 64;
const uint16_t kKFTolerance    = 512;  // ~0.78% of full timestamp range

enum EasingCurve {
  EASING_CURVE_STEP        = 0,
  EASING_CURVE_LINEAR      = 1,
  EASING_CURVE_IN_QUARTIC  = 2,
  EASING_CURVE_OUT_QUARTIC = 3,
  EASING_CURVE_SINE        = 4,
  EASING_CURVE_LAST        = 5,
};

struct Keyframe {
  uint16_t timestamp;
  uint16_t id;
  uint16_t values[kKFNumChannels];
};

struct KeyframeBank {
  uint16_t num_keyframes;
  uint8_t  easing[kKFNumChannels];   // per-channel easing override (Phase 2)
  uint8_t  output_bipolar;           // 1 = +-5 V (default), 0 = 0..5 V
  uint8_t  padding[2];
  Keyframe keyframe[kKFMaxKeyframes];
};

enum AppMode {
  APP_MODE_TIDES     = 0,
  APP_MODE_KEYFRAMER = 1,
};

class Keyframer {
 public:
  Keyframer() { }
  ~Keyframer() { }

  void Init(KeyframeBank* bank);

  bool    AddKeyframe(uint16_t timestamp, const float* values);
  bool    RemoveKeyframe(uint16_t timestamp);
  bool    RemoveNearest(uint16_t timestamp, uint16_t tolerance);
  int16_t FindNearestKeyframe(uint16_t timestamp, uint16_t tolerance) const;
  void    Clear();

  void Evaluate(uint16_t timestamp);

  inline void    set_immediate(int ch, float v) { immediate_[ch] = v; }
  inline float   immediate(int ch) const        { return immediate_[ch]; }
  inline float   level(int ch) const            { return levels_[ch]; }
  inline uint16_t num_keyframes() const         { return bank_->num_keyframes; }
  inline int16_t  nearest_keyframe() const      { return nearest_keyframe_; }
  inline uint16_t current_timestamp() const     { return current_timestamp_; }

  void set_global_easing(float p) { global_easing_ = p; }

 private:
  uint16_t FindKeyframeIndex(uint16_t timestamp) const;
  float    EaseScalar(float s, EasingCurve c) const;
  float    GlobalEase(float s) const;

  KeyframeBank* bank_;
  uint16_t      id_counter_;
  uint16_t      current_timestamp_;

  float   global_easing_;
  float   immediate_[kKFNumChannels];
  float   levels_[kKFNumChannels];
  int16_t nearest_keyframe_;

  DISALLOW_COPY_AND_ASSIGN(Keyframer);
};

}  // namespace tides

#endif  // TIDES2_KEYFRAMER_H_
