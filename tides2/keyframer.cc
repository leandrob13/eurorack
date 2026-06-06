// Copyright 2024 Leandro Bolivar.
//
// Float-port of frames/keyframer for Tides 2 Symbiote (3 channels).
// Easing replaces the LUT-based integer version with analytic float (FPU).

#include "tides2/keyframer.h"

#include <algorithm>
#include <cmath>

namespace tides {

using namespace std;

void Keyframer::Init(KeyframeBank* bank) {
  bank_              = bank;
  global_easing_     = 0.0f;
  current_timestamp_ = 0;
  nearest_keyframe_  = -1;
  fill(&immediate_[0], &immediate_[kKFNumChannels], 0.5f);
  fill(&levels_[0],    &levels_[kKFNumChannels],    0.5f);

  // Derive id_counter from stored keyframes so IDs stay monotonic across boots.
  id_counter_ = 0;
  for (uint16_t i = 0; i < bank_->num_keyframes; ++i) {
    if (bank_->keyframe[i].id >= id_counter_) {
      id_counter_ = bank_->keyframe[i].id + 1;
    }
  }
}

// Returns the index of the first keyframe with timestamp >= t (like lower_bound).
uint16_t Keyframer::FindKeyframeIndex(uint16_t timestamp) const {
  if (!bank_->num_keyframes) return 0;
  uint16_t lo = 0, hi = bank_->num_keyframes;
  while (lo < hi) {
    uint16_t mid = lo + (hi - lo) / 2;
    if (bank_->keyframe[mid].timestamp < timestamp) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  return lo;
}

bool Keyframer::AddKeyframe(uint16_t timestamp, const float* values) {
  if (bank_->num_keyframes == kKFMaxKeyframes) return false;
  uint16_t ins = FindKeyframeIndex(timestamp);
  if (ins >= bank_->num_keyframes ||
      bank_->keyframe[ins].timestamp != timestamp) {
    for (int i = static_cast<int>(bank_->num_keyframes) - 1; i >= static_cast<int>(ins); --i) {
      bank_->keyframe[i + 1] = bank_->keyframe[i];
    }
    bank_->keyframe[ins].timestamp = timestamp;
    bank_->keyframe[ins].id        = id_counter_++;
    ++bank_->num_keyframes;
  }
  for (int j = 0; j < kKFNumChannels; ++j) {
    float clamped = values[j] < 0.0f ? 0.0f : (values[j] > 1.0f ? 1.0f : values[j]);
    bank_->keyframe[ins].values[j] = static_cast<uint16_t>(clamped * 65535.0f);
  }
  return true;
}

bool Keyframer::RemoveKeyframe(uint16_t timestamp) {
  if (!bank_->num_keyframes) return false;
  uint16_t sp = FindKeyframeIndex(timestamp);
  if (sp >= bank_->num_keyframes ||
      bank_->keyframe[sp].timestamp != timestamp) return false;
  for (uint16_t i = sp; i < bank_->num_keyframes - 1; ++i) {
    bank_->keyframe[i] = bank_->keyframe[i + 1];
  }
  --bank_->num_keyframes;
  return true;
}

bool Keyframer::RemoveNearest(uint16_t timestamp, uint16_t tolerance) {
  int16_t idx = FindNearestKeyframe(timestamp, tolerance);
  if (idx < 0) return false;
  return RemoveKeyframe(bank_->keyframe[idx].timestamp);
}

int16_t Keyframer::FindNearestKeyframe(uint16_t timestamp, uint16_t tolerance) const {
  if (!bank_->num_keyframes) return -1;
  uint16_t idx   = FindKeyframeIndex(timestamp);
  uint16_t start = idx > 0 ? idx - 1 : 0;
  uint16_t end   = idx < bank_->num_keyframes ? idx + 1 : bank_->num_keyframes;
  for (uint16_t i = start; i < end; ++i) {
    int32_t d = static_cast<int32_t>(bank_->keyframe[i].timestamp)
              - static_cast<int32_t>(timestamp);
    if (d < static_cast<int32_t>(tolerance) &&
        d > -static_cast<int32_t>(tolerance)) return static_cast<int16_t>(i);
  }
  return -1;
}

void Keyframer::Clear() {
  bank_->num_keyframes = 0;
  id_counter_          = 0;
  Keyframe empty;
  empty.timestamp = 0;
  empty.id        = 0;
  fill(&empty.values[0], &empty.values[kKFNumChannels], uint16_t(0));
  fill(&bank_->keyframe[0], &bank_->keyframe[kKFMaxKeyframes], empty);
}

float Keyframer::EaseScalar(float s, EasingCurve c) const {
  switch (c) {
    case EASING_CURVE_STEP: {
      return s < 0.5f ? 0.0f : 1.0f;
    }
    case EASING_CURVE_IN_QUARTIC: {
      return s * s * s * s;
    }
    case EASING_CURVE_OUT_QUARTIC: {
      float t = 1.0f - s;
      return 1.0f - t * t * t * t;
    }
    case EASING_CURVE_SINE: {
      return 0.5f - 0.5f * cosf(s * 3.14159265f);
    }
    default: {
      return s;  // LINEAR
    }
  }
}

// Continuously morphs across the ordered curve set as global_easing_ sweeps 0..1.
// Order: LINEAR -> IN_QUARTIC -> SINE -> OUT_QUARTIC -> STEP
float Keyframer::GlobalEase(float s) const {
  static const EasingCurve kCurves[5] = {
    EASING_CURVE_LINEAR,
    EASING_CURVE_IN_QUARTIC,
    EASING_CURVE_SINE,
    EASING_CURVE_OUT_QUARTIC,
    EASING_CURVE_STEP,
  };
  float x = global_easing_ * 4.0f;
  int   i = static_cast<int>(x);
  if (i >= 4) return EaseScalar(s, kCurves[4]);
  float f = x - static_cast<float>(i);
  return (1.0f - f) * EaseScalar(s, kCurves[i]) +
             f      * EaseScalar(s, kCurves[i + 1]);
}

void Keyframer::Evaluate(uint16_t timestamp) {
  current_timestamp_ = timestamp;

  if (!bank_->num_keyframes) {
    for (int j = 0; j < kKFNumChannels; ++j) levels_[j] = immediate_[j];
    nearest_keyframe_ = -1;
    return;
  }

  uint16_t pos = FindKeyframeIndex(timestamp);

  if (pos == 0 || pos == bank_->num_keyframes) {
    // Before the first or after the last keyframe: hold the endpoint.
    const Keyframe& src =
        bank_->keyframe[pos == 0 ? 0 : bank_->num_keyframes - 1];
    for (int j = 0; j < kKFNumChannels; ++j) {
      levels_[j] = src.values[j] / 65535.0f;
    }
  } else {
    const Keyframe& a = bank_->keyframe[pos - 1];
    const Keyframe& b = bank_->keyframe[pos];
    float scale = static_cast<float>(timestamp - a.timestamp) /
                  static_cast<float>(b.timestamp - a.timestamp);
    float s = GlobalEase(scale);
    for (int j = 0; j < kKFNumChannels; ++j) {
      float from = a.values[j] / 65535.0f;
      float to   = b.values[j] / 65535.0f;
      levels_[j] = from + (to - from) * s;
    }
  }

  // Track which keyframe is closest for the LED indicator.
  if (pos == 0) {
    nearest_keyframe_ = 0;
  } else if (pos == bank_->num_keyframes) {
    nearest_keyframe_ = static_cast<int16_t>(bank_->num_keyframes - 1);
  } else {
    uint16_t d_prev = timestamp - bank_->keyframe[pos - 1].timestamp;
    uint16_t d_next = bank_->keyframe[pos].timestamp - timestamp;
    nearest_keyframe_ = static_cast<int16_t>(d_next < d_prev ? pos : pos - 1);
  }
}

}  // namespace tides
