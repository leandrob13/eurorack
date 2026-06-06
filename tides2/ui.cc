// Copyright 2017 Emilie Gillet.
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
// User interface.

#include "tides2/ui.h"

#include <algorithm>

#include "stmlib/system/system_clock.h"

#include "tides2/factory_test.h"

using namespace std;
using namespace stmlib;

const int32_t kLongPressDuration = 1200;
const uint32_t kBlinkDuration    = 300;  // ms

namespace tides {

/* static */
const LedColor Ui::palette_[4] = {
  LED_COLOR_GREEN,
  LED_COLOR_YELLOW,
  LED_COLOR_RED,
  LED_COLOR_OFF
};

void Ui::Init(Settings* settings, FactoryTest* factory_test, Keyframer* keyframer) {
  leds_.Init();
  switches_.Init();

  system_clock.Init();

  settings_    = settings;
  factory_test_ = factory_test;
  keyframer_    = keyframer;
  mode_         = UI_MODE_NORMAL;
  add_blink_time_ = 0;
  del_blink_time_ = 0;

  if (switches_.pressed_immediate(SWITCH_SHIFT)) {
    State* state = settings_->mutable_state();
    if (state->color_blind == 1) {
      state->color_blind = 0;
    } else {
      state->color_blind = 1;
    }
    settings_->SaveState();
  }

  // Restore UI mode to match persisted app_mode.
  if (settings_->state().app_mode == APP_MODE_KEYFRAMER) {
    mode_ = UI_MODE_KEYFRAME;
  }

  queue_.Init();

  fill(&press_time_[0], &press_time_[SWITCH_LAST], 0);
  fill(&ignore_release_[0], &ignore_release_[SWITCH_LAST], false);
}

void Ui::Poll() {
  system_clock.Tick();
  UpdateLEDs();

  switches_.Debounce();

  for (int i = 0; i < SWITCH_LAST; ++i) {
    Switch s = Switch(i);
    if (switches_.just_pressed(s)) {
      queue_.AddEvent(CONTROL_SWITCH, i, 0);
      press_time_[i] = system_clock.milliseconds();
      ignore_release_[i] = false;
    }
    if (switches_.pressed(s) && !ignore_release_[i]) {
      int32_t pressed_time = system_clock.milliseconds() - press_time_[i];
      if (pressed_time > kLongPressDuration) {
        queue_.AddEvent(CONTROL_SWITCH, i, pressed_time);
        ignore_release_[i] = true;
      }
    }
    if (switches_.released(s) && !ignore_release_[i]) {
      queue_.AddEvent(
          CONTROL_SWITCH,
          i,
          system_clock.milliseconds() - press_time_[i] + 1);
      ignore_release_[i] = true;
    }
  }
}

LedColor Ui::MakeColor(uint8_t value, bool color_blind) {
  LedColor color = palette_[value];
  if (color_blind) {
    uint8_t pwm_counter = system_clock.milliseconds() & 15;
    uint8_t triangle = (system_clock.milliseconds() >> 5) & 31;
    triangle = triangle < 16 ? triangle : 31 - triangle;

    if (value == 0) {
      color = pwm_counter < (4 + (triangle >> 2))
          ? LED_COLOR_GREEN
          : LED_COLOR_OFF;
    } else if (value == 1) {
      color = LED_COLOR_YELLOW;
    } else if (value == 2) {
      color = pwm_counter == 0 ? LED_COLOR_RED : LED_COLOR_OFF;
    }
  }
  return color;
}

void Ui::UpdateLEDs() {
  leds_.Clear();

  bool blink = system_clock.milliseconds() & 256;
  uint32_t now = system_clock.milliseconds();

  switch (mode_) {
    case UI_MODE_NORMAL:
      {
        const State& s = settings_->state();
        bool color_blind = s.color_blind == 1;

        leds_.set(LED_MODE,  MakeColor(s.mode, color_blind));
        leds_.set(LED_RANGE, MakeColor(s.range, color_blind));
        leds_.set(LED_SHIFT, MakeColor((s.output_mode + 3) % 4, color_blind));
      }
      break;

  case UI_MODE_CALIBRATION_C1:
    leds_.set(LED_RANGE, blink ? LED_COLOR_YELLOW : LED_COLOR_OFF);
    break;

  case UI_MODE_CALIBRATION_C3:
    leds_.set(LED_SHIFT, blink ? LED_COLOR_YELLOW : LED_COLOR_OFF);
    break;

  case UI_MODE_FACTORY_TEST:
    {
      size_t counter = (system_clock.milliseconds() >> 8) % 3;
      for (size_t i = 0; i < 3; ++i) {
        leds_.set(Led(i), palette_[counter]);
      }
    }
    break;

  case UI_MODE_KEYFRAME:
    {
      // LED_RANGE: solid when sitting within tolerance of a keyframe.
      bool on_keyframe = keyframer_->FindNearestKeyframe(
          keyframer_->current_timestamp(), kKFTolerance) >= 0;
      if (now - del_blink_time_ < kBlinkDuration) {
        leds_.set(LED_RANGE, blink ? LED_COLOR_RED : LED_COLOR_OFF);
      } else {
        leds_.set(LED_RANGE, on_keyframe ? LED_COLOR_RED : LED_COLOR_OFF);
      }

      // LED_MODE: brief green flash on ADD.
      if (now - add_blink_time_ < kBlinkDuration) {
        leds_.set(LED_MODE, blink ? LED_COLOR_GREEN : LED_COLOR_OFF);
      } else {
        leds_.set(LED_MODE, LED_COLOR_OFF);
      }

      // LED_SHIFT: yellow = bipolar (default), green = unipolar.
      bool bipolar = settings_->state().bank.output_bipolar != 0;
      leds_.set(LED_SHIFT, bipolar ? LED_COLOR_YELLOW : LED_COLOR_GREEN);
    }
    break;
  }
  leds_.Write();
}

void Ui::OnSwitchPressed(const Event& e) {

}

void Ui::OnSwitchReleased(const Event& e) {
  // --- Keyframe mode: all button actions go here. ---
  if (mode_ == UI_MODE_KEYFRAME) {
    if (e.data >= kLongPressDuration) {
      if (e.control_id == SWITCH_MODE) {
        // Long-press MODE: save bank and exit to normal Tides mode.
        settings_->mutable_state()->app_mode = APP_MODE_TIDES;
        settings_->SaveState();
        mode_ = UI_MODE_NORMAL;
      } else if (e.control_id == SWITCH_SHIFT) {
        // Long-press SHIFT: clear all keyframes.
        keyframer_->Clear();
        settings_->SaveState();
      }
    } else {
      // Short press.
      switch (e.control_id) {
        case SWITCH_MODE:
          {
            // ADD keyframe at current FRAME with current live pot values.
            uint16_t ts = keyframer_->current_timestamp();
            float vals[kKFNumChannels];
            for (int j = 0; j < kKFNumChannels; ++j) {
              vals[j] = keyframer_->immediate(j);
            }
            if (keyframer_->AddKeyframe(ts, vals)) {
              settings_->SaveState();
              add_blink_time_ = system_clock.milliseconds();
            }
          }
          break;
        case SWITCH_RANGE:
          {
            // DELETE nearest keyframe within tolerance.
            if (keyframer_->RemoveNearest(
                    keyframer_->current_timestamp(), kKFTolerance)) {
              settings_->SaveState();
              del_blink_time_ = system_clock.milliseconds();
            }
          }
          break;
        case SWITCH_SHIFT:
          {
            // Toggle bipolar / unipolar output polarity.
            State* s = settings_->mutable_state();
            s->bank.output_bipolar = s->bank.output_bipolar ? 0 : 1;
            settings_->SaveState();
          }
          break;
      }
    }
    return;
  }

  // --- Normal mode long-press handling. ---
  if (mode_ == UI_MODE_NORMAL && e.data >= kLongPressDuration) {
    if (e.control_id == SWITCH_MODE) {
      // Long-press MODE: enter keyframe mode.
      settings_->mutable_state()->app_mode = APP_MODE_KEYFRAMER;
      settings_->SaveState();
      mode_ = UI_MODE_KEYFRAME;
      return;
    }
    if ((e.control_id == SWITCH_RANGE && switches_.pressed(SWITCH_SHIFT)) ||
        (e.control_id == SWITCH_SHIFT && switches_.pressed(SWITCH_RANGE))) {
      mode_ = UI_MODE_CALIBRATION_C1;
      factory_test_->Calibrate(0, 1.0f, 3.0f);
      ignore_release_[SWITCH_RANGE] = ignore_release_[SWITCH_SHIFT] = true;
    }
    return;
  }

  // --- Calibration flow. ---
  if (mode_ == UI_MODE_CALIBRATION_C1) {
    factory_test_->Calibrate(1, 1.0f, 3.0f);
    mode_ = UI_MODE_CALIBRATION_C3;
    return;
  }
  if (mode_ == UI_MODE_CALIBRATION_C3) {
    factory_test_->Calibrate(2, 1.0f, 3.0f);
    mode_ = UI_MODE_NORMAL;
    return;
  }

  // --- Normal mode short-press: cycle mode / range / output_mode. ---
  State* s = settings_->mutable_state();
  switch (e.control_id) {
    case SWITCH_MODE:
      s->mode = (s->mode + 1) % 4;
      break;
    case SWITCH_RANGE:
      s->range = (s->range + 1) % 3;
      break;
    case SWITCH_SHIFT:
      s->output_mode = (s->output_mode + 1) % 4;
      break;
  }
  settings_->SaveState();
}

void Ui::DoEvents() {
  while (queue_.available()) {
    Event e = queue_.PullEvent();
    if (e.control_type == CONTROL_SWITCH) {
      if (e.data == 0) {
        OnSwitchPressed(e);
      } else {
        OnSwitchReleased(e);
      }
    }
  }

  if (queue_.idle_time() > 1000) {
    queue_.Touch();
  }
}

}  // namespace tides
