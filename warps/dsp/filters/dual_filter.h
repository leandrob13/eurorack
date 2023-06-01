//#pragma once
#ifndef WARPS_DSP_DUAL_FILTER_H_
#define WARPS_DSP_DUAL_FILTER_H_

#include <stdint.h>
#include <math.h>
#include "stmlib/dsp/filter.h"

#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define PI_F 3.1415927410125732421875f

using namespace stmlib;

namespace warps
{

typedef enum FilterLayout {
    LP_HP,
    LP_BP,
    BP_HP,
    BP_BP
} FilterLayout;

class DualFilter
{
  public:
    DualFilter() {}
    
    void Init() {
        filterL_.Init();
        filterR_.Init();
        layout_ = LP_HP;
    }

    float* Process(float inL, float inR) {
        static float out[2];
        float o1 = 0.0f;
        switch (layout_) {
            case LP_HP:
                {
                    o1 = filterL_.Process<FILTER_MODE_LOW_PASS>(inL);
                    out[0] = filterL_.Process<FILTER_MODE_LOW_PASS>(o1);
                    out[1] = filterR_.Process<FILTER_MODE_HIGH_PASS>(inR * 0.35f);
                }
                break;
            case LP_BP:
                {
                    o1 = filterL_.Process<FILTER_MODE_LOW_PASS>(inL);
                    out[0] = filterL_.Process<FILTER_MODE_LOW_PASS>(o1);
                    out[1] = filterR_.Process<FILTER_MODE_BAND_PASS>(inR);
                }
                break;
            case BP_HP:
                {
                    out[0] = filterL_.Process<FILTER_MODE_BAND_PASS>(inL);
                    out[1] = filterR_.Process<FILTER_MODE_HIGH_PASS>(inR * 0.35f);
                }
                break;
            case BP_BP:
                {
                    out[0] = filterL_.Process<FILTER_MODE_BAND_PASS>(inL);
                    out[1] = filterR_.Process<FILTER_MODE_BAND_PASS>(inR);
                }
                break;
            default:
                break;
        }
        
        return out;
    }

    void SetFreqsRes(float fl, float rl, float fr, float rr) {
        float l_factor = (layout_ == LP_HP || layout_ == LP_BP) ? 0.1f : 0.25f;
        float r_factor = (layout_ == BP_BP || layout_ == LP_BP) ? 0.25f : 0.04f;
        filterL_.set_f_q<FREQUENCY_FAST>(fl * l_factor, 15.0f * rl + 0.25f);
        filterR_.set_f_q<FREQUENCY_FAST>(fr * r_factor, 15.0f * rr + 0.25f);
    }

    void SetLayout(FilterLayout value) {
        layout_ = value;
    }

  private:
    Svf filterL_;
    Svf filterR_;
    FilterLayout layout_;
};

} // namespace warps
#endif
//#endif