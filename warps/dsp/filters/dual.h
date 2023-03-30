//#pragma once
#ifndef WARPS_DSP_SERIES_H_
#define WARPS_DSP_SERIES_H_

#include <stdint.h>
#include <math.h>

#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define PI_F 3.1415927410125732421875f

namespace warps
{

typedef enum Configuration {
    PARALLEL,
    HP_TO_LP,
    LP_TO_HP,
    DUAL_BAND
} Configuration;

typedef struct FilterParams {
    float res_, damp_;
    float notch_, low_, high_, band_;
    float out_low_, out_high_;
    float pre_drive_, drive_;
    float fc_, freq_, sr_, fc_max_;

    void Init(float sample_rate) {
        sr_        = sample_rate;
        fc_        = 200.0f;
        res_       = 0.1f;
        drive_     = 0.5f;
        pre_drive_ = 0.5f; // Adjust this to see effects on resonance
        freq_      = 500.0f;
        damp_      = 0.0f;
        notch_     = 0.0f;
        low_       = 0.0f;
        high_      = 0.0f;
        band_      = 0.0f;
        out_low_   = 0.0f;
        out_high_  = 0.0f;
        fc_max_    = sr_ / 3.f;
    }
} FilterParams;

class SeriesFilter
{
  public:
    SeriesFilter() {}
    
    void Init(float sample_rate);

    /** 
        Process the input signal, updating all of the outputs.
    */
    float Process(float in1, float in2);

    float Process(float in);

    void SetFreqs(float fh, float fl) {
        SetFreq(fh, &paramsH);
        SetFreq(fl, &paramsL);
    }

    void SetResonances(float rh, float rl) {
        SetRes(rh, &paramsH);
        SetRes(rl, &paramsL);
    }

    void SetDamps() {
        SetDamp(&paramsH);
        SetDamp(&paramsL);
    }

    void SetConfig(int32_t value) {
        switch (value)
        {
        case 0:
            config = PARALLEL;
            break;
        case 1:
            config = HP_TO_LP;
            break;
        case 2:
            config = LP_TO_HP;
            break;
        case 3:
            config = DUAL_BAND;
            break;
        default:
            config = PARALLEL;
            break;
        }
    }

    float High() {
        return paramsH.high_;
    }

    float Low() {
        return paramsL.low_;
    }

    Configuration Config() {
        return config;
    }

  private:
    FilterParams paramsL, paramsH;
    Configuration config;

    void ProcessParams(float in, FilterParams *p) {
        // first pass
        Pass(in, p);
        // take first sample of output
        p->out_low_   = 0.5f * p->low_;
        p->out_high_  = 0.5f * p->high_;
        // second pass
        Pass(in, p);
        // average second pass outputs
        p->out_low_ += 0.5f * p->low_;
        p->out_high_ += 0.5f * p->high_;
    }

    void Pass(float in, FilterParams *p) {
        p->notch_ = in - p->damp_ * p->band_;
        p->low_   = p->low_ + p->freq_ * p->band_;
        p->high_  = p->notch_ - p->low_;
        p->band_  = p->freq_ * p->high_ + p->band_ - p->drive_ * p->band_ * p->band_ * p->band_;
    }

    void SetFreq(float f, FilterParams *p) {
        p->fc_ = fclamp(f, 1.0e-6, p->fc_max_);
        // Set Internal Frequency for fc_
        p->freq_ = 2.0f * sinf(PI_F * MIN(0.2f, p->fc_ / (p->sr_ * 2.0f))); // fs*2 because double sampled
    }

    void SetRes(float r, FilterParams *p) {
        p->res_      = fclamp(r, 0.f, 1.0f);
        p->drive_ = p->pre_drive_ * p->res_;
    }

    void SetDrive(float d, FilterParams *p) {
        float drv  = fclamp(d * 0.1f, 0.f, 1.f);
        p->pre_drive_ = drv;
        p->drive_     = p->pre_drive_ * p->res_;
    }

    void SetDamp(FilterParams *p) {
        float x = expf(-5.0f * p->res_);
        p->damp_ = MIN(2.0f * x, MIN(2.0f, 2.0f / p->freq_ - p->freq_ * 0.5f));
    }

    inline float fclamp(float in, float min, float max) {
        return fmin(fmax(in, min), max);
    }
};

} // namespace warps
#endif
//#endif