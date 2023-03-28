//#pragma once
#ifndef WARPS_DSP_SERIES_H_
#define WARPS_DSP_SERIES_H_

#include <stdint.h>
#include <math.h>

namespace warps
{

typedef enum Configuration {
    PARALLEL,
    HP_TO_LP,
    LP_TO_HP
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
    float Process(float in);

    void ProcessParams(float in, FilterParams *p);

    /** sets the frequency of the cutoff frequency. 
        f must be between 0.0 and sample_rate / 3
    */
    void SetFreq(float f, FilterParams *p);

    /** sets the resonance of the filter.
        Must be between 0.0 and 1.0 to ensure stability.
    */
    void SetRes(float r, FilterParams *p);

    /** sets the drive of the filter 
        affects the response of the resonance of the filter
    */
    void SetDrive(float d, FilterParams *p);

    void SetDamp(FilterParams *p);

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

    void Pass(float in, FilterParams *p) {
        p->notch_ = in - p->damp_ * p->band_;
        p->low_   = p->low_ + p->freq_ * p->band_;
        p->high_  = p->notch_ - p->low_;
        p->band_  = p->freq_ * p->high_ + p->band_ - p->drive_ * p->band_ * p->band_ * p->band_;
    }

    inline float fclamp(float in, float min, float max)
    {
        return fmin(fmax(in, min), max);
    }

  private:
    FilterParams paramsL, paramsH;
};

} // namespace warps
#endif
//#endif