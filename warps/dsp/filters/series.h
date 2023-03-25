//#pragma once
#ifndef WARPS_DSP_SERIES_H_
#define WARPS_DSP_SERIES_H_

#include <stdint.h>
#include <math.h>
//#ifdef __cplusplus

namespace warps
{

typedef struct FilterParams {
    float res_, damp_;
    float notch_, low_, high_, band_;
    float out_low_, out_high_;
    float pre_drive_, drive_;
    float fc_, freq_, sr_, fc_max_;

    void Init(float sample_rate) {
        sr_        = sample_rate;
        fc_        = 200.0f;
        res_       = 0.0f;
        drive_     = 0.5f;
        pre_drive_ = 0.5f;
        freq_      = 0.25f;
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

/** Moog ladder filter module
Ported from soundpipe
Original author(s) : Victor Lazzarini, John ffitch (fast tanh), Bob Moog
*/
class SeriesFilter
{
  public:
    SeriesFilter() {}
    //~MoogLadder() {}
    /** Initializes the MoogLadder module.
        sample_rate - The sample rate of the audio engine being run. 
    */
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

    void SetFreqs(float fh, float fl) {
        SetFreq(fh, &paramsH);
        SetFreq(fl, &paramsL);
    }

    void SetResonances(float rh, float rl) {
        SetRes(rh, &paramsH);
        SetRes(rl, &paramsL);
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