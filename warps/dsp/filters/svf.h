//#pragma once
#ifndef WARPS_DSP_FILTERS_MOOGLADDER_H_
#define WARPS_DSP_FILTERS_MOOGLADDER_H_

#include <stdint.h>
#include <math.h>
//#ifdef __cplusplus

namespace warps
{

inline float fclamp(float in, float min, float max)
{
    return fmin(fmax(in, min), max);
}
/** Moog ladder filter module
Ported from soundpipe
Original author(s) : Victor Lazzarini, John ffitch (fast tanh), Bob Moog
*/
class Svf
{
  public:
    Svf() {}
    //~MoogLadder() {}
    /** Initializes the MoogLadder module.
        sample_rate - The sample rate of the audio engine being run. 
    */
    void Init(float sample_rate);


    /** 
        Process the input signal, updating all of the outputs.
    */
    void Process(float in);


    /** sets the frequency of the cutoff frequency. 
        f must be between 0.0 and sample_rate / 3
    */
    void SetFreq(float f);

    /** sets the resonance of the filter.
        Must be between 0.0 and 1.0 to ensure stability.
    */
    void SetRes(float r);

    /** sets the drive of the filter 
        affects the response of the resonance of the filter
    */
    void SetDrive(float d);
    /** lowpass output
        \return low pass output of the filter
    */
    inline float Low() { return out_low_; }
    /** highpass output
        \return high pass output of the filter
    */
    inline float High() { return out_high_; }
    /** bandpass output
        \return band pass output of the filter
    */
    inline float Band() { return out_band_; }
    /** notchpass output
        \return notch pass output of the filter
    */
    inline float Notch() { return out_notch_; }
    /** peak output
        \return peak output of the filter
    */
    inline float Peak() { return out_peak_; }

  private:
    float sr_, fc_, res_, drive_, freq_, damp_;
    float notch_, low_, high_, band_;
    float out_low_, out_high_, out_band_, out_peak_, out_notch_;
    float pre_drive_, fc_max_;
};
} // namespace warps
#endif
//#endif