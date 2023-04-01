#include "dual.h"

using namespace warps;

void SeriesFilter::Init(float sample_rate)
{
    paramsR.Init(sample_rate);
    paramsL.Init(sample_rate);
    config = LP_HP;
}

float* SeriesFilter::Process(float inL, float inR)
{
    static float out[2];
    ProcessParams(inR, &paramsR);
    ProcessParams(inL, &paramsL);
    switch (config) {
        case LP_HP:
            {
                out[0] = paramsL.out_low_;
                out[1] = paramsR.out_high_;
            }
            break;
        case LP_N:
            {
                out[0] = paramsL.out_low_;
                out[1] = paramsR.out_notch_;
            }
            break;
        case BP_HP:
            {
                out[0] = paramsL.out_band_;
                out[1] = paramsR.out_high_;
            }
            break;
        case BP_BP:
            {
                out[0] = paramsL.out_band_;
                out[1] = paramsR.out_band_;
            }
            break;
        default:
            {
                out[0] = paramsL.out_low_;
                out[1] = paramsR.out_high_;
            }
            break;
    }
    
    return out;
}