#include "dual.h"

using namespace warps;

void SeriesFilter::Init(float sample_rate)
{
    paramsH.Init(sample_rate);
    paramsL.Init(sample_rate);
    config = HP_TO_LP;
}

float SeriesFilter::Process(float in) {
    ProcessParams(in, &paramsH);
    ProcessParams(paramsH.out_high_, &paramsL);
    return paramsL.out_low_;
}

float SeriesFilter::Process(float in1, float in2)
{
    float out = 0.0f;
    switch (config) {
        case PARALLEL:
            {
                ProcessParams(in1, &paramsH);
                ProcessParams(in2, &paramsL);
                out = paramsH.out_high_ + paramsL.out_low_;
            }
            break;
        case HP_TO_LP:
            {
                ProcessParams(in1 + in2, &paramsH);
                ProcessParams(paramsH.out_high_, &paramsL);
                out = paramsL.out_low_;
            }
            break;
        case LP_TO_HP:
            {
                ProcessParams(in1 + in2, &paramsL);
                ProcessParams(paramsL.out_low_, &paramsH);
                out = paramsH.out_high_;
            }
            break;
        case DUAL_BAND:
            {
                ProcessParams(in1, &paramsH);
                ProcessParams(in2, &paramsL);
                out = paramsH.band_ + paramsL.band_;
            }
            break;
        default:
            {
                ProcessParams(in1 + in2, &paramsH);
                ProcessParams(paramsH.out_high_, &paramsL);
                out = paramsL.out_low_;
            }
            break;
    }
    
    return out;
}