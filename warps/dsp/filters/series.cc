#include "series.h"

#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define PI_F 3.1415927410125732421875f

using namespace warps;

void SeriesFilter::Init(float sample_rate)
{
    paramsH.Init(sample_rate);
    paramsL.Init(sample_rate);
    SetFreq(500.0f, &paramsH);
    SetFreq(500.0f, &paramsL);
}

void SeriesFilter::ProcessParams(float in, FilterParams *p)
{
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

float SeriesFilter::Process(float in)
{
    ProcessParams(in, &paramsH);
    ProcessParams(paramsH.out_high_, &paramsL);
    return paramsL.out_low_;
}

void SeriesFilter::SetFreq(float f, FilterParams *p)
{
    p->fc_ = fclamp(f, 1.0e-6, p->fc_max_);
    // Set Internal Frequency for fc_
    p->freq_ = 2.0f * sinf(PI_F * MIN(0.25f, p->fc_ / (p->sr_ * 2.0f))); // fs*2 because double sampled
    // recalculate damp
    p->damp_ = MIN(2.0f * (1.0f - powf(p->res_, 0.25f)), MIN(2.0f, 2.0f / p->freq_ - p->freq_ * 0.5f));
}

void SeriesFilter::SetRes(float r, FilterParams *p)
{
    float res = fclamp(r, 0.f, 1.f);
    p->res_      = res;
    // recalculate damp
    p->damp_  = MIN(2.0f * (1.0f - powf(p->res_, 0.25f)), MIN(2.0f, 2.0f / p->freq_ - p->freq_ * 0.5f));
    p->drive_ = p->pre_drive_ * p->res_;
}

void SeriesFilter::SetDrive(float d, FilterParams *p)
{
    float drv  = fclamp(d * 0.1f, 0.f, 1.f);
    p->pre_drive_ = drv;
    p->drive_     = p->pre_drive_ * p->res_;
}