#ifndef MUSIC_DSP_H
#define MUSIC_DSP_H
#include <cmath>

//#include "util.h"

//#define MOOG_PI 3.141592741012f
#define MOOG_PI 3.1415927410125732421875f

namespace warps
{  

class MoogLadderFilter
{
	
public:
    MoogLadderFilter() {}
	
	void Init(float sampleRate)
	{
        sampleRate_ = sampleRate;
		SetFreq(1000.0f);
		SetRes(0.0f);
        p = 0.0f;
	    k = 0.0f;
	    t1 = 0.0f;
	    t2 = 0.0f;
        for (int i = 0; i < 4; i++) {
            stage[i] = 0.0f;
            delay[i] = 0.0f;
        }
	}
	
	float Process(float sample)
	{
		float x = sample - resonance_ * stage[3];

        // Four cascaded one-pole filters (bilinear transform)
        stage[0] = x * p + delay[0]  * p - k * stage[0];
        stage[1] = stage[0] * p + delay[1] * p - k * stage[1];
        stage[2] = stage[1] * p + delay[2] * p - k * stage[2];
        stage[3] = stage[2] * p + delay[3] * p - k * stage[3];
    
        // Clipping band-limited sigmoid
        stage[3] -= (stage[3] * stage[3] * stage[3]) / 6.0f;
        
        delay[0] = x;
        delay[1] = stage[0];
        delay[2] = stage[1];
        delay[3] = stage[2];

        sample = stage[3];
        return sample;
	}
	
	void SetRes(float r)
	{
		resonance_ = (t2 == 0 && t1 == 0) ? 0: r * (t2 + 6.0f * t1) / (t2 - 6.0f * t1);
	}
	
	void SetFreq(float c)
	{
		cutoff_ = 2.0f * c / sampleRate_;

		p = cutoff_ * (1.8f - 0.8f * cutoff_);
		k = 2.0f * sin(cutoff_ * MOOG_PI * 0.5f) - 1.0f;
		t1 = (1.0f - p) * 1.386249f;
		t2 = 12.0f + t1 * t1;

		SetRes(resonance_);
	}
	
private:
	float stage[4];
	float delay[4];

	float p;
	float k;
	float t1;
	float t2;

    float cutoff_;
	float resonance_;
	float sampleRate_;
};
}

#endif