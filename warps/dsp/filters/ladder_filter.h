#ifndef LADDER_FILTER_H
#define LADDER_FILTER_H
#include <cmath>

#define MOOG_PI 3.1415927410125732421875f
#define VT 0.312f

namespace warps
{  

// Ported from https://github.com/ddiakopoulos/MoogLadders/blob/master/src/ImprovedModel.h
class MoogLadderFilter
{
public:
	
	MoogLadderFilter(){ }
	~MoogLadderFilter() { }

	void Init (float sample_rate) {
		sampleRate = sample_rate;
		SetFreq(1000.0f); // normalized cutoff frequency
		SetRes(0.1f); // [0, 4]
	}
	
	float Process(float in, float drive = 1.0f) {
		float dV0, dV1, dV2, dV3;
		float double_vt = 2.0f * VT;
		
		dV0 = -g * (
			tanhf(
			  (drive * in + resonance * V[3]) / double_vt
			) + tV[0]
		);
		/* 
		   Not oversampling like in the original model in order
		   to allow the filter to open more at fully CW 
		*/
		V[0] += (dV0 + dV[0]) / sampleRate;
		dV[0] = dV0;
		tV[0] = tanhf(V[0] / double_vt);
		
		dV1 = g * (tV[0] - tV[1]);
		V[1] += (dV1 + dV[1]) / sampleRate;
		dV[1] = dV1;
		tV[1] = tanhf(V[1] / double_vt);
		
		dV2 = g * (tV[1] - tV[2]);
		V[2] += (dV2 + dV[2]) / sampleRate;
		dV[2] = dV2;
		tV[2] = tanhf(V[2] / double_vt);
		
		dV3 = g * (tV[2] - tV[3]);
		V[3] += (dV3 + dV[3]) / sampleRate;
		dV[3] = dV3;
		tV[3] = tanhf(V[3] / double_vt);
		
		return V[3];
	}
	
	virtual void SetRes(float r) {
		resonance = r;
	}
	
	virtual void SetFreq(float c) {
		cutoff = c;
		x = (MOOG_PI * cutoff) / sampleRate;
		g = 4.0f * MOOG_PI * VT * cutoff * (1.0f - x) / (1.0f + x);
	}
	
private:
	
	float V[4];
	float dV[4];
	float tV[4];
	
	float x;
	float g;
	float cutoff;
	float resonance;
	float sampleRate;
};
}

#endif