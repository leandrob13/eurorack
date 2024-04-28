#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "tides2/resources.h"
#include "stmlib/utils/random.h"
#include "stmlib/dsp/units.h"

namespace tides {

using namespace std;
using namespace stmlib;

#define DS_DXDT(x,y,z) a * (y - x)
#define DS_DYDT(x,y,z) (c - a) * x - x * z + c * y
#define DS_DZDT(x,y,z) x * y - b * z

const float f_scale = 2.0439497f / 62500.0f;

enum Speed {
  SLOW,
  MID,
  FAST
};

class Attractors {
  public:

  void Init() {
    tx_ = Random::GetFloat();
    ty_ = Random::GetFloat();
    tz_ = Random::GetFloat();

    rx_ = Random::GetFloat();
    ry_ = Random::GetFloat();
    rz_ = Random::GetFloat();

    lx_ = Random::GetFloat();
    ly_ = Random::GetFloat();
    lz_ = Random::GetFloat();

    cx_ = Random::GetFloat();
    cy_ = Random::GetFloat();
    cz_ = Random::GetFloat();
  }  

  void set_thomas(float thomas) {
    thomas_ = thomas;
  }

  void set_chua(float chua) {
    chua_ = chua;
  }

  void set_gain(float gain) {
    gain_ = gain;
  }

  void set_rossler(float rossler) {
    rossler_ = rossler;
  }

  void set_speed(int speed) {
    speed_ = Speed(speed);
  }

  float channels(int8_t index) {
    return channels_[index];
  }

  void Process(float frequency) {
    channels_[0] = ProcessLorenzAttractor(frequency);
    channels_[1] = ProcessRosslerAttractor(frequency);
    channels_[2] = ProcessThomasSymmetricAttractor(frequency);
    channels_[3] = ProcessChuaAttractor(frequency);
  }

  float ProcessThomasSymmetricAttractor(float f) {

    float frequency = SemitonesToRatio(f) * f_scale;
    switch (speed_) {
        case SLOW:
        frequency /= 8.0f;
        break;
        case FAST:
        break;
        default:
        frequency /= 4.0f;
        break;
    }

    CONSTRAIN(frequency, 0.0f, 0.25f);
    frequency *= 32.0f;

    const float max_b = 0.2f;
    const float min_b = 0.01f;
    float b = ((max_b - min_b) * thomas_ + min_b);
    CONSTRAIN(b, min_b, max_b);

    const float amp = 0.5f;
    float x = tx_;
    float y = ty_;
    float z = tz_;
    
    const float dx = tcsa(y, x, b);
    const float dy = tcsa(z, y, b);
    const float dz = tcsa(x, z, b);
    x += frequency * dx;
    y += frequency * dy;
    z += frequency * dz;

    tx_ = x;
    ty_ = y;
    tz_ = z;

    return amp * (y - 2.f) * gain_;
  }

  float ProcessChuaAttractor(float f) {

    float frequency = SemitonesToRatio(f) * 1.3f * f_scale;
    switch (speed_) {
        case SLOW:
        frequency /= (16.0f * 16.0f);
        break;
        case FAST:
        break;
        default:
        frequency /= 16.0f;
        break;
    }
    CONSTRAIN(frequency, 0.0f, 0.01f);
    const float a = 42.0f;
    const float max_b = 6.0f;
    const float min_b = 1.0f;
    const float b = ((max_b - min_b) * chua_ + min_b);
    const float c = 28.0f;

    const float offset = -0.5f;
    const float amp = 10.0f / 8.0f;
    float x = cx_;
    float y = cy_;
    float z = cz_;
    
    const float dx = DS_DXDT(x, y, z);
    const float dy = DS_DYDT(x, y, z);
    const float dz = DS_DZDT(x, y, z);
    x += frequency * dx;
    y += frequency * dy;
    z += frequency * dz;

    float output = (x + 18.0f) / 36.0f;
    
    cx_ = x;
    cy_ = y;
    cz_ = z;

    return (amp * output + offset) * 10.0f * gain_;
  }

  float ProcessLorenzAttractor(float f) {

    float frequency = SemitonesToRatio(f) * 1.3f * f_scale;

    switch (speed_) {
        case SLOW:
        frequency /= (16.0f * 16.0f);
        break;
        case FAST:
        break;
        default:
        frequency /= 16.0f;
        break;
    }
    CONSTRAIN(frequency, 0.0f, 0.01f);

    const float sigma = 10.0f;
    const float rho = 28.0f;
    const float beta = 8.0f / 3.0f;

    float x = lx_;
    float y = ly_;
    float z = lz_;

    x += (frequency * ((sigma * (ly_ - lx_))));
    y += (frequency * ((lx_ * (rho - lz_)) - ly_));
    z += (frequency * ((lx_ * ly_) - (beta * lz_)));

    lx_ = x;
    ly_ = y;
    lz_ = z;

    return ((1.5f * x) / 4.0f) * gain_;
  }

  float ProcessRosslerAttractor(float f) {

    float frequency = SemitonesToRatio(f) * 1.3f * f_scale;

    switch (speed_) {
        case SLOW:
        frequency /= 8.0f;
        break;
        case FAST:
        frequency *= 8.0f;
        break;
        default:
        break;
    }
    CONSTRAIN(frequency, 0.0f, 0.01f);

    const float a = 0.4 * rossler_;
    const float b = 0.2;
    const float c = 5.7;

    float x = rx_;
    float y = ry_;
    float z = rz_;

    x += (-ry_ - rz_) * frequency;
    y += (rx_ + a * ry_) * frequency;
    z += (b + rz_ * (rx_ - c)) * frequency;

    rx_ = x;
    ry_ = y;
    rz_ = z;

    return (x / 2.0f) * gain_;
  }

  private:
    float tx_;
    float ty_;
    float tz_;

    float rx_;
    float ry_;
    float rz_;

    float lx_;
    float ly_;
    float lz_;

    float cx_;
    float cy_;
    float cz_;

    float thomas_;
    float gain_;
    float rossler_;
    float chua_;   
    Speed speed_;
    float channels_[4];

    inline float tcsa(float  v, const float w, const float b) {
        v *= 0.159155f; // Convert radians to phase.
        // need to calc wrap here since InterpolateWrap can't handle negatives
        // Using floorf is too slow... The ternary is apparently faster
        v -= static_cast<float>(static_cast<int32_t>(v));
        v = v < 0.0f ? 1.0f - v : v;
        return Interpolate(lut_sine, v, 1024.0f) - b * w;
    }
};

}