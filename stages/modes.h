#ifndef STAGES_MULTIMODE_H
#define STAGES_MULTIMODE_H

namespace stages {
enum MultiMode {
  MULTI_MODE_STAGES = 0,
  MULTI_MODE_STAGES_ADVANCED = 5,
  // Value 2 was MULTI_MODE_STAGES_SLOW_LFO; repurposed for the Symbiote synth
  // voice. The numeric value is kept so flash-stored State.multimode does not
  // shift for users who had button-2 selected.
  MULTI_MODE_SYNTH = 2,
  MULTI_MODE_SIX_EG = 3,
  MULTI_MODE_OUROBOROS = 1,
  MULTI_MODE_OUROBOROS_ALTERNATE = 4
};
}

#endif