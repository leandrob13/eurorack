# Stages "Mono" synth — control reference

A self-contained monophonic synth voice. The six sections are functional blocks
of one voice, left → right like a synth block diagram. The finished voice exits
the **rightmost** jack (ch5).

```
 ch0     ch1      ch2       ch3      ch4      ch5
 OSC1    OSC2     FILTER    LFO      ADSR-2   ADSR-1 → MAIN OUT
```

**Enter the mode:** hold the **3rd button from the left (ch2)** for 5 seconds.
(Hold any other section's button 5 s to switch to a different mode.)

---

## The one rule (every section works the same way)

| Gesture | What it does |
|---|---|
| **Slider** | Primary control **A** |
| **Pot** | Primary control **B** |
| **Tap button** (short press, don't move slider/pot) | Cycle the section's *type* — the **button LED** shows which one |
| **Hold button + move Pot** | Hidden "shift" parameter **B′** |
| **Hold button + move Slider** | Hidden "shift" parameter **A′** |
| **Slider LED** | Live signal animation (envelope / LFO activity) |

**Button LED colour = the selected type** (4 states):

| LED | green | orange | red | off |
|---|---|---|---|---|
| type index | 0 | 1 | 2 | 3 |

> The LED briefly blinks dark when you tap to confirm the change.

---

## Quick start (make a sound)

1. Enter the mode (hold ch2 button 5 s).
2. Patch a **gate/trigger into ch4** and **MAIN OUT from ch5**.
   *No gate handy?* Turn **ch4's slider down, hold ch4's button and push the
   slider up** to switch the envelope into **loop** (free-running) mode — it now
   self-triggers so you hear it without a gate.
3. Patch **1V/oct into ch0** to play pitches.
4. Bring ch4 **slider (sustain)** up and ch5 **slider (attack)** to taste, open
   the **filter (ch2 slider)**, and you have a voice.

---

## Per-section controls

### ch0 — OSC1
| Control | Function |
|---|---|
| Slider | **Coarse tune** (±2 octaves, centred at 12 o'clock) |
| Pot | **Shape** (pulse width on the square wave) |
| **Tap** | **Waveform**: green = Saw · orange = Square · red = Triangle · off = Sine |
| Hold + Slider | **Fine tune** (±1 semitone) |
| Hold + Pot | *(reserved — no effect in v1)* |
| Input jack | **1V/oct** pitch CV (main pitch) |
| Output jack | OSC1 raw |

### ch1 — OSC2
| Control | Function |
|---|---|
| Slider | **Coarse / interval** vs OSC1 (±2 octaves) |
| Pot | **Shape** (pulse width on the square wave) |
| **Tap** | **Waveform**: green = Saw · orange = Square · red = Triangle · off = Sine |
| Hold + Slider | **Fine tune** (±1 semitone) |
| Hold + Pot | **Mix** OSC1 ↔ OSC2 (fully down = OSC1 only, fully up = OSC2 only) |
| Input jack | **Gate edge → hard-sync** OSC2 to OSC1; **CV → FM** into OSC2 |
| Output jack | OSC2 raw |

### ch2 — FILTER
| Control | Function |
|---|---|
| Slider | **Cutoff** (~20 Hz … ~10 kHz) |
| Pot | **Resonance** |
| **Tap** | **Mode**: green = LP (12 dB) · orange = BP · red = HP · off = Ladder (24 dB) |
| Hold + Slider | **Key-track amount** (cutoff follows pitch) |
| Hold + Pot | **Drive** (saturation) |
| Input jack | **Cutoff CV** (added to the slider) |
| Output jack | Filter out (pre-VCA) |

### ch3 — LFO
| Control | Function |
|---|---|
| Slider | **Rate** (~0.05 … ~36 Hz) |
| Pot | **Depth** |
| **Tap** | **Waveform**: green = Triangle · orange = Saw · red = Square · off = Sample&Hold |
| Hold + Slider | **Fade-in** time (LFO ramps up after a gate/from start) |
| Hold + Pot | **Destination**: pot low = Pitch (vibrato) · middle = PWM · high = Cutoff |
| Input jack | *(unused in v1)* |
| Output jack | LFO out |

### ch4 — ADSR-2 (sustain / release)
| Control | Function |
|---|---|
| Slider | **Sustain** level |
| Pot | **Release** time |
| **Tap** | **Decay/Release curve**: green = linear · orange = exp (ease-in) · red = log (ease-out) · off = sharp |
| Hold + Slider | **Loop** the envelope (slider up = on → free-running, no gate needed) |
| Hold + Pot | **Env → Pitch** amount (bipolar: pot centred = none, CCW negative, CW positive) |
| Input jack | **GATE / TRIGGER** → fires the envelope |
| Output jack | Envelope CV |

### ch5 — ADSR-1 (attack / decay) + MAIN OUT
| Control | Function |
|---|---|
| Slider | **Attack** time |
| Pot | **Decay** time |
| **Tap** | **Attack curve**: green = linear · orange = exp (ease-in) · red = log (ease-out) · off = sharp |
| Hold + Slider | *(none)* |
| Hold + Pot | **Env → Filter** amount (bipolar: pot centred = none, CCW negative, CW positive) |
| Input jack | **Accent / velocity** → boosts VCA + opens cutoff |
| Output jack | **MAIN voice out** |

---

## Notes

- The single ADSR (attack ch5-slider, decay ch5-pot, sustain ch4-slider,
  release ch4-pot) drives **both** the VCA and the modulation routings
  (Env→Filter on ch5 hold+pot, Env→Pitch on ch4 hold+pot).
- **Persistence:** only the tap-cycled *types* (waveforms, filter mode, LFO
  waveform, curves) are saved to flash. The hidden hold+pot / hold+slider values
  (mix, fine tune, drive, key-track, LFO dest/fade, env-amounts, loop) are live
  and reset to defaults on power-up — by design, like all Stages knobs.
- All input jacks are **additive**: panel control sets the base value and a
  patched jack sums modulation on top.
- This is phase 1–5 of `synth_plan.md`. Phase 6 polish (true V/oct calibration,
  glide, richer LED animations) is still open.
