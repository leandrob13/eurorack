# Stages "Mono" synth — control reference

A self-contained monophonic synth voice. The six sections are functional blocks
of one voice, left → right like a synth block diagram. The finished voice exits
the **rightmost** jack (ch5).

```
 ch0     ch1      ch2       ch3      ch4         ch5
 OSC1    OSC2     FILTER    LFO      ENV A/Flt   ENV DR/S → MAIN OUT
```

Envelope split left→right: **attack + filter-env amount on ch4** (gate patched
here); **decay/release time + sustain on ch5** (the voice exits here). Decay and
release share one time. Envelope→pitch has been removed — for pitch/sync sweeps,
use the oscillator CV inputs (ch0/ch1) instead.

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
   *No gate handy?* **Hold ch5's button and push its slider up** to switch the
   envelope into **loop** (free-running) mode — it now self-triggers so you hear
   it without a gate.
3. Patch **1V/oct into ch0** to play pitches.
4. Bring ch5 **pot (sustain)** up and ch4 **slider (attack)** to taste, open
   the **filter (ch2 slider)**, and you have a voice.

---

## Per-section controls

### ch0 — OSC1
| Control | Function |
|---|---|
| Slider | **Coarse tune** (2 octaves: ±1 oct, centred at 12 o'clock) |
| Pot | **Shape** — meaning depends on the waveform: **Saw** = super-saw detune (0 = unison → up = lush 3-saw spread) · **Square** = pulse width · **Triangle** = wavefold amount · **Sine** = no effect |
| **Tap** | **Waveform**: green = Saw (**super-saw**) · orange = Square · red = Triangle (**wavefolder**) · off = Sine |
| **Medium press** (≈½–5 s, the regular-Stages loop gesture) | **Toggle sub-oscillator** — a square one octave below osc1, summed into osc1. When on, the button LED winks. Persists in flash. |
| Hold + Slider | **Fine tune** (±1 semitone) |
| Hold + Pot | *(reserved — no effect in v1)* |
| Input jack | **1V/oct** pitch CV (main pitch) |
| Output jack | OSC1 raw |

### ch1 — OSC2
| Control | Function |
|---|---|
| Slider | **Coarse / interval** vs OSC1 (2 octaves: ±1 oct) |
| Pot | **Shape** — meaning depends on the waveform: **Square** = pulse width · **Triangle** = wavefold amount · **Noise** = tone (dark → bright) · **Saw** = no effect (single saw) |
| **Tap** | **Waveform**: green = Saw · orange = Square · red = Triangle (**wavefolder**) · off = **Noise** |
| Hold + Slider | **Fine tune** (±1 semitone) |
| Hold + Pot | **Mix** OSC1 ↔ OSC2 (fully down = OSC1 only, fully up = OSC2 only; **defaults to a centred 50/50 blend** so both are audible) |
| Input jack | **Gate edge → hard-sync** (resets OSC2 phase to the patched signal); **CV → linear FM** into OSC2 (CV is block-rate, so this is fast pitch modulation, not audio-rate FM). *(No effect on the Noise slot.)* |
| Output jack | OSC2 raw |

### ch2 — FILTER
| Control | Function |
|---|---|
| Slider | **Cutoff** (~20 Hz … ~10 kHz) |
| Pot | **Resonance** |
| **Tap** | **Mode**: green = LP *aggressive* (MS-20-style saturating 4-pole) · orange = BP · red = HP · off = LP *gentle* (smooth 2-pole) |
| Hold + Slider | **Key-track amount** (cutoff follows pitch) |
| Hold + Pot | **Drive** (saturation into the soft clipper) |
| Input jack | **Cutoff CV** (added to the slider) |
| Output jack | Filter out (pre-VCA) |

### ch3 — LFO
| Control | Function |
|---|---|
| Slider | **Rate** (~0.05 … ~36 Hz) |
| Pot | **Depth** |
| **Tap** | **Waveform**: green = Triangle · orange = Saw · red = Square · off = Sample&Hold |
| Hold + Slider | **Fade-in** time (LFO ramps up after a gate/from start) |
| Hold + Pot | **Destination**: pot low = Pitch (vibrato) · middle = PWM · high = Cutoff (**defaults to PWM**) |
| Input jack | **Rate CV** (added to the rate slider) |
| Output jack | LFO out |

### ch4 — Attack + filter-env amount (gate input)
| Control | Function |
|---|---|
| Slider | **Attack** time |
| Pot | **Env → Filter** amount (primary; bipolar: centred = none, CW = positive, CCW = negative / envelope closes the filter) |
| **Tap** | **Attack curve**: green = linear · orange = exp (ease-in) · red = log (ease-out) · off = sharp |
| Hold + Slider | *(none)* |
| Hold + Pot | **Env → shape** amount (0 = off): the envelope sweeps the oscillator shape — saw detune / square PWM / triangle fold — on **both** oscillators |
| Input jack | **GATE / TRIGGER** → fires the envelope |
| Output jack | Envelope CV |

### ch5 — Decay/Release time + sustain + MAIN OUT
| Control | Function |
|---|---|
| Slider | **Decay & Release** time (one control drives both stages) |
| Pot | **Sustain** level |
| **Tap** | **Decay/Release curve**: green = linear · orange = exp (ease-in) · red = log (ease-out) · off = sharp |
| Hold + Slider | **Loop** the envelope (slider up = on → free-running, no gate needed) |
| Hold + Pot | *(none)* |
| Input jack | **Level / drone CV** → added to the VCA. A steady CV holds the voice open without a gate (droning); with a gate patched, it sums with the envelope. |
| Output jack | **MAIN voice out** |

---

## Notes

- The envelope (attack = ch4 slider, decay/release = ch5 slider, sustain = ch5
  pot) drives the VCA. Its **filter amount is the live ch4 pot** (bipolar).
  Envelope→pitch has been removed; use the oscillator CV inputs for pitch/sync
  sweeps. ch4/ch5 have no hold+pot gesture.
- **Persistence:** only the tap-cycled *types* (waveforms, filter mode, LFO
  waveform, curves) are saved to flash. The hidden hold+pot / hold+slider values
  (mix, fine tune, drive, key-track, LFO dest/fade, loop) are live and reset to
  defaults on power-up — by design, like all Stages knobs.
- All input jacks are **additive**: panel control sets the base value and a
  patched jack sums modulation on top.
- This is phase 1–5 of `synth_plan.md`. Phase 6 polish (true V/oct calibration,
  glide, richer LED animations) is still open.
