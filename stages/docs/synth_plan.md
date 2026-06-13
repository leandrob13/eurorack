# Stages "Mono" — full-synth Symbiote firmware

A new multimode that turns Stages into a self-contained monophonic synth voice.
The six sections become functional blocks of one voice, laid out left→right like a
synth block diagram. Oscillators on the left; the finished voice exits the
rightmost jack.

Status: **implemented (phases 1–5)**. The voice ships in `synth_voice.{h,cc}`,
wired through `ProcessSynth` in `stages.cc` and the synth UI/LED path in
`ui.cc`. Phase 6 polish (V/oct calibration, richer LED/slider animations,
glide) is still open. Firmware builds clean with the in-tree ARM toolchain.

Decisions taken during implementation (where the plan left choices open):
- **Hidden "shift" params are RAM-latched, not flashed.** Only the tap-cycled
  discrete type (low 2 bits of `segment_configuration[ch]`) persists. Hold+pot /
  hold+slider values live in RAM (`synth_hidden_*` in `stages.cc`) and reset to
  defaults on power-up. Keeps all flash writes in the UI thread (§7 intent).
- **LFO destination is single-select** (ch3 hold+pot picks pitch/PWM/cutoff).
- **ch0 hold+pot (sub/level) is reserved but inert in v1**; ch0/ch1 hold+slider
  are osc fine-tune (±1 semitone). Glide deferred.
- **Hard sync = gate-edge phase reset** of osc2 (osc has no per-sample sync),
  and **FM is block-rate** (CV sampled once per block by the hardware).
- **V/oct uses the Ouroboros ×96 scaling** on `block.cv[0]`; true calibration is
  phase-6 work.

---

## 1. Hardware budget (per channel, ×6)

Confirmed from `io_buffer.h` (`IOBuffer::Block`) and `cv_reader.cc`:

- **Slider** — analog (`block.slider[i]`)
- **Pot** — analog (`block.pot[i]`)
- **Button** — bicolor LED (red/green → **4 states**: off / green / red / orange)
  plus a separate **slider LED** for live animation
- **Input jack** — read *both* as calibrated analog CV (`block.cv[i]`) *and* as
  gate/trigger flags (`block.input[i][]`), with insertion detect
  (`input_patched[i]`)
- **Output jack** — 16-bit DAC, audio-capable (Ouroboros mode already renders 6
  BLEP oscillators)

Existing DSP we reuse:

- `envelope.h` — full DADSHR envelope with per-stage curves → the ADSR.
- `oscillator.h` — BLEP oscillator, supports through-zero linear FM via
  `external_fm` → osc + CV-FM path.
- `variable_shape_oscillator.h` — tri↔saw↔square morph (sync was stripped;
  re-enable a gate-edge phase reset for hard sync, available upstream in Plaits).
- `quantizer.{h,cc}`, `braids_quantizer.*`, `quantizer_scales.h` — pitch
  quantization if we want scale-locked play.
- **Missing:** a filter. Add `filter.h` (SVF + ladder, see §6).

Constraint: sample rate is 31.25 kHz (`kSampleRate`) → ~15.6 kHz Nyquist, so the
top end is gently dark (same as Ouroboros). Acceptable for a mono voice.

---

## 2. Interaction grammar

One vocabulary reused by every section, so the whole panel is learnable from a
single rule:

| Gesture | Role |
|---|---|
| **Slider** | Primary continuous A |
| **Pot** | Primary continuous B |
| **Button tap** | Cycle the section's discrete *type* (curve / waveform / filter mode) — LED color shows it |
| **Button-hold + Pot** | Pot's hidden "shift" parameter (B′) |
| **Button-hold + Slider** | Slider's hidden "shift" parameter (A′) |
| **Slider LED** | Live signal animation (env level, LFO, osc activity) |

Bicolor button LED = 4 discrete states (off / green / red / orange), which maps
cleanly onto 4-way type cycles (4 waveforms, 4 filter modes, etc.).

---

## 3. Voice architecture (internal, left→right)

```
 ch0        ch1         ch2          ch3        ch4         ch5
 OSC1 ─┐                FILTER       LFO        ADSR        VCA
       ├─► MIX ───────► (cutoff/Q) ──────────► (envelope) ─► ─► MAIN OUT
 OSC2 ─┘                  ▲           │            │  │
                          │           │            │  └──► Env→Pitch (to osc)
                  Env→Filter ◄────────┼────────────┘
                          ▲           │
                          └─ LFO→cutoff (assignable destination)
```

Monophonic, two oscillators sharing one filter + VCA + envelope.

---

## 4. Control map (right→left, ch5…ch0)

| Sect | Ch | Slider | Pot | Tap (LED) | Hold+Pot | Hold+Slider |
|---|---|---|---|---|---|---|
| **ADSR-1** | 5 | Attack | Decay | Attack curve | **Env→Filter amt** (bipolar) | — |
| **ADSR-2** | 4 | Sustain | Release | Dec/Rel curve | **Env→Pitch amt** (bipolar) | retrig/loop mode |
| **LFO** | 3 | Rate | Depth | Waveform | Destination (pitch/PWM/cutoff) | Fade-in / clock div |
| **Filter** | 2 | Cutoff | Resonance | Mode (LP2/BP/HP/Ladder) | Drive | Key-track amount |
| **OSC2** | 1 | Coarse / interval | Shape (PW↔morph) | Waveform | Mix balance (1↔2) | Fine tune |
| **OSC1** | 0 | Coarse tune | Shape (PW↔morph) | Waveform | Sub / level | Fine tune (or Glide) |

ADSR per your spec: A=slider5, D=pot5, S=slider4, R=pot4; curves on the two
buttons; gate on ch4-in triggers the envelope; Env→Filter and Env→Pitch are the
hidden hold+pot layers on ch5 and ch4.

---

## 5. Jack map

Inputs are **additive** — panel control = base value, patched jack = summed
modulation (reuses the existing `cv + slider` summation in `cv_reader.cc:84`).

| Ch | Input (additive unless noted) | Output (tap) |
|---|---|---|
| 5 | Accent / velocity → VCA + cutoff | **MAIN voice out** |
| 4 | **GATE / TRIG** → envelope (discrete) | Envelope CV |
| 3 | LFO sync / reset / ext-clock (discrete) | LFO out |
| 2 | Cutoff CV (+) | Filter out (pre-VCA) |
| 1 | **gate edge → hard sync** (osc2←osc1), **CV → linear FM** into osc2 | Osc2 raw |
| 0 | **V/OCT** (calibrated, main pitch) | Osc1 raw |

ch1 uses both reads of its single jack: gate flags drive hard sync, analog CV
drives FM. V/oct on ch0 uses the per-channel calibration in
`ChannelCalibrationData` (`settings.h`).

---

## 6. Filter

Both characters, button-cycled (4 LED states):

- **LP2** — SVF 12 dB/oct lowpass (stmlib `Svf`)
- **BP** — SVF bandpass
- **HP** — SVF highpass
- **Ladder** — 4-pole 24 dB/oct Moog-style lowpass with drive

Slider = cutoff, Pot = resonance, Hold+Pot = drive, Hold+Slider = key-track
amount. Cutoff modulation sums: panel + Env→Filter + LFO→cutoff + ch2-in CV.

---

## 7. Persistence

Stages knobs are **always live** — no recall of continuous values, by design.
Only the **discrete selections** persist to flash (waveforms, curves, filter
mode, LFO destination, osc/filter modes), reusing the
`State::segment_configuration[]` path in `settings.h`. No new patch-memory system.

---

## 8. Firmware integration

### Mode slot — replace `STAGES_SLOW_LFO`

Mode entry is **one button = one mode**: `Ui::multimodes_[6]` (`ui.cc`) maps each
of the 6 buttons' 5 s long-press to exactly one `MultiMode`. There is no 7th slot,
so the synth takes over an existing one.

**Decision: replace `MULTI_MODE_STAGES_SLOW_LFO` (button 2) with
`MULTI_MODE_SYNTH`.** Rationale: it's the most redundant of the six — its slow
time range is already reachable per-segment via the range bits (`ui.cc:153-160`
writes `0x0100`/`0x0200`; SLOW range already reaches ~13.4 min, `chain_state.h`
`cv_slider` case `0x0200`). Every other mode is a distinct instrument
(ADVANCED chaos DSP, SIX_EG's six envelopes, both Ouroboros oscillator modes).
Fallback if slow-LFO presets are missed: cut `OUROBOROS_ALTERNATE` (button 5)
instead.

Concrete changes:
- In `modes.h`: rename/repurpose the `MULTI_MODE_STAGES_SLOW_LFO = 2` enumerator
  to `MULTI_MODE_SYNTH = 2` (reuse value 2 so flash-stored `State.multimode`
  doesn't shift).
- In `ui.cc`: `multimodes_[2] = MULTI_MODE_SYNTH;`.
- In `settings.h`: drop `MULTI_MODE_STAGES_SLOW_LFO` from `in_seg_gen_mode()`;
  add an `in_synth_mode()` helper.
- Remove the now-dead SLOW_LFO branches in `ui.cc` / `segment_generator.cc`.
- Inside the `Process()` block loop, special-case the synth mode with a dedicated
  voice renderer (same shape as Ouroboros's special path) instead of the six
  `SegmentGenerator`s.
- New `Voice` class owning: 2× oscillators, mixer, filter, `Envelope`, LFO, and
  the modulation matrix (env→filter, env→pitch, lfo→dest, accent).
- UI: extend `ui.cc` to interpret the §2 gestures in synth mode (tap-cycle,
  hold+pot, hold+slider) and drive LED states per §2.
- Update `stages/test/` harness `CC_FILES` for any host-testable DSP (filter,
  voice) compiled with `-DTEST`.

---

## 9. Phased plan

1. **Mode scaffold** — repurpose enum value 2 (`STAGES_SLOW_LFO` →
   `MULTI_MODE_SYNTH`), point `multimodes_[2]` at it, add `in_synth_mode()`,
   wire an empty voice renderer into `Process()`; main out on ch5, silence
   elsewhere. Confirms button-2 long-press enters the mode and chain reinit works.
2. **Oscillators** — osc1+osc2 with coarse/fine/shape/wave, V/oct on ch0, mix;
   raw taps on ch0/ch1. (Sync/FM deferred to phase 5.)
3. **Envelope + VCA** — `Envelope` ADSR from ch5/ch4 controls, gate on ch4-in,
   VCA on main out, env CV tap on ch4-out.
4. **Filter** — SVF+ladder, ch2 controls, cutoff CV on ch2-in, env→filter amount.
5. **Modulation** — LFO (ch3) with destinations, env→pitch, osc2 hard sync (gate)
   + FM (CV) on ch1-in, accent on ch5-in.
6. **Polish** — LED schemes, slider-LED animations, persistence of discrete
   selections, glide, key-tracking, calibration of V/oct.

---

## 10. Deferred / open

- Glide vs. fine-tune on ch0 hold+slider (pick during phase 2).
- Whether LFO destination is multi (sum of small amounts) or single-select.
- Optional scale-quantized pitch (have `braids_quantizer`).
- Sub-oscillator vs. osc1 level on ch0 hold+pot.
- Paraphony later (osc2 independent V/oct) — out of scope for v1 mono.
