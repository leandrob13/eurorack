# Roboto — Plan B (pitch-shifter topology)

**Target:** Warps Symbiote feature mode, STM32F4 @ 168 MHz.
**Effect class:** Lo-fi pitch shifter (HT8950 emulation).
**Reference:** Synthrotek Roboto.

Plan A ([roboto_plan.md](roboto_plan.md)) modeled Roboto as a 1-band buzz
modulator (`carrier × envelope(input)`). After re-examining the HT8950 — and
specifically how Roboto's PITCH and RATE knobs behave on the panel — that
topology is wrong. **The HT8950 is a time-domain pitch shifter.** It
ADCs the input at a master clock rate, stores samples in a small internal
buffer, and reads them back out at a different rate to produce the pitch
shift. The "robot voice" character is the *combination* of (a) low sample
rate (RATE knob → audible aliasing), (b) low buffer resolution (granular
shift artifacts), and (c) optional LFO/sweep modulation on the read pointer
(Vibrato/Up/Down modes). The input's formants and timbre are preserved —
just transposed.

This plan keeps the lo-fi back end from Plan A (bitcrush, SR-reduce) but
swaps the buzz core for a pitch-shifter core.

---

## 1. Signal flow

```
IN2 (audio in) ──► [DC block 30 Hz]
                          │
                          ▼
                ┌─────────────────────────┐
                │ Pitch shifter            │
                │  ratio = PITCH + LFO    │     ◄── LFO/sweep
                │  (SOLA, shared FX buf)  │         (mode switch)
                └─────────────────────────┘
                          │
                          ▼
                    [SoftLimit]
                          │
                          ▼
                [Bitcrush (cont. bits)]
                          │
                          ▼
              [SR-reduce @ RATE (ZOH)]
                          │
                          ▼
                  [DC block 15 Hz]
                          │
                ┌─────────────────────────┐
                │ Dry/wet crossfade        │
                └─────────────────────────┘
                       │      │
                       ▼      ▼
                     OUT1   OUT2 (pre-crush "alt", free)
```

No carrier in this topology. IN1 is unused at the algorithm level (option to
repurpose as audio-rate pitch FM in a later phase — see §6).

---

## 2. Reuse map

| Need | Use this | Rationale |
|---|---|---|
| Pitch shifter core | `PitchShifter` in [warps/dsp/fx/pitch_shifter.h](../dsp/fx/pitch_shifter.h) | 2-tap SOLA shifter already in the tree; shares the 32k uint16_t FX buffer with reverb/ensemble/formant_shifter (mutually exclusive — Roboto joins this set). Currently commented out of the modulator dispatch in favor of tremolo, but the class is intact. |
| Audio-rate pitch ratio mod | New: small LFO + ramp generator | One sine LUT lookup + a slow-phase accumulator for sweep. Cheap. |
| Soft-clip before bitcrush | `stmlib::SoftLimit` | Standard. |
| Parameter smoothing | `stmlib::ParameterInterpolator` | Standard. |
| DC blockers | `stmlib::OnePole` | Standard. |
| Carrier-shape switch as mode selector | `parameters_.carrier_shape` | 4-position switch maps directly to HT8950's Robot/Vibrato/Up/Down. |

**Not used** (deliberately): `Vocoder`, `FilterBank`, `xmod_oscillator_` (no
carrier). The internal carrier infrastructure from Plan A is gone.

---

## 3. Control mapping

`carrier_shape` becomes the **HT8950 mode switch** directly — no more
external/internal carrier distinction (there is no carrier). LEVEL 1 reuses
cv_scaler's `note` wiring for calibrated 1 V/oct pitch.

| Physical | `Parameters` field | Function |
|---|---|---|
| **LEVEL 1 pot** | `raw_level_pot[0]` / `note` (pot part) | **PITCH** — center pitch-shift ratio (manual) |
| **LEVEL 1 CV** | `raw_level_cv[0]` / `note` (CV part) | **PITCH CV** — 1 V/oct, calibrated |
| **LEVEL 2 pot** | `raw_level_pot[1]` | **Dry/wet** |
| **LEVEL 2 CV** | `raw_level_cv[1]` | Input VCA (unpatched = 0.6, cv_scaler default) |
| **ALGORITHM** (knob+CV) | `raw_algorithm` | **RATE** — SR-reduce clock (48 k → 3 k, fractional ZOH) |
| **MODULATION** (knob+CV) | `modulation_parameter` | **Bit depth** (8 → 3, continuous) |
| **`carrier_shape` switch** | `carrier_shape` (0..3) | **Mode**: 0=Robot, 1=Vibrato, 2=Up, 3=Down |
| **IN2** | — | Input audio (mandatory) |
| **IN1** | — | Unused (Phase 4 candidate: external pitch-mod CV) |

### Mode behaviors

In all four modes the **PITCH knob sets the center shift ratio**. The mode
switch determines what (if anything) modulates that ratio:

| Mode | `carrier_shape` | Pitch behavior |
|---|---|---|
| **Robot** | 0 | Fixed shift = PITCH only. No modulation. |
| **Vibrato** | 1 | PITCH + 8 Hz sine LFO, ±1 semitone fixed depth. |
| **Up** | 2 | PITCH + slow upward sawtooth ramp, ~0.3 Hz, ±1 octave span. |
| **Down** | 3 | PITCH + slow downward sawtooth ramp, mirror of Up. |

LFO/ramp rates and depths are **fixed** for faithfulness to the chip — the
HT8950's rates aren't user-controllable. If we want them user-controllable
later, they'd have to take over a knob (which would cost us bit depth or
RATE — not worth it for Phase 1).

### LEVEL 1 has no role conflict in this plan

Unlike Plan A, LEVEL 1 doesn't dual-role by mode. There's no carrier path
to gate, so LEVEL 1 CV is *only* a pitch input (via `note`); it doesn't need
to also act as a VCA. cv_scaler already does the right thing — `note` is
populated regardless of mode, and we just don't pass `raw_level_cv[0]` into
`ApplyAmplification` (we only use channel 1 — the modulator/input audio
VCA).

---

## 4. Implementation

### 4.1 Files to add / touch

```
warps/dsp/fx/roboto.h          (new)   — Roboto class wrapping PitchShifter + lo-fi back end
warps/dsp/modulator.h          (edit)  — #include + static instance + ProcessRoboto decl
warps/dsp/modulator.cc         (edit)  — ProcessRoboto definition + dispatch case
warps/dsp/parameters.h         (edit)  — FEATURE_MODE_ROBOTO enum entry
warps/ui.cc                    (edit)  — expose the new mode in the carousel
warps/docs/roboto_plan_b.md    (this)
```

### 4.2 `dsp/fx/roboto.h` skeleton

```cpp
class Roboto {
 public:
  enum Mode {
    MODE_ROBOT = 0,
    MODE_VIBRATO,
    MODE_UP,
    MODE_DOWN,
  };

  void Init(uint16_t* buffer, float sample_rate) {
    sample_rate_ = sample_rate;
    pitch_shifter_.Init(buffer);          // shares the 32k uint16_t FX buffer
    dc_in_.Init();  dc_in_.set_f<FREQUENCY_FAST>(30.0f / sample_rate);
    dc_out_.Init(); dc_out_.set_f<FREQUENCY_FAST>(15.0f / sample_rate);
    vib_phase_ = 0.0f;  vib_inc_  = 8.0f  / sample_rate;     // 8 Hz fixed
    sweep_phase_ = 0.0f; sweep_inc_ = 0.3f / sample_rate;    // 0.3 Hz fixed
    bit_levels_ = 255.0f;
    sr_phase_ = 0.0f; sr_ratio_ = 1.0f; held_ = 0.0f;
    mix_ = mix_target_ = 1.0f;
    mode_ = MODE_ROBOT;
    pitch_semitones_ = 0.0f;
  }

  void Clear() {
    pitch_shifter_.Clear();
    vib_phase_ = sweep_phase_ = 0.0f;
    held_ = 0.0f;
    sr_phase_ = 0.0f;
  }

  // Setters from ProcessRoboto:
  void set_pitch(float note);             // semitones from parameters_.note
  void set_rate(float algo01);            // ALGORITHM → SR-reduce ratio
  void set_bits(float mod01);             // MODULATION → bit_levels_
  void set_mix(float pot01);              // LEVEL 2 pot → wet/dry target
  void set_mode(int shape) { mode_ = static_cast<Mode>(shape); }

  void Process(float* in, float* out_main, float* out_aux, size_t size);

 private:
  PitchShifter pitch_shifter_;       // SOLA, shared FX buffer
  stmlib::OnePole dc_in_, dc_out_;

  // Pitch modulation state
  Mode  mode_;
  float pitch_semitones_;            // center shift from set_pitch
  float vib_phase_, vib_inc_;        // 8 Hz vibrato LFO
  float sweep_phase_, sweep_inc_;    // 0.3 Hz Up/Down ramp

  // Lo-fi back end state
  float bit_levels_;
  float sr_phase_, sr_ratio_;
  float held_;

  float mix_, mix_target_;
  float sample_rate_;
};
```

### 4.3 Per-block processing

```cpp
void Roboto::Process(float* in, float* out_main, float* out_aux, size_t size) {
  // 1. DC-block input.
  for (size_t i = 0; i < size; ++i)
    in[i] = dc_in_.Process<FILTER_MODE_HIGH_PASS>(in[i]);

  // 2. Compute pitch-shift ratio in semitones at the block boundary.
  //    PitchShifter sets ratio once per block — sub-block LFO modulation is
  //    coarse but adequate for 8 Hz vibrato at 96-sample blocks (12.5 ms per
  //    block × 8 Hz ≈ 0.1 cycles per block — fine).
  float mod_semitones = 0.0f;
  switch (mode_) {
   case MODE_ROBOT:
    break;
   case MODE_VIBRATO:
    vib_phase_ += vib_inc_ * size;
    if (vib_phase_ >= 1.0f) vib_phase_ -= 1.0f;
    mod_semitones = stmlib::Interpolate(lut_sine, vib_phase_, kSineSize - 1) * 1.0f;
    break;
   case MODE_UP:
    sweep_phase_ += sweep_inc_ * size;
    if (sweep_phase_ >= 1.0f) sweep_phase_ -= 1.0f;
    mod_semitones = sweep_phase_ * 12.0f;       // 0 → +12 → snap back
    break;
   case MODE_DOWN:
    sweep_phase_ += sweep_inc_ * size;
    if (sweep_phase_ >= 1.0f) sweep_phase_ -= 1.0f;
    mod_semitones = (1.0f - sweep_phase_) * 12.0f - 12.0f;  // -12 → 0 → snap
    break;
  }

  float total_semitones = pitch_semitones_ + mod_semitones;
  pitch_shifter_.set_pitch_ratio(SemitonesToRatio(total_semitones));

  // 3. Pitch-shift the input. PitchShifter writes to its own out buffer.
  pitch_shifter_.Process(in, out_main, size);

  // 4. Snapshot pre-crush wet for OUT2 (free alt output).
  std::copy(out_main, out_main + size, out_aux);

  // 5. SoftLimit.
  for (size_t i = 0; i < size; ++i) out_main[i] = stmlib::SoftLimit(out_main[i]);

  // 6. Bitcrush. bit_levels_ precomputed in set_bits.
  for (size_t i = 0; i < size; ++i) {
    float x = out_main[i] * bit_levels_;
    int32_t q = static_cast<int32_t>(x + (x >= 0.0f ? 0.5f : -0.5f));
    out_main[i] = static_cast<float>(q) / bit_levels_;
  }

  // 7. SR-reduce (ZOH, fractional accumulator). Aliasing is intentional.
  for (size_t i = 0; i < size; ++i) {
    sr_phase_ += sr_ratio_;
    if (sr_phase_ >= 1.0f) { sr_phase_ -= 1.0f; held_ = out_main[i]; }
    out_main[i] = held_;
  }

  // 8. DC-block output.
  for (size_t i = 0; i < size; ++i)
    out_main[i] = dc_out_.Process<FILTER_MODE_HIGH_PASS>(out_main[i]);

  // 9. Dry/wet — dry is the post-DC-block input (not the pitch-shifted
  //    output) so the dry path is genuinely "unprocessed".
  stmlib::ParameterInterpolator mix(&mix_, mix_target_, size);
  for (size_t i = 0; i < size; ++i) {
    float m = mix.Next();
    out_main[i] = in[i] + (out_main[i] - in[i]) * m;
  }
}
```

Caveat on §3 above: the existing `PitchShifter` interface
([pitch_shifter.h](../dsp/fx/pitch_shifter.h)) doesn't expose
`set_pitch_ratio()` quite like this — it has coarse/fine/voicing setters
appropriate for the existing pitch-shifter feature mode. We may need to
either (a) add a `set_pitch_semitones()` method directly to `PitchShifter`,
or (b) wrap our own thin SOLA implementation if the existing class's API
fights us. Check before Phase 1.

### 4.4 `ProcessRoboto` in `modulator.cc`

```cpp
void Modulator::ProcessRoboto(ShortFrame* input, ShortFrame* output, size_t size) {
  float* in_audio    = buffer_[1];   // IN2 — input audio
  float* main_output = buffer_[0];
  float* aux_output  = buffer_[2];

  // VCA only the input audio channel. IN1 is unused; LEVEL 1 CV is consumed
  // upstream as pitch via parameters_.note.
  float level[2] = { 0.0f, parameters_.raw_level_cv[1] };
  ApplyAmplification(input, level, aux_output, size, true);

  int32_t shape = parameters_.carrier_shape;
  CONSTRAIN(shape, 0, 3);

  roboto.set_pitch(previous_parameters_.note);
  roboto.set_rate(previous_parameters_.raw_algorithm);
  roboto.set_bits(previous_parameters_.modulation_parameter);
  roboto.set_mix(previous_parameters_.raw_level_pot[1]);
  roboto.set_mode(shape);

  roboto.Process(in_audio, main_output, aux_output, size);

  Convert(output, main_output, aux_output, 32768.0f, size);
  previous_parameters_ = parameters_;
}
```

### 4.5 Dispatch + reset

- Add `FEATURE_MODE_ROBOTO` to the enum in [parameters.h](../dsp/parameters.h).
- Add dispatch case in `Modulator::Process()` alongside the other FX.
- Roboto **shares the 32 KB FX buffer** with reverb/ensemble/formant_shifter
  via the embedded `PitchShifter`. Mutually exclusive at runtime — same
  pattern as the other FX-buffer consumers.
- Add to the `is_fx` set in `set_feature_mode()` so `reset_fx` calls
  `roboto.Clear()` on mode switch — zeroes the SOLA buffer, LFO phases,
  held sample.

---

## 5. CPU & RAM budget (at 48 kHz internal FX rate)

| Stage | Cycles/sample (est.) |
|---|---|
| DC block in (OnePole HPF) | ~5 |
| `PitchShifter::Process` (2-tap SOLA, Hermite interp) | ~60–80 |
| SoftLimit | ~6 |
| Bitcrush | ~10 |
| SR-reduce (ZOH) | ~4 |
| DC block out | ~5 |
| Dry/wet | ~3 |
| **Total** | **~95–115 cycles/sample (~3% load)** |

The SOLA shifter dominates. Still well within FX-mode budget. LFO phase
update is per-block (not per-sample) — negligible.

RAM:
- Reuses the **shared 32 KB FX buffer** for SOLA history. No additional
  large allocation.
- Class members: `PitchShifter` state + 2 `OnePole` + LFO phases + a handful
  of scalars — under 200 bytes.
- Strictly cheaper RAM than Plan A's "new `Vocoder` instance" option, on
  par with Plan A's "reuse `vocoder_`" option.

---

## 6. Implementation phases

### Phase 1 — Pitch-shift core

1. Add `FEATURE_MODE_ROBOTO`, dispatch case, UI carousel entry.
2. Wire `PitchShifter` via the shared FX buffer. Verify mode switch
   to/from Roboto triggers `reset_fx` → `roboto.Clear()`.
3. LEVEL 1 → pitch via `parameters_.note`. Confirm 1 V/oct tracking through
   a tuner.
4. Dry/wet on LEVEL 2; LEVEL 2 CV input VCA only on channel 1.
5. Verify: voice → IN2, PITCH at noon = unity, sweep PITCH = chipmunk →
   Barry-White. Words still intelligible (sanity check that pitch-shift
   topology is correct).

### Phase 2 — Lo-fi back end

1. Bitcrush on MODULATION.
2. ZOH SR-reduce on ALGORITHM.
3. SoftLimit pre-crush, DC blockers in/out.
4. Tune ranges: 8 bits + ~6 kHz hold ≈ HT8950 territory. A/B against a
   Roboto recording.

### Phase 3 — Mode LFOs

1. Implement 8 Hz vibrato LUT-sine LFO (±1 semitone).
2. Implement Up/Down ramps (~0.3 Hz, ±1 octave).
3. Wire `carrier_shape` to mode select.
4. Confirm vibrato/sweep behave like the chip (especially that the LFO
   modulates *around* the PITCH knob, not in addition to a fixed shift).

### Phase 4 — Polish / bonus

1. Self-patch feedback (OUT1 → IN2 via attenuator) — document, don't engineer.
2. Crude-shifter A/B: if SOLA sounds too clean even after bitcrush/SR-reduce,
   prototype a ring-buffer + linear-interp shifter to expose granular
   artifacts; compare.
3. **Optional Warps bonus:** route IN1 audio (if patched) as an audio-rate
   FM modulator into `pitch_semitones_` before the LFO sum. Gives FM-rate
   pitch wobble that the original chip can't do. Free expressive control.
4. CPU profile; verify no underruns at extreme settings.

---

## 7. Validation patches

| Patch | Expected |
|-------|----------|
| Voice → IN2, mode=Robot, PITCH = +0 | Lo-fi voice, pitch unchanged, words intelligible |
| Voice → IN2, mode=Robot, PITCH = +12 | Chipmunk voice with grit |
| Voice → IN2, mode=Vibrato, PITCH = 0 | Wobbling voice, 8 Hz vibrato |
| Voice → IN2, mode=Up, PITCH = 0 | Voice slowly rising and snapping back |
| Drum loop → IN2, mode=Robot, PITCH = -7 | Pitched-down lo-fi drums |
| 1 V/oct sequence → IN2 (audio), mode=Robot | Pitch tracks the input transposition |
| Voice → IN2, mode=Robot, low RATE, 3 bits | Maximum HT8950 destruction |

---

## 8. Plan A vs. Plan B — which to build?

| | Plan A (buzz) | Plan B (pitch-shift) |
|---|---|---|
| HT8950 fidelity | Low — wrong topology | High — correct topology |
| Input identity (words, formants) | Destroyed (replaced by carrier) | Preserved (just transposed) |
| Carrier input (IN1) | Yes, expressive | None (Phase 4 bonus: FM input) |
| CPU | ~70 cyc/sample | ~110 cyc/sample |
| RAM | < 1 KB | Shares 32 KB FX buffer |
| Mode switch faithfulness | Robot/Vib/Sweep via cs_shape | Direct 4-mode emulation |
| Mutually exclusive with | None new | reverb / ensemble / formant_shifter / pitch_shifter |

**Plan B is the correct Roboto.** Plan A's buzz-vocoder topology turns out
to be a different effect (closer to Buchla 296e's "spectral shift" or a
single-band vocoder), useful in its own right but not what Roboto is.

Both could ship: Plan B as `FEATURE_MODE_ROBOTO`, Plan A as a separate
effect under a different name if there's appetite later. They sound nothing
alike on actual voice input — Plan A throws away the speaker, Plan B
preserves the speaker.
