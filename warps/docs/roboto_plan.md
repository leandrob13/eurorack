# Roboto — Warps Symbiote Implementation Plan

**Target:** Warps Symbiote feature mode, STM32F4 @ 168 MHz.
**Effect class:** Lo-fi buzz modulator (HT8950-flavoured).
**Reference:** Synthrotek Roboto.

This plan slots Roboto into the existing Warps Symbiote FX scaffolding
([warps/CLAUDE.md](../CLAUDE.md)).

**Architecture decision: 1-band buzz, not multi-band vocoder.**
The HT8950 is not a vocoder — it is a buzz oscillator whose pitch is shifted
and whose amplitude follows the input envelope. A faithful Roboto is one
envelope follower on the modulator, one carrier oscillator (internal or IN1),
and a lo-fi back end (bitcrush + sample-rate reduce). The 20-band
`Vocoder`/`FilterBank` infrastructure exists in this tree
([dsp/vocoder.h](../dsp/vocoder.h)) but is the wrong topology for this
effect — it adds vowel intelligibility Roboto doesn't have, costs ~200
cycles/sample of CPU, and flattens the carrier-shape switch (every shape
sounds similar after band-splitting). 1-band gives the freed CPU to do the
SR-reducer / bitcrush / carrier rendering properly.

---

## 1. Signal flow

```
IN2 (modulator: voice/source) ──► [HPF 30 Hz DC block]
                                       │
                                       ▼
                            [Rectify + OnePole env follower]
                                       │
IN1 (external carrier, shape==0)       │  env (scalar per sample)
   │                                   │
   ├── shape!=0 ──► internal Osc ──────┤  (saw / square / noise)
   │   (pitch = parameters_.note,      │
   │    1V/oct via LEVEL 1 CV)         │
   ▼                                   ▼
   carrier   ────────────×─────────────┘
                         │
                         ▼
                  [SoftLimit] ──► [Bitcrush (cont. bits)]
                         │
                         ▼
              [SR-reduce (fractional, ZOH)]
                         │
                         ▼
                  [DC block out]
                         │
                  [Dry/wet crossfade]
                    │         │
                    ▼         ▼
                  OUT1      OUT2 (pre-crush "alt", free)
```

Mono in / mono out at the algorithm level. OUT2 carries the pre-bitcrush
wet signal — a Parasites-style alt output that costs nothing because it's
already in `buffer_[2]`.

The SR-reducer is intentionally zero-order-hold (no anti-aliasing) —
aliased imaging is *the* HT8950 character. The bitcrush also runs at-rate
for the same reason. The freed CPU from going 1-band is headroom, not
something we spend on smoothing the lo-fi away.

---

## 2. Reuse map — don't rewrite this code

| Need | Use this | Rationale |
|---|---|---|
| Internal carrier osc | `xmod_oscillator_` in `Modulator` | V/oct-calibrated via `parameters_.note`, PolyBLEP-clean. `RenderCarrier()` at [modulator.h:221](../dsp/modulator.h#L221) does the wiring. `OscillatorShape` enum already covers saw / square / noise. |
| Envelope follower | `EnvelopeFollower` in [vocoder.h:41-93](../dsp/vocoder.h#L41-L93), *or* roll a 5-line `OnePole` rectifier-LPF | The Vocoder's follower is fine as-is — pulled out as a standalone, it's tiny. Has the correct asymmetric attack/decay branch (`error > 0 ? attack : decay`). The original plan's `env[i] += ... ; if (rect < env) env += ...` adds both updates and shortens release — don't repeat that. |
| Soft-clip before bitcrush | `stmlib::SoftLimit` | Used everywhere in this tree. |
| Parameter smoothing across block | `stmlib::ParameterInterpolator` | Standard Warps idiom; eliminates zipper on knob/CV sweeps. |
| One-pole HPF/LPF (DC block, smoothing) | `stmlib::OnePole` | Standard. |
| Jack-detect / carrier source | `parameters_.carrier_shape` + `cv_scaler` | `carrier_shape == 0` means "external carrier" in stock semantics. Don't roll detection. |

**Not used** (deliberately): `Vocoder`, `FilterBank`, `BandGain[]`, the
20-band delay-line pool. None of it is needed for a 1-band buzz.

---

## 3. Control mapping — final proposal

cv_scaler ([warps/cv_scaler.cc:174-200](../cv_scaler.cc#L174-L200)) wires
**LEVEL 1 pot and LEVEL 1 CV into `parameters_.note`** as the stock-meta
carrier pitch (calibrated 1 V/oct on the CV, plus `60·pot + 12` semitones
on the pot, with a +24-semitone normaling bump when LEVEL 1 jack is
unpatched). LEVEL 1 also drives `raw_level_pot[0]` / `raw_level_cv[0]`
which existing FX modes use as input VCAs. Roboto resolves the role
overlap via the `carrier_shape` switch, mirroring stock-vocoder muscle
memory (LEVEL 1 = carrier pitch in vocoder mode).

| Physical | `Parameters` field | `shape == 0` (external carrier) | `shape != 0` (internal carrier) |
|---|---|---|---|
| **LEVEL 1 CV** | `raw_level_cv[0]` / `note` | Carrier-path input VCA (unpatched = 0.6) | **1 V/oct carrier pitch** |
| **LEVEL 2 CV** | `raw_level_cv[1]` | Modulator-path input VCA (unpatched = 0.6) | Modulator-path input VCA |
| **LEVEL 1 pot** | `raw_level_pot[0]` / `note` | **Envelope response** (slow → fast attack & release) | **Carrier pitch (manual)** |
| **LEVEL 2 pot** | `raw_level_pot[1]` | **Dry/wet** | **Dry/wet** |
| **ALGORITHM** (knob + CV) | `raw_algorithm` | **SR-reduce** (48 k → 3 k, fractional) | **SR-reduce** |
| **MODULATION** (knob + CV) | `modulation_parameter` | **Bit depth** (8 → 3, continuous) | **Bit depth** |
| `carrier_shape` switch | `carrier_shape` (0..3) | `0` = external IN1 | `1` = saw, `2` = square, `3` = noise |
| **IN1** | — | External carrier audio | (unused; internal osc) |
| **IN2** | — | Voice / modulator source | Voice / modulator source |

Rationale:

1. cv_scaler already wires LEVEL 1 → `note` with calibrated 1 V/oct. Going
   with the grain instead of fighting it gives the "singing robot tracks a
   sequence" patch for free via internal carrier, without sacrificing
   external-carrier flexibility (still available via `shape == 0`).
2. The lo-fi pair (bitcrush + SR-reduce) lives on the two clean axes
   (MODULATION + ALGORITHM), keeping the destructive SR-reduce on the big
   centre knob where Roboto puts its RATE control.
3. LEVEL 1 pot does double duty by mode: envelope response when external
   (where you most want to dial the buzz from "smooth pad-following" to
   "choppy gating"), pitch when internal. Obvious from panel state.
4. Carrier shape uses the existing 4-way `carrier_shape` switch, not a knob.
5. Dry/wet exposed on LEVEL 2 pot. LEVEL 2 CV stays a modulator-path VCA
   unconditionally (free amplitude-mod / gating on the source).

Implementation consequences for `ProcessRoboto`:

- Conditionally call `ApplyAmplification(..., raw_level_cv, ..., true)` for
  the **carrier path (channel 0) only when `shape == 0`**. When internal,
  the carrier is generated at unity by `xmod_oscillator_.Render(shape,
  parameters_.note, ...)` and LEVEL 1 CV's only consumer is `note`.
- The modulator path (channel 1) always uses `raw_level_cv[1]` as VCA.
- When `shape != 0`, `raw_level_pot[0]` is *not* read directly (its job is
  being baked into `note` by cv_scaler — using it again would double-count).
  Envelope response defaults to a fixed Roboto-character value (~5 ms attack,
  ~50 ms release).
- When `shape == 0`, `parameters_.note` is irrelevant (no internal osc),
  and `raw_level_pot[0]` is read as envelope response. Map: pot value drives
  both attack and release coefficients on an exp curve, e.g.
  attack 50→1 ms, release 500→10 ms across the knob sweep.

---

## 4. Implementation — concrete steps

### 4.1 Files to add / touch

```
warps/dsp/fx/roboto.h          (new)   — self-contained Roboto class
warps/dsp/modulator.h          (edit)  — #include + static instance + ProcessRoboto decl
warps/dsp/modulator.cc         (edit)  — ProcessRoboto definition + dispatch case
warps/dsp/parameters.h         (edit)  — FEATURE_MODE_ROBOTO enum entry
warps/ui.cc                    (edit)  — expose the new mode in the carousel
warps/docs/roboto_plan.md      (this)  — design doc
```

### 4.2 `dsp/fx/roboto.h` skeleton

```cpp
class Roboto {
 public:
  void Init(float sample_rate) {
    sample_rate_ = sample_rate;
    vocoder_.Init(sample_rate);              // reuse stock vocoder pipeline
    dc_in_.Init();  dc_in_.set_f<FREQUENCY_FAST>(30.0f / sample_rate);
    dc_out_.Init(); dc_out_.set_f<FREQUENCY_FAST>(15.0f / sample_rate);
    envelope_ = 0.0f;
    attack_  = ms_to_coeff(5.0f);
    release_ = ms_to_coeff(50.0f);
    bit_levels_ = 255.0f;                    // 8-bit default
    sr_phase_ = 0.0f; sr_ratio_ = 1.0f; held_ = 0.0f;
    mix_ = mix_target_ = 1.0f;
    carrier_shape_ = 0;
  }

  // Setters (driven from ProcessRoboto with previous_parameters_):
  void set_bits(float mod01);                // MODULATION → 8..3 → bit_levels_
  void set_mix(float pot01);                 // LEVEL 2 pot → wet/dry target
  void set_sr_hold(float algo01);            // ALGORITHM → fractional ratio
  void set_response(float pot01);            // LEVEL 1 pot (shape==0 only) → att+rel coeffs
  void set_carrier_shape(int shape);         // 0..3 → external / saw / sq / noise
  void set_note(float note);                 // V/oct for internal carrier

  void Process(float* carrier_in, float* modulator_in,
               float* out_main, float* out_aux, size_t size);

 private:
  // Carrier — owned, not the Modulator's shared xmod_oscillator_, to avoid
  // fighting stock-mode state when switching back and forth.
  Oscillator osc_;
  stmlib::OnePole dc_in_, dc_out_;

  // Envelope follower (1-band, scalar). The Vocoder's EnvelopeFollower
  // would also work; keeping it inline avoids dragging in vocoder.h.
  float envelope_;
  float attack_, release_;     // exp-LPF coefficients

  // Lo-fi back end state
  float bit_levels_;           // precomputed (2^bits) - 1
  float sr_phase_;             // fractional accumulator in [0,1)
  float sr_ratio_;             // step per sample, in (0,1]
  float held_;                 // S&H state

  float mix_, mix_target_;     // smoothed dry/wet
  int   carrier_shape_;        // 0..3
  float note_;
  OscillatorShape internal_shape_;  // mapped from carrier_shape_ when !=0
  float sample_rate_;
};
```

Notes on the skeleton:

- **No `Vocoder` member.** Removed alongside the topology change. Saves the
  ~30 KB it would have cost as an independent instance (versus reusing
  `Modulator::vocoder_`).
- **Envelope follower is scalar.** One float of state, two coefficients,
  rectifier + asymmetric one-pole. Five lines in the inner loop.
- **`Oscillator osc_` is owned, not borrowed.** The stock vocoder mode also
  uses `xmod_oscillator_` and we don't want phase resets when toggling
  modes. A second `Oscillator` is small (no large tables — those are in
  `resources`).

### 4.3 Per-block processing pseudo

```cpp
void Roboto::Process(float* carrier_in, float* mod_in,
                     float* out_main, float* out_aux, size_t size) {
  // 1. DC-block the modulator (envelope follower isn't biased by DC).
  for (size_t i = 0; i < size; ++i)
    mod_in[i] = dc_in_.Process<FILTER_MODE_HIGH_PASS>(mod_in[i]);

  // 2. Carrier source.
  //    shape == 0 : carrier_in already holds IN1 audio (filled by caller).
  //    shape != 0 : overwrite carrier_in with internal saw/square/noise,
  //                 V/oct from note_ (which embeds LEVEL 1 pot + CV).
  if (carrier_shape_ != 0) {
    osc_.Render(internal_shape_, note_, /*fm=*/nullptr, carrier_in, size);
  }

  // 3. Envelope follow modulator (scalar, asymmetric one-pole). Multiply
  //    carrier by envelope. This IS the buzz-modulator core.
  float env = envelope_;
  for (size_t i = 0; i < size; ++i) {
    float rect = fabsf(mod_in[i]);
    float coeff = (rect > env) ? attack_ : release_;
    env += (rect - env) * coeff;
    out_main[i] = carrier_in[i] * env;
  }
  envelope_ = env;

  // 4. Snapshot pre-crush wet for OUT2 (free alt output).
  for (size_t i = 0; i < size; ++i) out_aux[i] = out_main[i];

  // 5. SoftLimit before bitcrush so loud peaks don't quantize into harsh
  //    clipping at the rails.
  for (size_t i = 0; i < size; ++i) out_main[i] = stmlib::SoftLimit(out_main[i]);

  // 6. Bitcrush. bit_levels_ precomputed in set_bits — no powf in loop.
  for (size_t i = 0; i < size; ++i) {
    float x = out_main[i] * bit_levels_;
    int32_t q = static_cast<int32_t>(x + (x >= 0.0f ? 0.5f : -0.5f));
    out_main[i] = static_cast<float>(q) / bit_levels_;
  }

  // 7. Zero-order-hold SR reduce. Fractional accumulator, no integer-hold
  //    zipper on CV sweeps; aliasing is intentional (Roboto character).
  for (size_t i = 0; i < size; ++i) {
    sr_phase_ += sr_ratio_;
    if (sr_phase_ >= 1.0f) { sr_phase_ -= 1.0f; held_ = out_main[i]; }
    out_main[i] = held_;
  }

  // 8. DC block (bitcrush + S&H can introduce offset).
  for (size_t i = 0; i < size; ++i)
    out_main[i] = dc_out_.Process<FILTER_MODE_HIGH_PASS>(out_main[i]);

  // 9. Smoothed dry/wet crossfade. mod_in is post-DC-block — good dry path.
  stmlib::ParameterInterpolator mix(&mix_, mix_target_, size);
  for (size_t i = 0; i < size; ++i) {
    float m = mix.Next();
    out_main[i] = mod_in[i] + (out_main[i] - mod_in[i]) * m;
  }
}
```

### 4.4 `ProcessRoboto` in `modulator.cc`

Mirrors `ProcessFormantShifter` / `ProcessPhaser`, with one twist: the
carrier-path VCA only fires when the carrier is external. When the carrier
is internal, LEVEL 1 CV is consumed upstream by cv_scaler as pitch (via
`parameters_.note`) and applying it again as a VCA would double-dip.

```cpp
void Modulator::ProcessRoboto(ShortFrame* input, ShortFrame* output, size_t size) {
  float* carrier     = buffer_[0];   // IN1 — external carrier audio (when shape==0)
  float* modulator   = buffer_[1];   // IN2 — voice/source
  float* main_output = buffer_[0];
  float* aux_output  = buffer_[2];

  int32_t shape = parameters_.carrier_shape;
  CONSTRAIN(shape, 0, 3);

  if (shape == 0) {
    // External carrier: both channels VCA'd by LEVEL CVs (standard FX idiom).
    ApplyAmplification(input, parameters_.raw_level_cv, aux_output, size, true);
  } else {
    // Internal carrier: only the modulator path is VCA'd.
    // raw_level_cv[1] still gates the modulator; the carrier is generated at
    // unity inside Roboto::Process via osc_.Render. LEVEL 1 CV's only
    // consumer is parameters_.note (1V/oct pitch).
    float level[2] = { 0.0f, parameters_.raw_level_cv[1] };
    ApplyAmplification(input, level, aux_output, size, true);
  }

  roboto.set_bits(previous_parameters_.modulation_parameter);   // MOD → bits
  roboto.set_mix(previous_parameters_.raw_level_pot[1]);        // LEVEL2 pot → wet
  roboto.set_sr_hold(previous_parameters_.raw_algorithm);       // ALGO → SR-reduce
  if (shape == 0) {
    roboto.set_response(previous_parameters_.raw_level_pot[0]); // LEVEL1 pot → env response
  } // else: response stays at default; LEVEL1 pot is baked into note already.
  roboto.set_carrier_shape(shape);
  roboto.set_note(previous_parameters_.note);

  roboto.Process(carrier, modulator, main_output, aux_output, size);

  Convert(output, main_output, aux_output, 32768.0f, size);
  previous_parameters_ = parameters_;
}
```

Uses `previous_parameters_` for control values (matches phaser/formant_shifter
idiom — keeps the values coherent with the rendered block).

### 4.5 Dispatch + reset

- Add `FEATURE_MODE_ROBOTO` to the enum in [parameters.h](../dsp/parameters.h).
- Add dispatch case in `Modulator::Process()` (~line 933) alongside the other FX.
- Roboto holds no large delay buffer, just envelope + S&H + osc-phase state.
  Add to the `is_fx` set in `set_feature_mode()` so `reset_fx` zeros the
  envelope and held sample on mode switch — otherwise a stale envelope can
  hold the output at non-zero when the mode is freshly entered.

---

## 5. CPU & RAM budget (at 48 kHz internal FX rate)

Per-sample cost is dominated by the bitcrush divide and the SoftLimit:

| Stage | Cycles/sample (est.) |
|---|---|
| DC block in (OnePole HPF) | ~5 |
| Carrier render (when internal; PolyBLEP saw/sq via `Oscillator`) | ~15 |
| Rectify + envelope follow + carrier multiply | ~8 |
| SoftLimit | ~6 |
| Bitcrush (multiply + round + divide) | ~10 |
| SR-reduce (ZOH, fractional) | ~4 |
| DC block out | ~5 |
| Dry/wet crossfade | ~3 |
| **Total** | **~56 cycles/sample, ~70 with internal carrier** |

At 48 kHz internal FX rate and 168 MHz CPU, ~70 cycles/sample is ~2% of
available CPU — well clear of the FX-mode budget and ~4× cheaper than the
20-band path. Profile after Phase 1 to confirm.

RAM:
- One `Oscillator` instance, two `OnePole`s, one envelope scalar, half a
  dozen floats. Total well under 1 KB. No shared FX buffer, no delay lines,
  no filterbank.
- This is one of the cheapest Symbiote FX modes by RAM.

---

## 6. Implementation phases

### Phase 1 — Buzz core

1. Add `FEATURE_MODE_ROBOTO`, dispatch case, UI mode entry.
2. `Roboto::Init` + scalar envelope follower + carrier multiply + carrier
   sourcing (external IN1 when `shape==0`, internal saw/square/noise via
   `osc_.Render` when `shape!=0`, V/oct from `parameters_.note`).
3. Raw output (no lo-fi stages), dry/wet hard-wired to 1.0.
4. Verify: voice → IN2, shape=1 (internal saw) → recognisable robot voice
   that pitches with LEVEL 1 CV. Patch tracked osc into IN1 with shape=0 →
   external-carrier robot. shape=3 (noise) → whisper robot.

### Phase 2 — Lo-fi back end

1. Bitcrush with precomputed `bit_levels_` driven by MODULATION.
2. Fractional ZOH SR-reducer driven by ALGORITHM.
3. SoftLimit pre-crush, OnePole DC-block post-crush.
4. Tune ranges so 8-bit / ~6 kHz matches a Roboto recording.

### Phase 3 — Control polish

1. Dry/wet on LEVEL 2 with `ParameterInterpolator` smoothing.
2. Envelope response on LEVEL 1 pot (shape==0 only). Tune the
   attack/release sweep curve by ear — should go from "smooth swell" at
   slow end to "choppy gating" at fast end.
3. Verify zipper-free CV sweeps on all four axes.

### Phase 4 — Edge cases

1. Self-patch feedback (OUT1 → IN1 via attenuator, shape=0) — document.
2. Extreme settings: 3-bit + minimum SR + sub-30 Hz internal carrier.
3. Reset behaviour on mode switch (envelope and held sample should zero).
4. CPU profile vs. stock vocoder mode (should be ~4× cheaper).

---

## 7. Validation patches

| Patch | Expected |
|-------|----------|
| Voice → IN2, nothing in IN1, shape=1 (saw), note ≈ 120 Hz | Classic robot voice |
| Voice → IN2, sequenced VCO → IN1, shape=0 | Singing robot, pitch follows sequence |
| Voice → IN2, shape=3 (noise) | Whispered robot |
| Drum loop → IN2, shape=1, low SR, 3-bit | Aggressively destroyed drum loop |
| Pad → IN2, shape=2 (square), high SR, 8-bit | Subtle "telephone-radio" colour |
| OUT1 → attenuator → IN1, shape=0 | Self-modulating feedback growl |

---

## 8. Departures from the original draft worth flagging in review

- **Topology changed: 1-band buzz, not multi-band vocoder.** Faithful to
  HT8950 (which is not a vocoder), ~4× cheaper CPU, makes the carrier-shape
  switch meaningful, simpler code path.
- Six-band custom biquad filterbank → dropped entirely. No `FilterBank`, no
  `Vocoder`, no per-band gain smoothing.
- `TIMBRE/MODULATION/WET-DRY/IN A LEVEL` → actual Warps controls
  (`LEVEL1/2 pot+CV`, `ALGORITHM`, `MODULATION`, `carrier_shape` switch).
- LEVEL 1 is dual-purpose by `carrier_shape`: VCA+envelope-response when
  external, 1V/oct pitch when internal. Leans on cv_scaler's existing
  LEVEL 1 → `note` wiring instead of fighting it.
- Carrier-source detection via `carrier_shape == 0` (stock semantic) rather
  than ad-hoc jack-detect.
- Envelope follower uses the `error > 0 ? attack : decay` form (the
  original draft's `env += ... ; if (rect < env) env += ...` adds both
  branches and shortens release).
- Bitcrusher uses precomputed `levels` + integer round, no `powf` /
  `roundf` in the hot loop.
- SR-reducer is a fractional ZOH accumulator (no integer-hold zipper on CV
  sweeps; aliasing is intentional Roboto character).
- CPU budget reframed at 48 kHz FX-mode rate; ~70 cycles/sample (~2% load).
- `ParameterInterpolator` for dry/wet smoothing across the block, matching
  surrounding FX modes' idiom.
