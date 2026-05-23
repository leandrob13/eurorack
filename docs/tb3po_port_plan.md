# Marbles Custom Firmware: TB-3PO Port Plan

This document outlines the plan to port the TB-3PO acid sequencer from its
original O&C/Hemisphere applet form into Marbles, where it runs on the X-section
whenever Grids mode is active on the T-section.

See [plan.md](plan.md) for the Grids T-section port that this builds on, and
[../marbles/docs/MEMORY.md](../marbles/docs/MEMORY.md) for what shipped with
Grids mode.

---

## 1. Goal

When `state.t_model == T_GENERATOR_MODEL_GRIDS`:

- The **T-section** continues to run the Grids drum / Euclidean engine as
  already implemented.
- The **X-section** is replaced by a TB-3PO style generative acid sequencer.
  TB-3PO is driven by the same master clock as Grids, so the drums and
  bassline are always musically locked.

When Grids is not active, the X-section behaves normally (random CV through
the scale quantizer) — the TB-3PO sequencer is dormant.

---

## 2. Source code situation

The TB-3PO source at [`marbles/tb3po/tb3po.h`](../marbles/tb3po/tb3po.h) is an
O&C / Hemisphere applet. It inherits `HemisphereApplet` and depends on:

- `HS::QuantizerLookup`, `HS::GetScale`, `OC::Scales::GetScale`, `OC::Strings`
- `Clock()`, `Gate()`, `SemitoneIn()`, `DetentedIn()`, `Out()`
- `gfx*` icons / cursor / text rendering
- Arduino `random()`, `randomSeed()`, `micros()`
- `PackLocation` / `Pack` / `Unpack` for preset I/O
- `CursorToggle`, `MoveCursor`, `EditMode`, `CancelEdit`
- `OC::CORE::ticks` for gate-off timing

**None of these exist in Marbles.** The class cannot be dropped in as-is.
Only the algorithm is reusable, ported into a plain C++ class.

---

## 3. Architectural overview

```
                          ┌─────────────────────────────────────┐
                          │   t_generator (Grids mode)          │
                          │                                     │
   T clock / patched ────▶│   PatternGenerator::TickClock()     │
                          │   grids_pulse_ (0..5)               │
                          │   ramps.master  (step-level ramp)   │
                          │                                     │
                          │   T1=BD, T2=SD, T3=HH   gates       │
                          └──────────────┬──────────────────────┘
                                         │
                                         ▼
                          ┌─────────────────────────────────────┐
                          │   marbles.cc Process() (Grids mode) │
                          │                                     │
                          │   tb3po.Tick(reset) on every        │
                          │   master_phase_ wrap                │
                          │                                     │
                          │   per-sample slide IIR              │
                          └──────────────┬──────────────────────┘
                                         │
        ┌─────────────────────┬──────────┴───────────┬───────────────────┐
        ▼                     ▼                      ▼                   ▼
   X1 = clock           X2 = pitch CV           X3 = gate           Y  = accent
   (5V/0V from          (1V/oct,                (5V/0V,             (5V/0V,
    ramp_buffer)         scale-quantized,       held through        co-incident
                         slide-slewed)          slides)             with gated
                                                                    accent steps)
```

`xy_generator.Process()` is **still called** for Marbles' X-section so its
ramp_extractor and random_sequence state stay coherent across mode switches,
but its X2/X3/Y outputs are overwritten before the DAC write loop.

---

## 4. File plan

| File | Action |
|---|---|
| `marbles/tb3po/tb3po_sequencer.h`  (NEW) | Algorithm-only class. ~80 LOC header. |
| `marbles/tb3po/tb3po_sequencer.cc` (NEW) | ~250 LOC implementation. |
| `marbles/tb3po/tb3po.h`              | DELETE after port (Phase 5). |
| `marbles/random/t_generator.{h,cc}`  | Expose `grids_pulse_` wrap signal so marbles.cc knows when to tick TB-3PO. |
| `marbles/marbles.cc`                 | Instantiate `TB3PoSequencer`, drive it from X knobs/CVs, overwrite X2/X3/Y outputs in `Process()`. Detect `x_deja_vu` edges, write `state.tb3po_seed`, call `settings.SaveState()` on lock edge. Seed the sequencer from `state.tb3po_seed` at boot. |
| `marbles/settings.h`                 | Reuse 2 bytes of `State::padding` for `uint16_t tb3po_seed`. |
| `marbles/settings.cc`                | Default `state_.tb3po_seed = 0` in `Settings::Init()`. |
| `marbles/makefile`                   | Confirm `marbles/tb3po` is in `PACKAGES`. |
| `marbles/docs/MEMORY.md`             | Append a TB-3PO section after the port lands. |

---

## 5. `TB3PoSequencer` API

```cpp
namespace marbles {

class TB3PoSequencer {
 public:
  static constexpr int kMaxSteps = 16;

  void Init();

  // Live parameters (called every block from marbles.cc):
  void set_density(int encoder, int cv);      // encoder 0..14, cv -7..+7
  void set_transpose(float scale_degrees);    // bipolar, knob + CV combined
  void set_length(int steps);                 // 1..16
  void set_lock_seed(bool locked);            // from state.x_deja_vu
  void set_scale_index(int idx);              // from state.x_scale

  // Persistence:
  uint16_t seed() const { return seed_; }
  void set_seed(uint16_t s);
  void Reseed();   // draw a new random seed and regenerate; called by
                   // marbles.cc on the x_deja_vu ON|LOCKED → OFF edge

  // Clock:
  void Tick(bool reset);                      // call once per Grids step boundary
  void TickHalfCycle();                       // call at half-step (for gate-off)

  // Per-sample slide IIR (called from audio loop):
  void StepSlide();                           // advance one sample
  float pitch_volts() const { return pitch_volts_; }
  bool  gate() const        { return gate_; }
  bool  accent() const      { return accent_; }

 private:
  // Pattern data
  uint16_t seed_;
  bool     lock_seed_;
  uint8_t  num_steps_;
  uint8_t  current_pattern_density_;
  uint8_t  current_pattern_scale_size_;

  uint32_t gates_;     // bitfield per step
  uint32_t slides_;
  uint32_t accents_;
  uint32_t oct_ups_;
  uint32_t oct_downs_;
  uint8_t  notes_[kMaxSteps];

  // Live params
  int   density_encoder_;
  int   density_cv_;
  int   density_;          // derived = clamp(encoder + cv, 0, 14)
  float transpose_;
  int   scale_index_;
  uint8_t scale_size_;

  // Playback
  uint8_t step_;
  bool    reset_flag_;
  bool    gate_;
  bool    accent_;
  bool    gate_off_pending_;

  // Slide IIR
  float pitch_volts_;
  float slide_target_;
  float slide_start_;

  void Reseed();
  void RegenerateIfDirty();
  void RegeneratePitches();
  void ApplyDensity();
  int  GetOnOffDensity() const;
  int  GetPitchChangeDensity() const;
  bool RandBit(int prob);
  int  GetNextStep(int step) const;
  bool StepIsGated(int s) const   { return gates_   & (1u << s); }
  bool StepIsSlid(int s) const    { return slides_  & (1u << s); }
  bool StepIsAccent(int s) const  { return accents_ & (1u << s); }
  bool StepIsOctUp(int s) const   { return oct_ups_ & (1u << s); }
  bool StepIsOctDown(int s) const { return oct_downs_ & (1u << s); }
  float PitchForStep(int s) const;
};

}  // namespace marbles
```

### Algorithm carry-over

Kept verbatim (translated from O&C int CV codes to float volts):

- `RegeneratePitches` — pitch-density-driven scale-degree generation with
  consecutive-note repeat suppression and octave-jump bitfields.
- `ApplyDensity` — gate / slide / accent bitfields with consecutive-bit
  probability shaping (accents and slides are less likely after themselves).
- `GetOnOffDensity` / `GetPitchChangeDensity` — density curve shaping
  (bipolar around 7).
- `PitchForStep` — scale-degree + transpose + octave-up/down → scale lookup.

Dropped (deferred to v2 if needed):

- `density_auto[]` motion recording
- `transpose_in_semitones` toggle (always degrees in v1)
- `hold_pitch` toggle (always on in v1 — gate off doesn't drop pitch)
- `no_slides` toggle
- 4-digit hex-seed manual edit
- All `View / DrawGraphics / OnEncoderMove / OnButtonPress / AuxButton`

Replaced:

- `random()` / `randomSeed()` → [`marbles::grids::GridsRandom`](../marbles/grids/grids_random.h)
- `HS::QuantizerLookup(qselect, note)` → direct read of
  `settings.persistent_data().scale[state.x_scale]` with scale-degree +
  octave indexing
- `OC::CORE::ticks` → `ramps.master` phase (gate off when phase ≥ 0.5)

---

## 6. Marbles X-side wiring

All changes in [`marbles/marbles.cc`](../marbles/marbles.cc) `Process()`.

### Block-level parameter feed

```cpp
if (grids_mode) {
  int dens_enc = int(roundf(parameters[ADC_CHANNEL_X_SPREAD] * 14.0f));
  int dens_cv  = int(roundf(cv_reader.channel(ADC_CHANNEL_X_SPREAD).cv() * 7.0f));
  tb3po.set_density(dens_enc, dens_cv);

  float transpose = (parameters[ADC_CHANNEL_X_BIAS] - 0.5f) * 24.0f;  // ±12 degrees
  tb3po.set_transpose(transpose);

  int len = 1 + int(parameters[ADC_CHANNEL_X_STEPS] * 15.0f);  // 1..16
  tb3po.set_length(len);

  tb3po.set_lock_seed(state.x_deja_vu != DEJA_VU_OFF);
  tb3po.set_scale_index(state.x_scale);
}
```

### Reset trigger (X STEPS CV rising edge)

Resets **both** sections so drums and bassline restart together:

```cpp
bool x_steps_reset =
    grids_mode &&
    (hidden_gates[ADC_CHANNEL_X_STEPS] & GATE_FLAG_RISING);

if (x_steps_reset) {
  t_section_reset = true;        // Grids step → 0, grids_pulse_ → 0
  // tb3po reset handled below via Tick(reset=true)
}
```

### Step tick

`grids_pulse_` (currently in `t_generator`) wraps every Grids step. Expose
a single-shot "tick" signal to marbles.cc:

```cpp
// In marbles.cc Process(), per sample:
if (t_generator.grids_pulse_wrapped()) {
  tb3po.Tick(/*reset=*/x_steps_reset && (i == 0));
}
tb3po.StepSlide();
```

### DAC overwrite

```cpp
for (size_t i = 0; i < size; ++i) {
  float x1 = grids_mode ? (ramp_buffer[i] < 0.5f ? 5.0f : 0.0f) : *v;
  v++;

  float x2 = grids_mode ? tb3po.pitch_volts()       : *v;  v++;
  float x3 = grids_mode ? (tb3po.gate() ? 5.0f : 0.0f) : *v;  v++;
  float y  = grids_mode ? (tb3po.accent() ? 5.0f : 0.0f) : *v;  v++;

  block->cv_output[1][i] = DacCode(1, x1);
  block->cv_output[2][i] = DacCode(2, x2);
  block->cv_output[3][i] = DacCode(3, x3);
  block->cv_output[0][i] = DacCode(0, y);
  // gate outputs unchanged (Grids drums)
}
```

> **Note:** X2 pitch ignores `state.x_range` (always 1V/oct full range). X3
> gate and Y accent are fixed 0V/5V.

---

## 7. Control mapping

| Marbles control | TB-3PO parameter | Mapping |
|---|---|---|
| X SPREAD knob | `density_encoder` | `round(spread * 14)` → 0..14 (treated as -7..+7) |
| X SPREAD CV  | `density_cv`      | `round(cv * 7)` → -7..+7 offset |
| X BIAS knob | `transpose` (knob) | `HysteresisQuantizer2` → ±active_count scale degrees (= ±1 octave on the current scale). Stepped, no glide. |
| X BIAS CV   | `transpose` (CV)   | Same quantizer treatment, summed with the knob value. Combined range is ±2 octaves stepped. |
| LENGTH knob (Deja Vu Length) | `num_steps` | Shared with Grids Euclidean length — keeps drums and bassline loops aligned. Uses the existing `deja_vu_length_quantizer.Lookup(loop_length, …)` → 1..16. |
| X STEPS CV (rising edge) | clock reset (T + X) | Resets both sequencers to step 0 |
| X DEJA VU switch | `lock_seed`   | Transition `ON|LOCKED → OFF` → reseed once; otherwise seed is held (locked while `ON|LOCKED`, frozen at last value while `OFF`). Tap OFF again to draw another seed. |
| X SCALE (existing X selector) | scale lookup | reuses `state.x_scale` |
| X STEPS knob | _unused in Grids mode_ | only X STEPS CV is consumed (as reset) |
| X RANGE switch | _unused on X2 in Grids mode_ | pitch is always 1V/oct |

---

## 8. Output mapping

| Output | Voltage | Source |
|---|---|---|
| **X1** | 5V / 0V square wave | Existing Grids clock (`ramp_buffer[i] < 0.5f`) |
| **X2** | 1V/oct, slewed | `tb3po.pitch_volts()` |
| **X3** | 5V / 0V | `tb3po.gate()` — high while step is gated; held through slides |
| **Y**  | 5V / 0V | `tb3po.accent()` — high when accent ∧ gated |
| t1 (gate) | drum | BD (unchanged) |
| t2 (master gate) | drum | SD (unchanged) |
| t3 (gate) | drum | HH (unchanged) |

---

## 9. Seed persistence

### Storage layout

- Reuse 2 of the 5 padding bytes in `State` for `uint16_t tb3po_seed;`. State
  is already persisted by Marbles' `chunk_storage_` to flash — the seed rides
  along automatically with the existing save/load mechanism.
- Add a default of `0` for `state_.tb3po_seed` in
  [`marbles/settings.cc Settings::Init()`](../marbles/settings.cc) alongside
  the other state defaults, so first-boot / corrupted-flash recovery is well
  defined.

### Runtime behaviour

- **Reseed only on the `x_deja_vu` transition to `OFF`** (edge-triggered, not
  level-triggered). marbles.cc tracks the previous switch value; when it goes
  from `ON|LOCKED → OFF`, draw a new 16-bit seed via `GridsRandom`, write it
  to `settings.mutable_state()->tb3po_seed`, push it to `tb3po.set_seed()`,
  and regenerate the pattern.
- While `x_deja_vu == OFF` is held, the seed is **not** redrawn on pattern
  wraps. The same pattern repeats until you either toggle OFF again (new
  seed) or flip ON|LOCKED (locks the current seed). This lets you audition a
  pattern and lock it once you like it.
- While `x_deja_vu == ON | LOCKED`: `tb3po_seed` is held. On a parameter
  change that affects pattern shape (density / scale size) regenerate using
  the same seed for deterministic shaping.

### Save-to-flash trigger

- **On the `x_deja_vu OFF → ON|LOCKED` edge**, call
  `settings.SaveState()` immediately. This is the moment the user has
  committed to the current pattern, and the locked seed must survive a power
  cycle. This is a new call site — add it in marbles.cc next to the edge
  detector.
- We deliberately do **not** save on every OFF-tap reseed. Auditioning may
  flip the switch many times before committing; saving each time would beat
  on the flash. The OFF-state seed only lives in RAM `state_` between
  power-ups — losing it on an accidental power-cycle is acceptable since the
  user hadn't committed to it yet.
- The existing save paths (calibration, scale edits, other state edits) also
  pick up `tb3po_seed` naturally because it's part of `State`.

### Boot flow

1. `Settings::Init()` calls `chunk_storage_.Init(&persistent_data_, &state_)`
   which **loads `state_` from flash**, including `state_.tb3po_seed` and
   `state_.x_deja_vu`.
2. `TB3PoSequencer::Init()` then takes `state.tb3po_seed` and
   `state.x_deja_vu` from the loaded state:
   - Seeds the generator with `state.tb3po_seed`.
   - Sets `lock_seed_` from `state.x_deja_vu != DEJA_VU_OFF`.
   - Generates the initial pattern from that seed (deterministic — same
     seed always yields the same pattern given the same scale/density).
3. marbles.cc initialises `prev_x_deja_vu_ = state.x_deja_vu` so no spurious
   reseed fires on the first audio block.

**Result**: if `x_deja_vu` was `ON|LOCKED` at power-down, the locked seed is
in flash; at next power-up it is loaded into `state_`, the sequencer is
seeded with it, and the same pattern plays as before. If `x_deja_vu` was
`OFF` at power-down, the last seed used is still in `state_` (whatever was
saved during the last commit / state save), so the user resumes on a known
pattern rather than getting random noise on boot.

---

## 10. Slide IIR

TB-3PO's original slide uses a fixed-point IIR at the O&C CV update rate
(~16.6 kHz):

```cpp
int x = slide_end_cv - curr_pitch_cv;
x >>= 18;
x *= 3;            // k = 0x3
curr_pitch_cv += x;
```

Float translation at Marbles' audio rate (~32 kHz block-level):

```cpp
constexpr float kSlideCoef = 0.003f;  // tune by ear; ~25 ms time constant
pitch_volts_ += kSlideCoef * (slide_target_ - pitch_volts_);
// Clamp to keep direction monotonic, matching TB-3PO's CONSTRAIN
if (slide_start_ < slide_target_)
  pitch_volts_ = std::min(pitch_volts_, slide_target_);
else
  pitch_volts_ = std::max(pitch_volts_, slide_target_);
```

`kSlideCoef` to be tuned on hardware against a reference 303 / TB-3PO recording.

---

## 11. Implementation phases

| # | Phase | Files | Verification |
|---|------|---|---|
| 1 | `TB3PoSequencer` core class — algorithm only, no I/O integration | `marbles/tb3po/tb3po_sequencer.{h,cc}` NEW | Build clean; ideally unit-test pattern generation determinism |
| 2 | Wire into `marbles.cc Process()`: feed knobs/CVs, write X2/X3/Y; expose `grids_pulse_wrapped()` from `t_generator` | `marbles/marbles.cc`, `marbles/random/t_generator.{h,cc}` | X2 produces stepped pitch in scale, X3 fires gates, Y fires accents in time with drums |
| 3 | Slide IIR per-sample slew between step pitch targets | `marbles/marbles.cc` (audio loop) | Audible 303-style portamento on slid steps |
| 4 | X STEPS CV → clock reset for both T and X | `marbles/marbles.cc` | External trigger restarts both drums and bass at step 0 |
| 5 | Seed persistence + `x_deja_vu` edge-triggered reseed; 2 padding bytes in `State`; default in `Settings::Init()`; `SaveState()` call on lock edge; sequencer seeded from `state.tb3po_seed` at boot | `marbles/settings.h`, `marbles/settings.cc`, `marbles/marbles.cc` | Lock → power cycle → **same locked pattern returns**; tap OFF → new pattern; tap OFF again → another new pattern; flip to ON|LOCKED → current pattern frozen and committed to flash |
| 6 | Cleanup: delete `marbles/tb3po/tb3po.h`, update `marbles/docs/MEMORY.md` | — | — |

---

## 12. Risks / open items

- **`xy_generator.Process()` left running but outputs discarded** — costs the
  ~40% CPU mentioned in `Process()` comments. If we hit timing problems,
  short-circuit the X branch inside `xy_generator` when `clock_source` plus a
  mode flag indicate Grids+TB-3PO. Defer until measured.
- **Slide coefficient** — `0.003f` is a starting guess. Adjust on hardware.
- **Scale degree count** — `regenerate_pitches` needs `scale_size`. In Marbles,
  this is `Scale::degree[]` length from `persistent_data().scale[x_scale]`.
  Confirm the type / accessor in `xy_generator.cc` when implementing Phase 1.
- **Hold pitch behavior** — TB-3PO can drop pitch CV between gated steps if
  `hold_pitch == false`. We default to `true` (pitch is held). If users want
  the alternative, that's a v2 toggle.
- **Edge detection for `x_deja_vu`** — marbles.cc must keep a `prev_x_deja_vu`
  field to detect the `ON|LOCKED → OFF` transition. Initial value at boot
  should match the loaded `state.x_deja_vu` so we don't reseed on startup.
