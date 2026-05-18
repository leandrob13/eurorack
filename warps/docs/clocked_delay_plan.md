# Clocked delay plan (FEATURE_MODE_DELAY)

Plan for adding external-clock sync to the existing `ProcessDelay` in [dsp/modulator.cc](../dsp/modulator.cc#L566-L781), using **TIMBRE CV as a clock input** and the **TIMBRE pot as a subdivision selector**. The current free-running tape/varispeed behaviour stays as the fallback when no clock is present.

## Verdict (TL;DR)

Feasible. Repurposing TIMBRE CV is the right jack choice: the TIMBRE pot remains available for subdivision selection (so the "quantized musical subdivisions" idea falls out naturally), `time` in `ProcessDelay` is already in samples so wiring in a clock-derived sample count is a one-line change, and the CV update rate (1.6 kHz) is fast enough for musical tempos.

Two real caveats:

1. The CV is heavily LP-filtered before it reaches `modulation_parameter` — for clock detection the raw ADC channel must be read directly.
2. At fast subdivisions the per-block sample cadence becomes an audible jitter floor; clocks should be treated as a **tempo reference** (BPM derivation + median filter), not as per-step events.

## Signal-path facts gathered during investigation

### Audio engine timing — [warps.cc:47](../warps.cc#L47), [warps.cc:104](../warps.cc#L104)
- `kSampleRate = 96000.0f` — note the original 48 k is commented out.
- `codec.Start(60, &FillBuffer)` → block size = 60 samples → `FillBuffer` fires every **625 µs (1600 Hz)**.

### CV pipeline — [warps.cc:72](../warps.cc#L72), [cv_scaler.cc:123-203](../cv_scaler.cc#L123-L203)
- `cv_scaler.Read()` runs once per audio block. CV update rate = **1600 Hz**.
- DMA-fed ADC with software-triggered conversion at the end of `Read()` ([drivers/adc.cc:109](../drivers/adc.cc#L109)).
- TIMBRE jack = `ADC_PARAMETER_CV`, enum index 3 ([drivers/adc.h:36-46](../drivers/adc.h#L36-L46)).

### The smoothing problem — [cv_scaler.cc:65-72](../cv_scaler.cc#L65-L72), [cv_scaler.cc:126](../cv_scaler.cc#L126)
- `BIND(p->modulation_parameter, PARAMETER, false, 2.0f, 0.08f, false)` applies a one-pole LP every block: `lp_state_[ADC_PARAMETER_CV] += 0.08 * (raw - lp_state_)` at 1600 Hz → **τ ≈ 7.8 ms**, ~22 ms to 95%.
- Fine for a knob, fatal for triggers — a 1 ms gate pulse will not cleanly clear a threshold after smoothing.
- **Fix:** for clock detection, bypass `lp_state_` and read `adc_.float_value(ADC_PARAMETER_CV)` directly — that's a single ADC sample updated each block.
- The `raw_modulation_cv` field is **also** derived from `lp_state_` ([cv_scaler.cc:152](../cv_scaler.cc#L152)), so it's smoothed too; only `adc_.float_value()` is raw.

### Polarity — [cv_scaler.cc:151](../cv_scaler.cc#L151)
- `raw_modulation_cv = (calibration_offset - lp_state_) * 2.0f`, constrained to [-1, 1]. Hardware inverts — positive input voltage → negative deflection from the calibration offset. Pick the right sign in the Schmitt logic; threshold detection is symmetric otherwise.

### Delay engine integration point — [dsp/modulator.cc:578-579](../dsp/modulator.cc#L578-L579)
- `time = modulation_parameter * (DELAY_SIZE-10) + 5;` — `time` is already in **samples**. Replacing the RHS with `subdivision_samples` is a drop-in.
- The inner-loop `ONE_POLE(lp_time, time, 0.00002f)` already smooths step changes (τ ≈ 31 ms at 96 k), so musical subdivision jumps won't click.
- `DELAY_SIZE` is computed from the reused buffer pool ([dsp/modulator.h:408-414](../dsp/modulator.h#L408-L414)) — needs to be measured once, then max useful subdivision is `bpm_samples * max_div ≤ DELAY_SIZE - 10`. Long subdivisions at slow tempos will clip — clamp before assigning.

### No normalization probe on TIMBRE CV — [cv_scaler.cc:77-97](../cv_scaler.cc#L77-L97), [cv_scaler.cc:188-198](../cv_scaler.cc#L188-L198)
- The probe-based jack-detection only runs for the two LEVEL channels (`for i = 0; i < 2`). PARAMETER CV has no automatic "is something plugged?" signal.
- Fallback to pot-only delay time needs a **timeout heuristic**: "no rising edges seen in the last 2 s → mark clock absent, restore pot-as-time behaviour."

## Timing budget

Worst-case ±1 sample jitter on edge timestamps = ±625 µs:

| Tempo   | 1/4 note  | Jitter vs 1/4 | 1/16 note | Jitter vs 1/16 |
|---------|-----------|---------------|-----------|----------------|
| 60 BPM  | 1000 ms   | 0.06%         | 250 ms    | 0.25%          |
| 120 BPM | 500 ms    | 0.13%         | 125 ms    | 0.50%          |
| 200 BPM | 300 ms    | 0.21%         | 75 ms     | 0.83%          |

Conclusion: fine for tempo-derived delay times across the musical range.

**Do not** treat each clock pulse as an immediate "step" event (24/48 PPQN-style). At 120 BPM and 24 PPQN, period is 2 ms and 625 µs is 31% jitter, audibly sloppy. Treat the clock as a tempo reference: measure interval between rising edges, apply a median-of-N filter (N=4-5 recent intervals), and recompute the delay-time target only when the BPM estimate moves more than ~1%.

## Concrete plan

### 1. Add a clock detector to `CvScaler`

Keep it isolated from the existing BIND pipeline so it doesn't pollute the per-block LP smoothing.

Suggested fields on `CvScaler` ([cv_scaler.h](../cv_scaler.h)):

```cpp
// Clock detection (TIMBRE CV repurposed as clock input).
float clock_prev_raw_;                  // last sample for Schmitt comparison
bool  clock_high_;                      // current Schmitt state
uint32_t clock_sample_counter_;         // monotonic block-tick counter (in samples)
uint32_t clock_last_edge_sample_;       // sample-index of most recent rising edge
uint32_t clock_intervals_[5];           // ring buffer of last 5 intervals (samples)
uint32_t clock_intervals_head_;
uint32_t clock_interval_samples_;       // median-filtered tempo, exposed to DSP
bool clock_present_;                    // false when no edges seen in 2 s
```

In `Read()`, *before* `DetectNormalization()` (so we don't disturb the existing audio-channel probe path), do something like:

```cpp
// Bypass lp_state_ for clock detection.
float raw = adc_.float_value(ADC_PARAMETER_CV);
float offset = calibration_data_->offset[ADC_PARAMETER_CV];

// +V at the jack → raw drops below offset (inverting front-end).
// Schmitt thresholds in raw-ADC units (≈ ±1 V hysteresis after calibration).
const float kThresholdHi = offset - 0.15f;  // tune from bench measurements
const float kThresholdLo = offset - 0.05f;

bool rising_edge = false;
if (clock_high_) {
  if (raw > kThresholdLo) clock_high_ = false;
} else {
  if (raw < kThresholdHi) { clock_high_ = true; rising_edge = true; }
}

clock_sample_counter_ += kBlockSize;   // 60 samples per Read() call

if (rising_edge) {
  uint32_t interval = clock_sample_counter_ - clock_last_edge_sample_;
  clock_last_edge_sample_ = clock_sample_counter_;

  // Reject pathological intervals (< 10 ms, > 8 s).
  if (interval >= 960 && interval <= 768000) {
    clock_intervals_[clock_intervals_head_] = interval;
    clock_intervals_head_ = (clock_intervals_head_ + 1) % 5;
    clock_interval_samples_ = MedianOf5(clock_intervals_);
    clock_present_ = true;
  }
}

// Stale-edge timeout: ~2 s of silence drops the clock.
if (clock_sample_counter_ - clock_last_edge_sample_ > 192000) {
  clock_present_ = false;
}
```

Expose to DSP via two new fields on `Parameters` ([dsp/parameters.h](../dsp/parameters.h)):

```cpp
uint32_t clock_interval_samples;   // 0 when no clock
bool clock_present;
```

…and set them in `Read()` before returning.

### 2. Quantize the TIMBRE pot to musical subdivisions

In `ProcessDelay` ([dsp/modulator.cc:566-781](../dsp/modulator.cc#L566-L781)), define the subdivision table once at file scope (top of `modulator.cc`):

```cpp
// Multipliers applied to one quarter-note's worth of samples.
// 1.0 = 1/4 note, 2.0 = 1/2 note, 0.5 = 1/8, etc.
static const float kClockSubdivisions[] = {
  4.0f,                                     // 1/1 (whole)
  3.0f,                                     // 1/2 dotted
  2.0f,                                     // 1/2
  1.5f,                                     // 1/4 dotted
  1.0f,                                     // 1/4
  2.0f / 3.0f,                              // 1/4 triplet
  0.5f,                                     // 1/8
  0.375f,                                   // 1/8 dotted
  1.0f / 3.0f,                              // 1/8 triplet
  0.25f,                                    // 1/16
  1.0f / 6.0f,                              // 1/16 triplet
  0.125f                                    // 1/32
};
static const int kNumClockSubdivisions = 12;
```

Replace the existing `time` / `time_end` lines at the top of `ProcessDelay`:

```cpp
float time, time_end;
if (parameters_.clock_present) {
  // Quantize TIMBRE pot to subdivision index. Add small hysteresis so the
  // index doesn't dither when the pot sits on a detent boundary.
  float pot = parameters_.raw_modulation_pot;
  int idx = static_cast<int>(pot * kNumClockSubdivisions);
  CONSTRAIN(idx, 0, kNumClockSubdivisions - 1);

  float quarter = static_cast<float>(parameters_.clock_interval_samples);
  float target = quarter * kClockSubdivisions[idx];
  CONSTRAIN(target, 5.0f, static_cast<float>(DELAY_SIZE - 10));

  // No interpolation needed across the block when clock-locked — let the
  // existing ONE_POLE(lp_time, time, 0.00002f) smooth the step.
  time = time_end = target;
} else {
  time     = previous_parameters_.modulation_parameter * (DELAY_SIZE - 10) + 5;
  time_end =          parameters_.modulation_parameter * (DELAY_SIZE - 10) + 5;
}
float time_increment = (time_end - time) / static_cast<float>(size);
```

`raw_modulation_pot` ([cv_scaler.cc:148](../cv_scaler.cc#L148)) is the pot value alone — exactly what's wanted when the CV is now a clock.

### 3. Hysteresis on subdivision selection

To avoid zipper artefacts when the pot rests on a subdivision boundary, only advance the stored `subdivision_index_` when the pot has moved more than ~1/(2·kNumClockSubdivisions) past the current cell boundary. One field on `Modulator`:

```cpp
int delay_subdivision_index_;  // initialised to closest cell at mode entry
```

Update it inside `ProcessDelay` only when the raw cell index has been off by ≥1 for ≥2 blocks. Cheap to implement; massive improvement in feel.

### 4. Visual feedback (nice-to-have)

The mode LED already blinks per `feature_mode_` selection. Adding a tempo-locked blink (toggle every `clock_interval_samples / 2`) gives the user confidence the clock has been captured. Implement in `ui.cc` using the existing block tick.

### 5. Carrier-shape voicings stay as-is

`carrier_shape` selects between clean / tape-hiss / open-feedback / ping-pong in `ProcessDelay` — keep these. Tape-hiss + clocked delay is a particularly musical combination.

## Open questions to resolve before implementation

1. **Threshold calibration.** Assumed +5 V triggers create a ~1 V deflection from offset (so `kThresholdHi = offset - 0.15`). Patch a trigger source in, log `adc_.float_value(ADC_PARAMETER_CV)` over time, tune from real measurements. The PARAMETER CV front-end gain may differ from LEVEL.
2. **Input bandwidth / RC.** If the front-end has an aggressive anti-aliasing RC, fast 1 ms gates may be slewed below threshold after the input filter. Worth scoping the jack or bench-testing with a known good trigger source (Pamela's, Marbles, etc.) before committing to the threshold values.
3. **`DELAY_SIZE` numerical value.** Compute it once. The slowest sensible tempo at the longest subdivision will exceed the buffer (e.g. 40 BPM × 1/1 = 6 s × 96 k = 576 k samples, almost certainly > `DELAY_SIZE`). Decide whether to (a) clamp + flash a warning LED or (b) drop to the next-shorter subdivision automatically.
4. **Pot + CV ergonomics.** Currently the pot and CV sum into `modulation_parameter`. With this change the pot becomes "subdivision" and the jack becomes "clock" — they no longer interact. That's the natural model, but worth confirming with a few patches before shipping.
5. **First-edge behaviour.** Before two edges have arrived, `clock_interval_samples_` is undefined. Either initialise it to a reasonable default (120 BPM = 48000 samples at 96 k) or treat `clock_present_` as false until the second edge.
6. **Tempo halving / doubling.** A common pitfall: if a sequencer emits 2 ppq instead of 1 ppq, the subdivision table will sound half-time. Consider exposing a clock-multiplier via `carrier_shape` (×1, ×2, ÷2, ÷4) as an alternative to using `carrier_shape` for voicing — or document the assumption clearly.

## Validation strategy

- **Phase 0 — bench test threshold:** add a temporary debug-print of `adc_.float_value(ADC_PARAMETER_CV)` to the easter-egg path, patch in known triggers, confirm thresholds are clean. No firmware behaviour changes.
- **Phase 1 — host harness:** extend `test/warps_test.cc` with a synthetic clock test (square wave on the PARAMETER CV input channel at the host harness layer), assert that `clock_interval_samples_` converges to the expected sample count within 3-5 edges and that the median filter rejects a single bad interval.
- **Phase 2 — hardware audition:** flash, patch a stable clock source, hold a recognisable rhythm in the buffer, sweep TIMBRE pot through subdivisions, listen for clicks at boundaries (validates the hysteresis logic).
- **Phase 3 — drift / dropout:** stop the clock mid-playback, confirm pot-driven fallback engages after 2 s without audio glitch on the transition.

## Source-of-truth ordering

When this doc disagrees with the C++:

1. The C++ in `warps/` wins.
2. This file is the design intent — update it when shipped behaviour diverges.
3. Cross-reference [CLAUDE.md](../CLAUDE.md) for module-wide FX-mode conventions (Convert layout, `previous_parameters_` for reverb-style control reads, `reset_fx` semantics).
