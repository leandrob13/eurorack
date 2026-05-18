# Rings — `Delay` crunchiness fix & tuning plan

Status: planning. Owner: Leandro. Target: [rings/dsp/fx/delay.h](../dsp/fx/delay.h), consumed by `ChordStringSynth` (easter egg) under `fx_type_ == DELAY` and `DELAY_2`.

## Goal

Stop the audible crunch / fizz in the existing delay and give `delay_time` a musical mapping. The class structure (`Delay`, `DelayLine<T,max>`, `DelayLine16Bits`) can stay; the changes are inside the three `Process*` methods, plus a small extension to the parameter setters and a remap of the time curve.

## Reference: what's there now

`Delay` ships three rendering paths used by the easter egg:

| Method | Storage backend | Read | Feedback path | Bus mode | Wired to |
|---|---|---|---|---|---|
| `Process` | `FxEngine<32768, FORMAT_16_BIT>` (q15) with two `Reserve<16383>` lines | `c.Interpolate(..., LFO_1, 0.5f, 0.5f)` — fractional read, ±0.5 sample LFO mod | `del_out * feedback_` (bare multiply) | True stereo: `line_l`/`line_r` independent | `fx_type_ == DELAY` ([chord_string_synth.cc:188-194](../dsp/chord_string_synth.cc#L188-L194)) |
| `Process2` | `DelayLine<uint16_t, 16384>` (q15, manual `Compress`/`Decompress`) | `Read(int32_t)` — **integer**, no interpolation | bare multiply | Mono sum L+R → write to both | not currently dispatched |
| `Process3` | `FxEngine<32768, FORMAT_16_BIT>` with one `Reserve<24576>` line | `c.Interpolate(..., LFO_2, 0.5f, 0.5f)` | bare multiply | Mono sum L+R → write to both | `fx_type_ == DELAY_2` ([chord_string_synth.cc:195-200](../dsp/chord_string_synth.cc#L195-L200)) |

Confirmed via [rings/dsp/fx/fx_engine.h:241-258](../dsp/fx/fx_engine.h#L241-L258): the second `Interpolate` overload shifts the read offset by `amplitude * lfo_value_[index]`. With `amplitude = 0.5` and `lfo_value` in `[-1, 1]`, that's **±0.5 samples** of LFO modulation — sub-sample dither, not audible wow. Crucially, that's not enough to mask zippering when `delay_time_` is swept.

LFO rates set in `Delay::Init`: LFO_1 at `0.3 / 48000`, LFO_2 at `0.05 / 48000` — both fine, the depth is the issue.

## Root-cause analysis of the crunch

Ranked by perceived audibility on a typical `feedback_ ≈ 0.6–0.9` setting:

1. **No slew on `delay_time_`.** `set_delay_time(synth.delay_time)` is called once per block, and inside every `Process*` the read offset is recomputed every *sample* as `reserved * delay_time_`. Any knob/CV change instantly jumps the read pointer by hundreds of samples → audible click on every knob nudge, continuous zipper noise on CV sweeps. The LFO depth (0.5 samples) cannot mask jumps that large.
2. **q15 storage compounds with feedback.** Both `FORMAT_16_BIT` in `FxEngine` and `Compress`/`Decompress` in `Process2` round-trip through int16. Quantization noise floor climbs by a few dB per repeat at high feedback — what starts "warm" decays into "grainy" and ends as "fizz". Standard fix: roll off highs in the feedback loop so each repeat is slightly darker, masking the rising noise floor with reduced bandwidth.
3. **Bare feedback path.** Three problems in one line (`c.Load(del_out * feedback_)`):
   - No soft-clip → hard-clips abruptly when feedback × hot input exceeds 1.0.
   - No DC blocker → any input offset accumulates in the feedback ring, biases q15 toward one rail, eats headroom.
   - No tone shaping → high-frequency content survives every repeat at full level (the classic "harsh digital delay" character).
4. **`Process2` reads with the integer overload** ([delay.h:85-87](../dsp/fx/delay.h#L85-L87)). No fractional interpolation at all — sweeping time gives full-sample stair-stepping on top of q15 quantization. This mode is the crunchiest of the three by design; the float-overload `Read(float)` exists right below it on lines 89-94 and produces audibly cleaner sweeps for the same RAM.

## Why the time knob feels untunable

Two separate problems, fix independently:

### Linear mapping

`delay_time_` arrives as `0..1` and is multiplied directly by `reserved`:

- `Process`: `reserved = buffer_size - 1 = 16383` → max **341 ms** per side at 48 kHz.
- `Process3`: `reserved = 24576` → max **512 ms**.
- `Process2`: `(buffer_size - 1) = 16383` → max **341 ms** mono.

At 48 kHz, the slap-back / haas region (`~5–30 ms`) lives in the bottom ~9 % of the knob, and the top half of the knob is "long delays all the same". Standard fix: cube the parameter (`x³`) or use an exponential curve so short / medium / long delays each get a third of the travel.

### No slew → pitch-glitch jumps

Even with a good curve, *changing* `delay_time_` discontinuously jumps the read pointer. Two well-known fixes:

- **Tape-style** (recommend): slew the read offset across the block with `stmlib::ParameterInterpolator`. The read pointer moves continuously, which produces musical pitch-bend (Doppler) on sweep — the "tape" character. Single-buffer.
- **Digital-style**: keep two read pointers, crossfade old→new over ~5–20 ms when `delay_time_` changes by more than a threshold. Time changes silently, no Doppler. Costs an extra read + a small state machine.

Recommend tape-style as the default — it matches the rest of the easter egg's flavour (chorus/ensemble/reverb modes are all wow-and-flutter-friendly) and is the smaller change.

## Suggested fix order (highest ROI first)

Items 1–3 cover ~80 % of the perceived crunch in ~30 lines of change total. Items 4–5 are tuning. Item 6 is dispatcher cleanup. Item 7 is the "rewrite" tier and is probably not worth the RAM cost.

### 1. Feedback-path LP — single biggest improvement

In each `Process*` loop, hold a `float feedback_state_l_`, `feedback_state_r_` on the `Delay` instance, and damp the feedback line:

```cpp
// pseudo, inside the FxEngine variant:
c.Load(0.0f);
c.Interpolate(line_l, offset, LFO_1, 0.5f, 1.0f);
c.Write(del_out);
c.Lp(feedback_state_l_, lp_coefficient_);  // <-- new
c.Write(del_out_damped);
// ... use del_out_damped in the feedback path
c.Load(del_out_damped * feedback_);
c.Read(*left, 0.5f);
c.Write(line_l, 1.0f);
```

`lp_coefficient_` ≈ `0.3–0.6` gives a comfortable analog-ish darkening per repeat. Expose as a private constant first; bind to a user control only if it earns the panel space.

### 2. SoftClip in the feedback path

```cpp
const float fb_in = SoftClip(del_out_damped * feedback_);
c.Load(fb_in);
```

`SoftClip` is already used elsewhere in the codebase (Plaits VCF, see [filter_plaits_vcf_port_plan.md](filter_plaits_vcf_port_plan.md)). Cheap, gracefully overdrives instead of crackling, lets you push `feedback_` past 1.0 safely if desired.

### 3. Slew `delay_time_` with `ParameterInterpolator`

```cpp
void Delay::Process(float* left, float* right, size_t size) {
  const float new_offset = static_cast<float>(reserved) * MapTime(delay_time_);
  stmlib::ParameterInterpolator offset_l(&previous_offset_l_, new_offset, size);
  stmlib::ParameterInterpolator offset_r(&previous_offset_r_, new_offset, size);
  // ... use offset_l.Next() / offset_r.Next() inside the per-sample loop
}
```

Adds `previous_offset_l_`, `previous_offset_r_` to `Delay`'s private state, init to `0.0f` in `Delay::Init`. `Process3` needs only `previous_offset_`. This is the change that kills the zipper. With this in place, the existing LFO modulation at ±0.5 samples is fine — it's not the issue.

### 4. Re-map `delay_time_` to a musical curve

```cpp
inline float MapTime(float x) {
  // cube: gives ~3.3% / 25% / 75% knob travel to slap / medium / long
  return x * x * x;
}
```

Or, if you want named bands, a 3-piece piecewise:

```cpp
inline float MapTime(float x) {
  if (x < 0.3f)       return x * (0.05f / 0.3f);            // 0–50 ms
  else if (x < 0.7f)  return 0.05f + (x - 0.3f) * (0.25f / 0.4f); // 50–300 ms
  else                return 0.30f + (x - 0.7f) * (0.70f / 0.3f); // 300 ms–max
}
```

The cube form is simpler and well-behaved with the slew from step 3 (smooth derivative everywhere). Recommend it as the default; revisit if user calibration data says otherwise.

### 5. DC-block the feedback return

```cpp
c.Hp(dc_state_, 0.001f);  // ~3.8 Hz at 48 kHz
```

Or a manual one-pole on the feedback sample. Prevents long-term ring DC build-up. Cheap insurance.

### 6. Dispatcher / mode cleanup

`Process2` is not currently selected by any `fx_type_` (the dispatch in [chord_string_synth.cc:188-200](../dsp/chord_string_synth.cc#L188-L200) only hits `Process` and `Process3`). Options:

- **Delete `Process2`** and the `DelayLine<uint16_t, buffer_size> delay_line_` member it backs. Frees up state, narrows the surface.
- **Or repair it**: replace `Read(int32_t)` with `Read(float)` ([delay.h:89-94](../dsp/fx/delay.h#L89-L94)) and apply fixes 1–5 to it as well; then it becomes a valid mono-sum variant alongside `Process3`.

Recommend deletion unless the user has a planned mode for it.

### 7. (Defer) Bump FxEngine storage to `FORMAT_32_BIT`

`FxEngine<32768, FORMAT_32_BIT>` would eliminate the q15 noise floor entirely — but the buffer is `32768 * 4 = 128 KB`, which won't co-exist with the existing reverb buffer on this STM32F4 budget. The current shared `uint16_t* reverb_buffer` allocation in `rings.cc` is `32768 * 2 = 64 KB`; doubling it means either dropping the reverb mode or reducing delay length. Skip until/unless fixes 1–5 prove insufficient on hardware.

## Risks

- **Step 3 changes the audible behaviour of fast CV sweeps** — they'll now Doppler-pitch-bend instead of click/zipper. This is the *correct* behaviour but it *is* a behaviour change; mention in the release notes for anyone who liked the artifact.
- **Step 1 LP coefficient is a taste call.** Too aggressive (`> 0.7`) muddies short delays. Too gentle (`< 0.2`) doesn't mask q15 noise. Start at `0.5`, A/B at three feedback settings.
- **Step 4 changes the meaning of every saved patch's `delay_time`.** If patches are persisted (check `settings.cc` — currently `fx_type` is persisted via `state.model` but `delay_time` looks like a live `PerformanceState` field, not stored), no migration needed; otherwise add a settings version bump.
- **CPU**: `ParameterInterpolator::Next()` + one extra `Lp` + one `SoftClip` per sample × two channels in `Process`. Trivial on F4, but verify with `make size` and ideally a scope-on-trigger pin if you're already CPU-tight (you shouldn't be — easter egg is far below the stock engine's voice count).

## Open questions

- **Q1**. Keep `Process2` (after repair) as a "lo-fi / digital crunch" voicing, or delete it? Current vote: delete — the easter egg already has plenty of FX flavours, and a real lo-fi mode would warrant bit-rate + sample-rate reduction, not just a stair-stepped read.
- **Q2**. Should the LP coefficient be user-controllable? Easiest panel home would be: scale it from `feedback_` itself (more feedback → more damping, classic Lexicon move). Avoids adding a UI control.
- **Q3**. Tape-style (single read, Doppler-on-sweep) vs digital-style (crossfade, silent retunes) — confirm tape is the desired character before committing step 3.
- **Q4**. Time-knob curve: cube vs piecewise. Default cube; switch if hardware testing says short delays still feel cramped.

## Suggested commit sequence

Each step is independently audible and bisectable on hardware:

1. Add `feedback_state_*` members + LP in feedback path (step 1). Sound: highs roll off in tail, fizz reduced, behaviour otherwise unchanged. ~10 lines.
2. SoftClip the feedback multiply (step 2). Sound: high-feedback regimes stop cracking; can now push past `feedback_ = 1.0`. ~3 lines.
3. `ParameterInterpolator` on the read offset (step 3). Sound: knob/CV moves now glide instead of click. ~15 lines across the three methods + struct fields. Biggest perceptual win.
4. Cube time map (step 4). Sound: knob feels usable end-to-end. ~3 lines.
5. DC-block on feedback (step 5). Inaudible day one, prevents long-tail bias drift. ~2 lines.
6. Delete `Process2` and its backing `delay_line_` (step 6). Pure cleanup. ~15 lines removed.
7. Stop here unless on-hardware listening reveals residual q15 hash; if so, scope step 7 separately with a RAM/feature-tradeoff doc.

## Out of scope

- Tap-tempo / clocked-delay input.
- Reverse delay, granular repeats, pitch-shifted feedback.
- Changing the shared reverb buffer allocation.
- Stock Rings (`Part`) — uses none of these FX classes.
