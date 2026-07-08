# SpeedCam — Goal & Roadmap

## Mission

Measure the speed of passing cars with a cell phone (primary target: **Pixel 8a**) at **near-radar accuracy (±3 km/h)** with **zero or minimal manual calibration** — ideally the user just points the phone at the road and gets trustworthy numbers.

**Non-goals (for now):**
- Legal / enforcement-grade evidence (chain of custody, certified accuracy)
- Tracking multiple vehicles simultaneously across lanes
- Supporting every phone model equally — Pixel 8a first, generalize later

## Why ±3 km/h is hard — the accuracy budget

Speed = distance / time. Every error source lands in one of those two terms:

| Error source | Current state | Impact | Fix |
|---|---|---|---|
| **Scale / geometry** (FOV, camera height, pitch) | FOV guessed at 70° and auto-tuned statistically; camera height typed in by the user | Dominant — a 5% FOV error is a 5% speed error (≈2.5 km/h at 50 km/h) | Known intrinsics, ARCore pose, calibration-free physics (Doppler) |
| **Tracking noise** | Centroid of a COCO-SSD bounding box, EMA-smoothed; the box breathes as the car's aspect changes | Large at short tracks — box jitter of a few px over 400 ms is several km/h | Feature-level tracking (optical flow), full-track regression |
| **Timing** | `requestAnimationFrame` timestamps ≠ camera exposure times; frames can be dropped or delivered late | Up to a frame period of jitter (≈33–66 ms) on a 400 ms baseline → several % | `requestVideoFrameCallback` (web) / sensor timestamps (native) |
| **Two-point displacement** | Speed from just 2 samples ~400 ms apart | Throws away most of the track's information | Least-squares fit over the whole track |
| **Road-plane assumption** | Ground assumed flat and horizontal at the tracked point | Small on flat roads; grows with crown/slope | ARCore plane detection; accept as residual error on web |

Conclusion: the current pipeline (detection-box centroid + rAF timing + statistical FOV) tops out around ±10–15%. Reaching ±3 km/h (≈3–6% at urban speeds) requires attacking **all** rows of this table, and the geometry row realistically needs either exact intrinsics/pose (native) or calibration-free physics (audio Doppler).

## Calibration strategy — the path to "fully automatic"

Ordered from least to most automatic. Later entries don't replace earlier ones — they fuse.

1. **Known-device intrinsics** — the Pixel 8a main camera's FOV is a fixed, publicly known constant. Ship a small per-device table (keyed off `navigator.userAgent` / model hints) instead of guessing 70° and slowly auto-tuning. Immediate, free accuracy win.
2. **IMU ground-plane** *(exists today)* — device orientation + camera height gives a metric ground plane with no per-scene calibration. The remaining manual input is **camera height**; estimate it automatically by statistically fitting the observed car-size distribution over many tracks (cars average 4.5 m long, 1.45 m tall — many samples pin down height well).
3. **Road-feature scale detection** — dashed lane markings have standardized lengths (e.g. 3 m line / 6 m gap in much of Europe). Detecting them in the image gives an in-scene metric scale with zero user input, independent of IMU and height.
4. **Audio Doppler** — the physics anchor. A passing car's tire/engine noise shifts in frequency; the shift through closest approach yields the car's speed from **first principles with no geometric calibration at all** (only the speed of sound). Fused with vision, this both measures and continuously *recalibrates* the camera geometry. This is the key to ±3 km/h on the web stack.
5. **Native track: exact geometry** — an Android app reads exact per-frame camera intrinsics (Camera2) and full 6-DoF device pose + detected ground planes (ARCore). Calibration ceases to exist as a concept: the geometry is simply known. This is where ±3 km/h becomes realistic rather than aspirational.

## Roadmap

### Phase 1 (web) — measurement fidelity
Fix the error sources that need no new calibration:
- [x] Switch frame timing to `requestVideoFrameCallback` (exposes camera `mediaTime` per frame) instead of `requestAnimationFrame`.
- [x] Replace 2-point displacement with a **least-squares linear fit** of ground-plane position over the whole track; report the fit residual as a per-measurement confidence (shown as ± on screen and in the log).
- [x] Track **features inside the vehicle** (optical flow on corner points) rather than the detection-box centroid, using COCO-SSD only for finding/classifying the vehicle. (Pyramidal Lucas-Kanade at 320 px width, velocity prior, median-consensus gating, drift-corrected by slow blend toward the detection centroid; falls back to the centroid when flow has no lock.)
- [x] Quality gating: discard tracks that are too short, too noisy, or partially occluded instead of averaging bad data in.

### Phase 2 (web) — automatic calibration
- [x] Pixel 8a (and friends) **intrinsics lookup table**; keep statistical FOV auto-tune only as fallback for unknown devices. (Pixel 6–9 main cameras ≈25 mm equivalent → 70° horizontal at 16:9; FOV is locked on these devices.)
- [x] **Automatic camera-height estimation** from the car-size distribution across accumulated tracks (removes the last manual input). On locked-FOV devices the car-size statistics tune height instead of FOV; the manual entry is only a starting guess, and editing it resets the auto-tune.
- [ ] Optional: **lane-marking detection** for in-scene metric scale.

### Phase 3 (web) — audio Doppler fusion
- [x] Microphone capture + FFT around each vehicle's closest approach; extract the Doppler curve and solve for speed. (Rolling log-frequency spectrum buffer; the pass geometry — closest-approach time, distance, angle factor μ — comes from the vision ground-plane track; speed from the log-spectral shift between approach and recede windows: v = c(R−1)/(μ(R+1)). Runs retrospectively ~2.5 s after each save and annotates the record.)
- [x] Fuse Doppler and vision estimates (Doppler as the unbiased anchor, vision for per-vehicle association and coverage); use disagreement to auto-correct camera geometry over time. (Doppler/vision speed ratios imply the true camera height; with ≥5 samples this supersedes the car-length height statistic.)

### Phase 4 (native Android) — exact geometry
- [x] Android app sharing the estimation/fusion logic (see `android/`): exact intrinsics from Camera2 (`LENS_INFO_AVAILABLE_FOCAL_LENGTHS` + `SENSOR_INFO_PHYSICAL_SIZE` — no FOV guessing), gravity-sensor ground plane, sensor-clock frame timestamps, TFLite EfficientDet-Lite0 detection, full regression + confidence pipeline, audio Doppler cross-check with height auto-calibration. The math lives in a pure-Kotlin `core` module unit-tested against the web implementation's verification vectors.
- [ ] ARCore 6-DoF pose (handles camera translation, per-frame intrinsics) — future upgrade; v1 uses gravity + fixed height.
- The web app remains the zero-install entry point; the native app is the accuracy flagship.

### Phase 5 — validation & error reporting
- Ground-truth protocol: drive your own car past the phone at known speed (GPS logger / OBD-II) across a matrix of speeds, distances, and angles.
- In-app per-measurement **confidence interval**, derived from fit residuals and estimator agreement — a speed without an error bar is not a measurement.
- Track accuracy per estimator (manual / ground-plane / size / Doppler / native) against ground truth to prove which phases actually paid off.

## Success criteria

- **Native + fusion stack:** ±3 km/h at 95th percentile vs GPS ground truth, for cars at 30–90 km/h within ~30 m.
- **Pure web stack:** ±5–10% under the same protocol.
- **Calibration burden on Pixel 8a:** zero required user input beyond pointing the phone (no height entry, no tap calibration, no angle setting).
- Every displayed speed carries a confidence estimate.
