# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

See `GOAL.md` for the project mission (near-radar accuracy with automatic calibration) and the phased roadmap.

Two implementations share the same estimation concepts:
- **Web app** (`index.html`) — described below; zero-install entry point.
- **Native Android app** (`android/`) — the accuracy flagship: exact Camera2 intrinsics, gravity-sensor ground plane, TFLite detection. Estimation math lives in the pure-Kotlin `android/core` module (`gradlew :core:test` runs without an Android SDK); the tests pin the Kotlin ports to the web implementation's verified vectors. See `android/README.md`.

SpeedCam Mobile — a single-page, single-file web app (`index.html`) that uses a phone's rear camera plus TensorFlow.js (COCO-SSD) to detect passing vehicles and estimate their speed in real time. Everything — HTML, CSS, and JS — lives in `index.html`; there is no build step, package.json, or bundler.

## Running it

```
python -m http.server 8377
```
(also defined as the `speedcam` launch config in `.claude/launch.json`). Open the served `index.html` on a phone (or a desktop browser with camera + motion-sensor emulation) — `getUserMedia` requires a secure context (HTTPS or localhost).

There is no test suite, linter, or build/bundling step. TensorFlow.js and the COCO-SSD model are loaded from CDN (`<script>` tags at the bottom of the file), not installed as dependencies.

## Architecture

Everything is organized as top-level functions and module-scoped state inside the one `<script>` block in `index.html`. Key sections, in the order they appear:

- **CONFIG / STATE** — tunable constants (`VEHICLE_LENGTHS_M`, FOV bounds, EMA smoothing factors, speed thresholds) and mutable state, most of it persisted to `localStorage` (calibration, view mode, speed limit, FOV estimate, saved records).
- **`init()` / `startDetectLoop()` / `processFrame()`** — camera + model bootstrap, then a per-video-frame loop (`requestVideoFrameCallback` where available, so each track point carries the camera frame's own capture timestamp `mediaTime`; rAF fallback otherwise) that runs COCO-SSD detection, tracks the single largest vehicle bounding box, and drives speed estimation. Track points store `t` (capture time, used for all speed math) and `wall` (`performance.now()`, used only for timeout logic).
- **Speed estimator chain** (`estimateSpeed`) — three interchangeable estimators tried in priority order. Each least-squares-fits position vs capture time over the recent track window (`trackWindow`/`linFit`/`fitSpeed2D`) and returns `{ speedKmh, errKmh, source }` or `null`, where `errKmh` is the fit's standard error propagated to speed — displayed as ± in the UI, stored in records, and used for quality gating (estimates with high relative error are rejected rather than shown/saved):
  1. `estimateManual` — pixel-to-meter ratio from the manual two-tap calibration wizard (`PIXELS_PER_METER`).
  2. `estimateGroundPlane` — IMU-based: projects tracked pixel points onto the real-world road plane using device orientation (`deviceorientation`) and a known camera height, then measures real ground displacement. No vehicle-size guessing.
  3. `estimateVehicleSize` — fallback pinhole/proportion estimate using average per-class vehicle dimensions (`VEHICLE_LENGTHS_M`/`VEHICLE_WIDTHS_M`) when neither of the above is available.
  A track only produces a speed once the fit window has enough points (≥6) spanning enough time (≥350 ms); results are lightly EMA-smoothed before display.
- **Optical flow tracker** (`updateFlow`, `lkTrack`, `detectFeatures`, `grayPyramid`) — pyramidal Lucas-Kanade on a 320px-wide grayscale downscale of the video. Tracks Shi-Tomasi corners inside the detection box between consecutive frames; the median feature displacement (with a velocity prior and a median-consensus inlier gate) advances a virtual vehicle position that replaces the jittery detection-box centroid in track points. A slow blend (`FLOW_BLEND`) toward the detection centroid prevents integration drift; when flow has no lock (too few agreeing features, stale frames, new car) it returns null and the loop falls back to the EMA-smoothed detection centroid.
- **Audio Doppler** (`initAudio`, `captureSpectrum`, `estimateDoppler`, `scheduleDoppler`, `recordDopplerScale`) — optional microphone-based retrospective speed check. A rolling buffer of log-frequency spectra (AnalyserNode, raw audio with browser processing disabled) is cross-correlated between the approach and recede windows of each pass; the log-frequency shift gives the Doppler ratio, and the pass geometry (closest-approach time/distance from a constant-velocity fit of the ground-plane track, wall-clock timebase) converts it to speed. Runs ~2.5 s after a record saves, annotates `record.dopplerKmh`, and — because ground-plane vision speed scales linearly with camera height — feeds Doppler/vision ratios into height auto-calibration (`dopplerHeightSamples`), which takes precedence over the car-length statistic once it has ≥5 samples.
- **IMU ground-plane math** (`initOrientation`, `listenOrientation`, `earthUpInDevice`, `projectToGround`) — converts `deviceorientation` beta/gamma into an up vector, casts a ray per detected pixel, and intersects it with a horizontal ground plane at `CAMERA_HEIGHT_M` to get real-world lat/depth coordinates. Handles iOS's gesture-gated permission flow (`imu.needsTap`) vs Android's ungated events.
- **Scale auto-tuning** (`recordScaleSample`, `detectDeviceFov`) — each side-on car track implies a real car length (assuming ~4.5m average); the median over many cars nudges the one unknown geometry parameter. On known devices (Pixel 6–9 → 70° horizontal FOV via UA match, `FOV_LOCKED`) the FOV is fixed and the statistics auto-tune **camera height** (`HEIGHT_AUTO`); on unknown devices the user-entered height is trusted and the statistics tune `FOV_H_DEG` instead. The two parameters are degenerate for this observable, so only one is ever tuned. A manual height entry resets the height auto-tune state.
- **Calibration wizard** (`startCalibrationWizard`) — two-tap UI on a canvas overlay to set a manual real-world distance between two road points, producing `PIXELS_PER_METER` (highest-priority estimator, works in "side" view only).
- **History / records** — finalized tracks are saved to `localStorage` (`speedRecords`, capped at 50) with a captured JPEG frame; the history panel renders them.
- **Location context** (`fetchLocationContext`, `fetchOsmRoadData`) — optional GPS lookup: reverse-geocodes via a user-supplied Mapy.com API key for the road name, and queries the Overpass API for `maxspeed` tags to auto-set the speed limit threshold.
- **`window.__speedcamDebug`** — an intentional escape hatch exposing `projectToGround`, `estimateGroundPlane`, `recordFovSample`, orientation setters, etc., for manual testing in a browser console without needing a real camera/sensors.

## Key behaviors to preserve when editing

- View mode (`side` vs `front`) changes which geometry model applies; `side` supports manual calibration and the ground-plane's lateral/depth decomposition, `front` uses a pinhole distance-from-width model instead.
- The estimator priority order (manual > ground-plane/IMU > vehicle-size) is deliberate — manual calibration is most accurate, IMU ground-plane needs no per-vehicle guessing, size-based is the least reliable fallback.
- All persisted settings/state use `localStorage` directly with no abstraction layer — new persisted fields should follow the existing `localStorage.getItem/setItem` pattern used throughout `init()` and the config block.
