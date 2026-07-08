# SpeedCam — native Android app

Phase 4 of the [roadmap](../GOAL.md): the accuracy flagship. Exact camera intrinsics
(Camera2), gravity-sensor ground-plane geometry, nanosecond sensor timestamps, and the
same estimation math as the web app — shared through the pure-Kotlin `:core` module.

## Modules

- **`core/`** — pure JVM Kotlin, no Android dependencies. All the estimation math:
  least-squares speed fitting (`LinFit`, `SpeedEstimator`), pinhole/ground-plane
  projection (`GroundPlane`), audio Doppler (`Doppler`, `Fft`). Unit-tested against
  the same verification vectors as the web implementation, including a cross-language
  regression test pinning `projectToGround` to the JS output.
- **`app/`** — the Android app. CameraX analysis stream → TFLite EfficientDet-Lite0
  (COCO: car/truck/bus/motorcycle) → `VehicleTracker` (port of the web tracking logic)
  → live speed with ± confidence. `AudioDoppler` runs the retrospective per-pass
  Doppler cross-check and auto-calibrates camera height from the Doppler/vision ratio.

## Build

Open `android/` in Android Studio (it supplies the SDK via `local.properties`), or:

```
gradlew :app:assembleDebug     # needs ANDROID_HOME or local.properties
gradlew :core:test             # pure JVM, no SDK needed
```

The detection model `app/src/main/assets/efficientdet_lite0.tflite` is committed
(4.5 MB, from download.tensorflow.org). If it's ever missing, the app shows a status
message telling you where to put it.

## How it measures

1. Each analysis frame is rotated upright; the activity is **portrait-locked**, which
   makes the gravity vector (TYPE_GRAVITY, device coords) directly usable as the
   up-vector in the camera frame — no rotation bookkeeping.
2. Focal length in pixels comes from `LENS_INFO_AVAILABLE_FOCAL_LENGTHS` and
   `SENSOR_INFO_PHYSICAL_SIZE` — measured hardware values, no FOV guessing.
3. The largest detected vehicle box is tracked; its road-plane positions are fitted
   by least squares over the last second → speed ± standard error (`:core`).
4. After each pass, the Doppler module cross-correlates approach/recede audio spectra
   and reports an independent speed; ground-plane/Doppler disagreement continuously
   refines the camera-height estimate.

## Known gaps vs the web app (v1)

- No optical-flow tracking stage yet (detector box + EMA only).
- No history UI / photo capture — measurements surface via the status line and toasts.
- No manual two-tap calibration (native shouldn't need it).
- ARCore 6-DoF pose is not used yet; gravity + fixed height is the v1 geometry. ARCore
  would additionally handle camera translation and read per-frame intrinsics.

## On-device checklist (first run on a Pixel 8a)

1. Grant camera + microphone permissions.
2. Check the status line shows real intrinsics (f ≈ 6.9 mm, sensor ≈ 9.8 mm wide).
3. Set your camera height (m); after ~5 Doppler-validated passes it switches to (auto).
4. Compare the live speed, the logged value, and the Doppler toast against a known
   reference (GPS speedometer of a cooperating driver).
