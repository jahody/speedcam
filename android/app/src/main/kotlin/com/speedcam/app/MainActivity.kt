package com.speedcam.app

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.Matrix
import android.graphics.RectF
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.util.Size
import android.widget.EditText
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.speedcam.core.Doppler
import com.speedcam.core.Ground
import com.speedcam.core.GroundPlane
import com.speedcam.core.TrackPoint
import com.speedcam.core.Vec3
import org.tensorflow.lite.task.vision.detector.ObjectDetector
import org.tensorflow.lite.support.image.TensorImage
import java.util.concurrent.Executors
import kotlin.math.roundToInt

class MainActivity : AppCompatActivity(), SensorEventListener {

    companion object {
        private const val MODEL_ASSET = "efficientdet_lite0.tflite"
        private val VEHICLE_LABELS = setOf("car", "truck", "bus", "motorcycle")
        private const val SCORE_THRESHOLD = 0.45f
        private const val MIN_BOX_PX = 40
        private const val SPEED_LIMIT_KMH = 50.0
        private const val PREFS = "speedcam"
    }

    private lateinit var previewView: PreviewView
    private lateinit var overlay: OverlayView
    private lateinit var speedText: TextView
    private lateinit var statusText: TextView
    private lateinit var heightInput: EditText

    private val analysisExecutor = Executors.newSingleThreadExecutor()
    private val mainHandler = Handler(Looper.getMainLooper())
    private var detector: ObjectDetector? = null
    private var intrinsics: CameraGeometry.Intrinsics? = null
    private val audioDoppler = AudioDoppler()

    // Gravity-up unit vector in the (portrait-locked) camera frame; volatile-ish via lock
    private val gravityLock = Any()
    private var up: Vec3? = null

    private var cameraHeightM = 1.5
    private var heightAuto = false
    private val dopplerHeightSamples = ArrayList<Double>()

    private var fps = 0
    private var frameCount = 0
    private var lastFpsAt = 0L

    private val tracker = VehicleTracker { finished -> onTrackFinished(finished) }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        previewView = findViewById(R.id.preview)
        overlay = findViewById(R.id.overlay)
        speedText = findViewById(R.id.speed)
        statusText = findViewById(R.id.status)
        heightInput = findViewById(R.id.height)

        val prefs = getSharedPreferences(PREFS, Context.MODE_PRIVATE)
        cameraHeightM = prefs.getFloat("cameraHeight", 1.5f).toDouble()
        heightAuto = prefs.getBoolean("heightAuto", false)
        heightInput.setText(String.format("%.2f", cameraHeightM))
        heightInput.setOnEditorActionListener { v, _, _ ->
            // Manual entry asserts new ground truth: discard auto-calibration state
            cameraHeightM = v.text.toString().toDoubleOrNull() ?: 1.5
            heightAuto = false
            dopplerHeightSamples.clear()
            prefs.edit().putFloat("cameraHeight", cameraHeightM.toFloat())
                .putBoolean("heightAuto", false).apply()
            updateStatus("height set to %.2f m".format(cameraHeightM))
            false
        }

        detector = try {
            ObjectDetector.createFromFileAndOptions(
                this, MODEL_ASSET,
                ObjectDetector.ObjectDetectorOptions.builder()
                    .setMaxResults(5)
                    .setScoreThreshold(SCORE_THRESHOLD)
                    .build()
            )
        } catch (e: Exception) {
            statusText.text = "Model missing: put $MODEL_ASSET in app/src/main/assets/"
            null
        }

        val perms = arrayOf(Manifest.permission.CAMERA, Manifest.permission.RECORD_AUDIO)
        if (perms.all {
                ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
            }) {
            startEverything()
        } else {
            ActivityCompat.requestPermissions(this, perms, 1)
        }
    }

    override fun onRequestPermissionsResult(
        code: Int, permissions: Array<out String>, results: IntArray
    ) {
        super.onRequestPermissionsResult(code, permissions, results)
        if (results.isNotEmpty() && results[0] == PackageManager.PERMISSION_GRANTED) {
            startEverything() // camera granted; audio may still be denied (Doppler optional)
        } else {
            statusText.text = "Camera permission required"
        }
    }

    private fun startEverything() {
        startCamera()
        val sm = getSystemService(SENSOR_SERVICE) as SensorManager
        sm.getDefaultSensor(Sensor.TYPE_GRAVITY)?.let {
            sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO)
            == PackageManager.PERMISSION_GRANTED
        ) {
            audioDoppler.start()
        }
    }

    private fun startCamera() {
        val providerFuture = ProcessCameraProvider.getInstance(this)
        providerFuture.addListener({
            val provider = providerFuture.get()
            val preview = Preview.Builder().build()
                .also { it.setSurfaceProvider(previewView.surfaceProvider) }

            @Suppress("DEPRECATION")
            val analysis = ImageAnalysis.Builder()
                .setTargetResolution(Size(1280, 720))
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                .build()
            analysis.setAnalyzer(analysisExecutor) { image -> analyze(image) }

            provider.unbindAll()
            val camera = provider.bindToLifecycle(
                this, CameraSelector.DEFAULT_BACK_CAMERA, preview, analysis
            )
            intrinsics = CameraGeometry.read(camera.cameraInfo)
            updateStatus(
                intrinsics?.let { "intrinsics: f=%.2f mm, sensor %.2f mm".format(it.focalMm, it.sensorWidthMm) }
                    ?: "intrinsics unavailable — ground-plane disabled"
            )
        }, ContextCompat.getMainExecutor(this))
    }

    private fun analyze(image: ImageProxy) {
        val det = detector
        if (det == null) { image.close(); return }

        // Capture time in seconds, same clock as the audio spectra when the sensor
        // supports the REALTIME timestamp source; otherwise fall back to arrival time
        val t = if (intrinsics?.timestampsAreRealtime == true) {
            image.imageInfo.timestamp / 1e9
        } else {
            SystemClock.elapsedRealtimeNanos() / 1e9
        }

        val bitmap = image.use { rgbaToUprightBitmap(it) }
        val results = det.detect(TensorImage.fromBitmap(bitmap))

        // Largest vehicle box wins (same policy as the web app)
        var best: VehicleTracker.Detection? = null
        var bestArea = 0.0
        for (r in results) {
            val cat = r.categories.firstOrNull() ?: continue
            if (cat.label !in VEHICLE_LABELS) continue
            val b = r.boundingBox
            if (b.width() < MIN_BOX_PX) continue
            val area = (b.width() * b.height()).toDouble()
            if (area > bestArea) {
                bestArea = area
                best = VehicleTracker.Detection(
                    b.left.toDouble(), b.top.toDouble(),
                    b.width().toDouble(), b.height().toDouble(), cat.label
                )
            }
        }

        val f = intrinsics?.focalPx(maxOf(bitmap.width, bitmap.height))
        val upNow = synchronized(gravityLock) { up }
        val groundOf: (Double, Double) -> Ground? = { x, y ->
            if (f != null && upNow != null) {
                GroundPlane.projectToGround(
                    x, y, bitmap.width.toDouble(), bitmap.height.toDouble(),
                    f, upNow, cameraHeightM
                )
            } else null
        }

        val result = tracker.onFrame(best, t, groundOf)

        frameCount++
        val now = SystemClock.elapsedRealtime()
        if (now - lastFpsAt >= 1000) { fps = frameCount; frameCount = 0; lastFpsAt = now }

        mainHandler.post {
            val speed = result.displaySpeedKmh
            val over = speed != null && speed > SPEED_LIMIT_KMH
            overlay.update(
                result.box?.let { RectF(it.x.toFloat(), it.y.toFloat(),
                    (it.x + it.w).toFloat(), (it.y + it.h).toFloat()) },
                bitmap.width, bitmap.height, over
            )
            if (speed != null) {
                speedText.text = "${speed.roundToInt()} km/h ±${maxOf(1.0, result.errKmh ?: 1.0).roundToInt()}"
            } else if (!result.tracking) {
                speedText.text = "--"
            }
            val src = result.source?.let { " • $it" } ?: ""
            val h = "h=%.2f m%s".format(cameraHeightM, if (heightAuto) " (auto)" else "")
            statusText.text = if (result.tracking) "tracking$src • $h • $fps fps"
                              else "aim at moving cars • $h • $fps fps"
        }
    }

    /** RGBA_8888 ImageProxy -> upright Bitmap (rotated by the frame's rotationDegrees). */
    private fun rgbaToUprightBitmap(image: ImageProxy): Bitmap {
        val plane = image.planes[0]
        val pixelStride = plane.pixelStride
        val rowPadding = plane.rowStride - pixelStride * image.width
        val paddedW = image.width + rowPadding / pixelStride
        var bmp = Bitmap.createBitmap(paddedW, image.height, Bitmap.Config.ARGB_8888)
        bmp.copyPixelsFromBuffer(plane.buffer)
        if (paddedW != image.width) {
            bmp = Bitmap.createBitmap(bmp, 0, 0, image.width, image.height)
        }
        val rotation = image.imageInfo.rotationDegrees
        if (rotation != 0) {
            val m = Matrix().apply { postRotate(rotation.toFloat()) }
            bmp = Bitmap.createBitmap(bmp, 0, 0, bmp.width, bmp.height, m, true)
        }
        return bmp
    }

    private fun onTrackFinished(track: VehicleTracker.FinishedTrack) {
        mainHandler.post {
            updateStatus("logged %.0f km/h ±%.0f (%s)".format(track.speedKmh, track.errKmh, track.source))
        }
        if (!audioDoppler.active) return
        // Wait for the recede audio, then measure
        mainHandler.postDelayed({
            val gp = track.history.filter { it.ground != null }
            val res = Doppler.estimate(gp, audioDoppler.snapshot()) ?: return@postDelayed
            Toast.makeText(
                this,
                "Doppler: %.0f km/h (camera %.0f)".format(res.speedKmh, track.speedKmh),
                Toast.LENGTH_LONG
            ).show()
            recordDopplerScale(track, res.speedKmh)
        }, 2500)
    }

    /** Ground-plane speeds scale linearly with camera height, so the Doppler/vision
     *  ratio implies the true height — same auto-calibration as the web app. */
    private fun recordDopplerScale(track: VehicleTracker.FinishedTrack, dopplerKmh: Double) {
        if (track.source != "imu") return
        val ratio = dopplerKmh / track.speedKmh
        if (ratio <= 0.5 || ratio >= 2.0) return
        dopplerHeightSamples.add(cameraHeightM * ratio)
        if (dopplerHeightSamples.size > 30) dopplerHeightSamples.removeAt(0)
        if (dopplerHeightSamples.size >= 5) {
            val target = dopplerHeightSamples.sorted()[dopplerHeightSamples.size / 2]
            cameraHeightM += 0.25 * (target - cameraHeightM)
            cameraHeightM = cameraHeightM.coerceIn(0.5, 12.0)
            heightAuto = true
            getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit()
                .putFloat("cameraHeight", cameraHeightM.toFloat())
                .putBoolean("heightAuto", true).apply()
            mainHandler.post { heightInput.setText(String.format("%.2f", cameraHeightM)) }
        }
    }

    private fun updateStatus(msg: String) {
        statusText.text = msg
    }

    // ===== SensorEventListener =====
    override fun onSensorChanged(event: SensorEvent) {
        if (event.sensor.type != Sensor.TYPE_GRAVITY) return
        // Portrait-locked activity: device coords == camera frame (x right, y up, -z view)
        synchronized(gravityLock) {
            up = Vec3(
                event.values[0].toDouble(),
                event.values[1].toDouble(),
                event.values[2].toDouble()
            ).normalized()
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    override fun onDestroy() {
        super.onDestroy()
        audioDoppler.stop()
        analysisExecutor.shutdown()
        (getSystemService(SENSOR_SERVICE) as SensorManager).unregisterListener(this)
    }
}
