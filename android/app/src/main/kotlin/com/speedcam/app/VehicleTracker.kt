package com.speedcam.app

import com.speedcam.core.Estimate
import com.speedcam.core.Ground
import com.speedcam.core.SpeedEstimator
import com.speedcam.core.TrackPoint
import kotlin.math.abs

/**
 * Single-vehicle tracking state machine — port of the web app's processFrame tracking
 * (without the optical-flow stage; native v1 relies on the detector box + EMA).
 */
class VehicleTracker(private val onTrackFinished: (FinishedTrack) -> Unit) {

    data class Detection(
        val x: Double, val y: Double, val w: Double, val h: Double, val label: String
    )

    data class FinishedTrack(
        val history: List<TrackPoint>,
        val speedKmh: Double,
        val errKmh: Double,
        val source: String,
        val vehicleClass: String
    )

    data class FrameResult(
        val box: Detection?,
        val displaySpeedKmh: Double?,
        val errKmh: Double?,
        val source: String?,
        val tracking: Boolean
    )

    companion object {
        const val EMA_ALPHA = 0.5
        const val TRACKING_TIMEOUT_S = 1.5
        const val HISTORY_CAP = 60
        val VEHICLE_LENGTHS_M = mapOf(
            "car" to 4.5, "truck" to 8.0, "bus" to 12.0, "motorcycle" to 2.2
        )
    }

    private var history = mutableListOf<TrackPoint>()
    private var vehicleClass = "car"
    private var smoothedSpeed: Double? = null
    private var finalSpeed: Double? = null
    private var finalErr: Double? = null
    private var lastSource: String? = null
    private var lastSeenT = 0.0

    /**
     * Advance one frame. [t] is capture time in seconds; [groundOf] projects an image
     * pixel onto the road plane (null when orientation/height unavailable).
     */
    fun onFrame(det: Detection?, t: Double, groundOf: (Double, Double) -> Ground?): FrameResult {
        if (det == null) {
            if (history.isNotEmpty() && t - lastSeenT > TRACKING_TIMEOUT_S) {
                finalize()
            }
            return FrameResult(null, null, null, null, history.isNotEmpty())
        }
        lastSeenT = t

        var cx = det.x + det.w / 2
        var cy = det.y + det.h / 2
        var cw = det.w
        var ch = det.h

        // If the centroid jumped more than a full car width, it's probably a new car
        val last = history.lastOrNull()
        if (last != null && abs(cx - last.x) > det.w * 1.5) {
            finalize()
        }

        val prev = history.lastOrNull()
        if (prev != null) {
            cx = EMA_ALPHA * cx + (1 - EMA_ALPHA) * prev.x
            cy = EMA_ALPHA * cy + (1 - EMA_ALPHA) * prev.y
            cw = EMA_ALPHA * cw + (1 - EMA_ALPHA) * prev.w
            ch = EMA_ALPHA * ch + (1 - EMA_ALPHA) * prev.h
        }
        history.add(TrackPoint(t, cx, cy, cw, ch, groundOf(cx, cy + ch / 2)))
        if (history.size > HISTORY_CAP) history.removeAt(0)
        vehicleClass = det.label

        var est: Estimate? = null
        SpeedEstimator.window(history, t)?.let { pts ->
            est = SpeedEstimator.estimate(pts, VEHICLE_LENGTHS_M[vehicleClass] ?: 4.5)
        }
        est?.let { lastSource = it.source }

        val e = est
        if (e != null &&
            e.speedKmh >= SpeedEstimator.MIN_SPEED_KMH &&
            e.speedKmh <= SpeedEstimator.MAX_SPEED_KMH &&
            e.errKmh / e.speedKmh <= SpeedEstimator.MAX_REL_STDERR
        ) {
            smoothedSpeed = smoothedSpeed?.let { it * 0.7 + e.speedKmh * 0.3 } ?: e.speedKmh
            finalSpeed = smoothedSpeed
            finalErr = e.errKmh
        }

        return FrameResult(det, smoothedSpeed, finalErr, lastSource, true)
    }

    private fun finalize() {
        val speed = finalSpeed
        val err = finalErr
        if (speed != null && speed >= SpeedEstimator.MIN_SPEED_KMH &&
            (err == null || err / speed <= SpeedEstimator.SAVE_MAX_REL_STDERR)
        ) {
            onTrackFinished(
                FinishedTrack(history.toList(), speed, err ?: 0.0, lastSource ?: "?", vehicleClass)
            )
        }
        history = mutableListOf()
        smoothedSpeed = null
        finalSpeed = null
        finalErr = null
        lastSource = null
    }
}
