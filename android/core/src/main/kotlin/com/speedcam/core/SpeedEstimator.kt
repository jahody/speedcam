package com.speedcam.core

/**
 * Full-track regression speed estimators — port of the web app's estimator chain.
 * Native has exact intrinsics + IMU, so ground-plane is primary; the vehicle-size
 * pixel fallback covers moments without gravity/projection data.
 */
object SpeedEstimator {

    const val REG_WINDOW_S = 1.0       // fit over at most the last second of track
    const val MIN_TRACK_POINTS = 6     // minimum samples in the fit window
    const val MIN_TRACK_SPAN_S = 0.35  // minimum time covered by the fit window
    const val MAX_REL_STDERR = 0.3     // reject live estimates noisier than ±30%
    const val SAVE_MAX_REL_STDERR = 0.25

    const val MIN_SPEED_KMH = 5.0
    const val MAX_SPEED_KMH = 180.0

    /** Recent points suitable for fitting, or null when the track is too short/brief. */
    fun window(history: List<TrackPoint>, tNow: Double): List<TrackPoint>? {
        val pts = history.filter { tNow - it.t <= REG_WINDOW_S }
        if (pts.size < MIN_TRACK_POINTS) return null
        if (pts.last().t - pts.first().t < MIN_TRACK_SPAN_S) return null
        return pts
    }

    fun estimate(pts: List<TrackPoint>, vehicleLengthM: Double = 4.5): Estimate? =
        groundPlane(pts) ?: sizeSide(pts, vehicleLengthM)

    /** Metric speed from projected road positions. */
    fun groundPlane(pts: List<TrackPoint>): Estimate? {
        val gp = pts.filter { it.ground != null }
        if (gp.size < MIN_TRACK_POINTS) return null
        if (gp.last().t - gp.first().t < MIN_TRACK_SPAN_S) return null
        val fit = LinFit.speed2D(
            gp.map { it.t }.toDoubleArray(),
            gp.map { it.ground!!.lat }.toDoubleArray(),
            gp.map { it.ground!!.depth }.toDoubleArray()
        ) ?: return null
        return Estimate(fit.v * 3.6, fit.se * 3.6, "imu")
    }

    /** Pixel-space fallback scaled by average box width vs typical vehicle length. */
    fun sizeSide(pts: List<TrackPoint>, vehicleLengthM: Double): Estimate? {
        val avgBoxWidth = pts.sumOf { it.w } / pts.size
        if (avgBoxWidth <= 0) return null
        val metersPerPx = vehicleLengthM / avgBoxWidth
        val fit = LinFit.speed2D(
            pts.map { it.t }.toDoubleArray(),
            pts.map { it.x * metersPerPx }.toDoubleArray(),
            pts.map { it.y * metersPerPx }.toDoubleArray()
        ) ?: return null
        return Estimate(fit.v * 3.6, fit.se * 3.6, "size")
    }
}
