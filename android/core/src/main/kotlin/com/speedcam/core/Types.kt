package com.speedcam.core

import kotlin.math.sqrt

/** Position of a tracked vehicle projected onto the road plane, in meters.
 *  lat = sideways, depth = horizontal distance ahead, zCam = distance along the
 *  camera axis (used for size-based estimates). */
data class Ground(val lat: Double, val depth: Double, val zCam: Double)

/** One tracked sample. [t] is capture time in seconds on a monotonic clock shared
 *  with the audio pipeline (Android: SystemClock.elapsedRealtimeNanos / 1e9).
 *  x/y/w/h are pixels in the upright analysis image. */
data class TrackPoint(
    val t: Double,
    val x: Double,
    val y: Double,
    val w: Double,
    val h: Double,
    val ground: Ground?
)

/** A speed measurement with its 1-sigma uncertainty, both in km/h. */
data class Estimate(val speedKmh: Double, val errKmh: Double, val source: String)

data class Vec3(val x: Double, val y: Double, val z: Double) {
    fun normalized(): Vec3 {
        val n = sqrt(x * x + y * y + z * z)
        return Vec3(x / n, y / n, z / n)
    }
}
