package com.speedcam.core

import kotlin.math.PI
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Pinhole ray / ground-plane intersection — direct port of projectToGround in the web app.
 *
 * Camera frame convention (matches an upright portrait image on Android):
 *   x right, y up (image up), camera looks along -z.
 * [up] is the unit gravity-up vector expressed in this camera frame. With the activity
 * locked to portrait, Android's TYPE_GRAVITY vector (device coords) is already in this
 * frame after normalization.
 */
object GroundPlane {

    /** Rays within 2 degrees of the horizon are unreliable. */
    val MIN_GRAZE_SIN: Double = sin(2.0 * PI / 180.0)
    const val MAX_GROUND_RANGE_M = 60.0

    /**
     * Map an image pixel (vx, vy — origin top-left, y down) to the road plane.
     * Returns null when the ray is unreliable (near/above horizon, or out of range).
     *
     * @param focalPx focal length in pixels of the analysis image
     * @param cameraHeightM camera height above the road surface
     */
    fun projectToGround(
        vx: Double, vy: Double,
        imageW: Double, imageH: Double,
        focalPx: Double,
        up: Vec3,
        cameraHeightM: Double,
        maxRangeM: Double = MAX_GROUND_RANGE_M
    ): Ground? {
        if (cameraHeightM <= 0) return null

        // Pixel offset from the principal point, up-positive
        val sx = vx - imageW / 2.0
        val sy = -(vy - imageH / 2.0)

        // Camera looks along -z
        var rx = sx
        var ry = sy
        var rz = -focalPx
        val rn = sqrt(rx * rx + ry * ry + rz * rz)
        rx /= rn; ry /= rn; rz /= rn

        val vert = rx * up.x + ry * up.y + rz * up.z
        if (vert > -MIN_GRAZE_SIN) return null
        val t = cameraHeightM / (-vert)

        // Horizontal component of the ray and a horizontal basis (forward = camera axis
        // projected onto the ground, right = up x forward)
        val hx = rx - vert * up.x
        val hy = ry - vert * up.y
        val hz = rz - vert * up.z
        val fwx = up.z * up.x
        val fwy = up.z * up.y
        val fwz = up.z * up.z - 1.0
        val fm = sqrt(fwx * fwx + fwy * fwy + fwz * fwz)
        if (fm < 0.1) return null
        val e1x = fwx / fm; val e1y = fwy / fm; val e1z = fwz / fm
        val e2x = up.y * e1z - up.z * e1y
        val e2y = up.z * e1x - up.x * e1z
        val e2z = up.x * e1y - up.y * e1x

        val depth = t * (hx * e1x + hy * e1y + hz * e1z)
        val lat = t * (hx * e2x + hy * e2y + hz * e2z)
        if (depth < 0.5 || sqrt(depth * depth + lat * lat) > maxRangeM) return null
        return Ground(lat, depth, t * (focalPx / rn))
    }
}
