package com.speedcam.app

import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraMetadata
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.core.CameraInfo

/**
 * Exact camera intrinsics from Camera2 — the whole point of the native app: no FOV
 * guessing or statistical tuning, the geometry is simply known.
 */
object CameraGeometry {

    data class Intrinsics(
        val focalMm: Double,
        val sensorWidthMm: Double,
        val timestampsAreRealtime: Boolean
    ) {
        /** Focal length in pixels of an image whose long side is [longSidePx]
         *  (16:9 video streams use the sensor's full width). */
        fun focalPx(longSidePx: Int): Double = focalMm / sensorWidthMm * longSidePx
    }

    @OptIn(ExperimentalCamera2Interop::class)
    fun read(cameraInfo: CameraInfo): Intrinsics? {
        val c2 = Camera2CameraInfo.from(cameraInfo)
        val focal = c2.getCameraCharacteristic(
            CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS
        )?.firstOrNull() ?: return null
        val phys = c2.getCameraCharacteristic(
            CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE
        ) ?: return null
        val tsSource = c2.getCameraCharacteristic(
            CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE
        )
        return Intrinsics(
            focal.toDouble(),
            phys.width.toDouble(),
            tsSource == CameraMetadata.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME
        )
    }
}
