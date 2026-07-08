package com.speedcam.core

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.ln
import kotlin.math.sin
import kotlin.math.tan
import kotlin.random.Random

/**
 * These tests mirror the verification vectors used for the web app (index.html), which
 * were validated in-browser. Where a numeric constant appears (e.g. the projectToGround
 * depth), it is the value produced by the JS implementation — the Kotlin port must match.
 */
class CoreMathTest {

    // ===== LinFit =====

    @Test
    fun `linFit recovers an exact line`() {
        val ts = doubleArrayOf(0.0, 0.1, 0.2, 0.3, 0.4, 0.5)
        val vs = ts.map { 2.0 * it + 1.0 }.toDoubleArray()
        val fit = LinFit.fit(ts, vs)!!
        assertEquals(2.0, fit.slope, 1e-9)
        assertEquals(1.0, fit.intercept, 1e-9)
        assertEquals(0.0, fit.se, 1e-9)
    }

    @Test
    fun `linFit rejects degenerate input`() {
        assertNull(LinFit.fit(doubleArrayOf(1.0, 1.0), doubleArrayOf(2.0, 3.0)))
    }

    // ===== Ground-plane speed (36 km/h vector from the web verification) =====

    private fun groundTrack(noise: Double, rng: Random): List<TrackPoint> =
        (0 until 20).map { i ->
            val t = i * 0.05
            fun n() = (rng.nextDouble() - 0.5) * 2 * noise
            TrackPoint(t, 500.0 + i * 10, 400.0, 200.0, 100.0,
                Ground(10.0 * t + n(), 12.0 + n(), 13.0))
        }

    @Test
    fun `ground-plane speed exact track measures 36 kmh with ~0 error`() {
        val est = SpeedEstimator.groundPlane(groundTrack(0.0, Random(1)))!!
        assertEquals(36.0, est.speedKmh, 1e-6)
        assertEquals(0.0, est.errKmh, 1e-6)
        assertEquals("imu", est.source)
    }

    @Test
    fun `ground-plane speed noisy track gets an honest error bar`() {
        val est = SpeedEstimator.groundPlane(groundTrack(0.15, Random(2)))!!
        assertEquals(36.0, est.speedKmh, 3.0)
        assertTrue("errKmh should reflect the noise", est.errKmh > 0.05)
    }

    @Test
    fun `window gating rejects short tracks`() {
        val pts = groundTrack(0.0, Random(3)).take(4)
        assertNull(SpeedEstimator.window(pts, 1.0))
        assertNotNull(SpeedEstimator.window(groundTrack(0.0, Random(3)), 0.95))
    }

    // ===== projectToGround (cross-language regression vs the JS implementation) =====

    @Test
    fun `projectToGround matches the JS implementation`() {
        // Phone upright portrait, camera at 1.5 m, FOV 70°, 1920x1080 image,
        // pixel 260 px below center. JS produced depth = zCam = 7.90974280657171, lat = 0.
        val f = 960.0 / tan(35.0 * PI / 180.0)
        val g = GroundPlane.projectToGround(
            960.0, 800.0, 1920.0, 1080.0, f,
            Vec3(0.0, 1.0, 0.0), 1.5
        )!!
        assertEquals(7.90974280657171, g.depth, 1e-9)
        assertEquals(0.0, g.lat, 1e-9)
        assertEquals(7.90974280657171, g.zCam, 1e-9)
    }

    @Test
    fun `projectToGround rejects rays above the horizon`() {
        val f = 960.0 / tan(35.0 * PI / 180.0)
        assertNull(
            GroundPlane.projectToGround(960.0, 400.0, 1920.0, 1080.0, f, Vec3(0.0, 1.0, 0.0), 1.5)
        )
    }

    @Test
    fun `projectToGround depth scales linearly with camera height`() {
        val f = 960.0 / tan(35.0 * PI / 180.0)
        val g1 = GroundPlane.projectToGround(960.0, 800.0, 1920.0, 1080.0, f, Vec3(0.0, 1.0, 0.0), 1.5)!!
        val g2 = GroundPlane.projectToGround(960.0, 800.0, 1920.0, 1080.0, f, Vec3(0.0, 1.0, 0.0), 3.0)!!
        assertEquals(2.0, g2.depth / g1.depth, 1e-9)
    }

    // ===== Doppler (50.4 km/h synthetic pass, as verified in-browser) =====

    private fun dopplerPass(vTrue: Double, dist: Double, tCa: Double, rng: Random):
        Pair<List<TrackPoint>, List<Doppler.Spectrum>> {
        val n = Doppler.LOG_BINS
        val lstep = ln(Doppler.FMAX / Doppler.FMIN) / (n - 1)

        // Smooth random "tire roar" spectrum from gaussian bumps
        val bumps = List(14) {
            Triple(rng.nextDouble() * n, 4 + rng.nextDouble() * 12, 5 + rng.nextDouble() * 15)
        }
        fun s0(i: Double): Double {
            var v = -70.0
            for ((c, w, a) in bumps) v += a * exp(-((i - c) * (i - c)) / (2 * w * w))
            return v
        }
        fun gof(t: Double): Double {
            val u = vTrue * (t - tCa)
            return u / hypot(dist, u)
        }

        val spectra = ArrayList<Doppler.Spectrum>()
        var t = tCa - 2.0
        while (t <= tCa + 2.0) {
            val shiftBins = ln(Doppler.SOUND_SPEED / (Doppler.SOUND_SPEED + vTrue * gof(t))) / lstep
            val s = DoubleArray(n) { i -> s0(i - shiftBins) + (rng.nextDouble() - 0.5) * 2 }
            spectra.add(Doppler.Spectrum(t, s))
            t += 0.05
        }

        // Vision track covering the approach: road along lat, constant depth
        val track = (0 until 13).map { i ->
            val tt = tCa - 1.4 + i * 0.1
            TrackPoint(tt, 0.0, 0.0, 100.0, 60.0, Ground(vTrue * (tt - tCa), dist, 10.0))
        }
        return track to spectra
    }

    @Test
    fun `doppler recovers a 50 kmh pass within 2 kmh`() {
        val vTrue = 14.0 // 50.4 km/h
        val (track, spectra) = dopplerPass(vTrue, 8.0, 5.0, Random(7))
        val res = Doppler.estimate(track, spectra)!!
        assertEquals(vTrue * 3.6, res.speedKmh, 2.0)
        assertTrue("correlation should be high on clean data", res.corr > 0.9)
        assertEquals(8.0, res.distanceM, 0.01)
    }

    @Test
    fun `doppler works across the speed range`() {
        for (vTrue in listOf(8.0, 20.0, 30.0)) { // 29 / 72 / 108 km/h
            val (track, spectra) = dopplerPass(vTrue, 10.0, 5.0, Random(11))
            val res = Doppler.estimate(track, spectra)
            assertNotNull("failed at ${vTrue * 3.6} km/h", res)
            assertEquals(vTrue * 3.6, res!!.speedKmh, vTrue * 3.6 * 0.06)
        }
    }

    @Test
    fun `doppler rejects uncorrelated noise`() {
        val rng = Random(3)
        val track = (0 until 13).map { i ->
            val t = 3.6 + i * 0.1
            TrackPoint(t, 0.0, 0.0, 100.0, 60.0, Ground(14.0 * (t - 5.0), 8.0, 10.0))
        }
        val spectra = ArrayList<Doppler.Spectrum>()
        var t = 3.0
        while (t <= 7.0) {
            spectra.add(Doppler.Spectrum(t, DoubleArray(Doppler.LOG_BINS) { -70 + rng.nextDouble() * 30 }))
            t += 0.05
        }
        assertNull(Doppler.estimate(track, spectra))
    }

    @Test
    fun `doppler rejects a pass with no recede audio`() {
        val (track, spectraAll) = dopplerPass(14.0, 8.0, 5.0, Random(7))
        val truncated = spectraAll.filter { it.t < 4.9 }
        assertNull(Doppler.estimate(track, truncated))
    }

    // ===== FFT =====

    @Test
    fun `fft finds a 1 kHz tone at the right bin`() {
        val sr = 48000.0
        val n = 4096
        val samples = DoubleArray(n) { i -> sin(2.0 * PI * 1000.0 * i / sr) }
        val mag = Fft.magnitudeDb(samples)
        val peak = mag.indices.maxBy { mag[it] }
        val expected = (1000.0 / sr * n).toInt() // bin ~85
        assertTrue("peak at $peak, expected ~$expected", kotlin.math.abs(peak - expected) <= 1)
        // Peak should stand well above the median (windowed noise floor)
        val median = mag.sorted()[mag.size / 2]
        assertTrue(mag[peak] - median > 40)
    }

    @Test
    fun `logResample maps band edges correctly`() {
        val sr = 48000.0
        val nBins = 2048
        // Spectrum with spikes at FMIN and FMAX (2 bins wide so linear interpolation
        // between adjacent bins can't attenuate them)
        val mag = DoubleArray(nBins) { -90.0 }
        val binMin = (Doppler.FMIN / (sr / 2) * nBins).toInt()
        val binMax = (Doppler.FMAX / (sr / 2) * nBins).toInt()
        mag[binMin] = 0.0; mag[binMin + 1] = 0.0
        mag[binMax] = 0.0; mag[binMax + 1] = 0.0
        val log = Doppler.logResample(mag, sr)
        assertTrue("FMIN spike should land at the first log bin", log[0] > -45.0)
        assertTrue("FMAX spike should land at the last log bin", log[Doppler.LOG_BINS - 1] > -45.0)
    }
}
