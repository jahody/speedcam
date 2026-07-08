package com.speedcam.core

import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.ln
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Audio Doppler speed estimation — port of the web app's Doppler module.
 *
 * A passing car's broadband noise shifts down in pitch; in log-frequency space the
 * shift is a pure translation, so cross-correlating the averaged spectrum before vs
 * after closest approach yields the frequency ratio R. With the pass geometry from
 * the vision track: v = c(R-1)/(mu(R+1)).
 */
object Doppler {

    const val SOUND_SPEED = 343.0        // m/s at ~20°C
    const val FMIN = 300.0               // analysis band (Hz) — tire/engine noise
    const val FMAX = 8000.0
    const val LOG_BINS = 256             // log-frequency resampling resolution
    const val MAX_LAG = 25               // correlation search range (~±180 km/h)
    const val WIN_GAP_S = 0.25           // skip around closest approach (shift crosses zero)
    const val WIN_LEN_S = 1.25           // approach/recede window length
    const val MIN_SPECTRA = 5            // min spectra per window
    const val MIN_MU = 0.25              // min |geometry factor|
    const val MIN_CORR = 0.4             // min spectral correlation to trust the shift

    /** One spectrum snapshot: [t] in seconds (same clock as TrackPoint.t),
     *  [s] on the uniform log-frequency grid (LOG_BINS values, dB-like scale). */
    class Spectrum(val t: Double, val s: DoubleArray)

    data class Result(val speedKmh: Double, val corr: Double, val mu: Double, val distanceM: Double)

    data class ShiftResult(val shift: Double, val corr: Double)

    /** Resample a linear-frequency magnitude spectrum onto the uniform log grid. */
    fun logResample(mag: DoubleArray, sampleRate: Double): DoubleArray {
        val nb = mag.size
        val lmin = ln(FMIN)
        val lmax = ln(min(FMAX, sampleRate / 2.0))
        val s = DoubleArray(LOG_BINS)
        for (i in 0 until LOG_BINS) {
            val f = exp(lmin + (lmax - lmin) * i / (LOG_BINS - 1))
            val b = f / (sampleRate / 2.0) * nb
            val b0 = b.toInt()
            val fr = b - b0
            s[i] = mag[b0] * (1 - fr) + mag[min(nb - 1, b0 + 1)] * fr
        }
        return s
    }

    /** Average a list of spectra and remove the mean (correlation cares about shape only). */
    fun avgSpectrum(list: List<Spectrum>): DoubleArray {
        val n = list[0].s.size
        val m = DoubleArray(n)
        val cnt = list.size.toDouble()
        for (sp in list) for (i in 0 until n) m[i] += sp.s[i]
        var mean = 0.0
        for (i in 0 until n) { m[i] /= cnt; mean += m[i] }
        mean /= n
        for (i in 0 until n) m[i] -= mean
        return m
    }

    /** Best shift of A relative to B in log bins (positive = A higher pitch),
     *  by normalized cross-correlation with sub-bin parabolic refinement. */
    fun logShift(a: DoubleArray, b: DoubleArray): ShiftResult {
        val n = a.size
        fun corrAt(lag: Int): Double {
            var sa = 0.0; var sb = 0.0; var sab = 0.0; var saa = 0.0; var sbb = 0.0
            var cnt = 0
            for (i in 0 until n) {
                val j = i - lag
                if (j < 0 || j >= n) continue
                val av = a[i]; val bv = b[j]
                sa += av; sb += bv; sab += av * bv; saa += av * av; sbb += bv * bv
                cnt++
            }
            if (cnt < 32) return -2.0
            val cov = sab - sa * sb / cnt
            val va = saa - sa * sa / cnt
            val vb = sbb - sb * sb / cnt
            return if (va > 0 && vb > 0) cov / sqrt(va * vb) else -2.0
        }

        val cs = DoubleArray(2 * MAX_LAG + 1)
        var best = 0
        var bestC = -2.0
        for (lag in -MAX_LAG..MAX_LAG) {
            val c = corrAt(lag)
            cs[lag + MAX_LAG] = c
            if (c > bestC) { bestC = c; best = lag }
        }
        var delta = 0.0
        val i = best + MAX_LAG
        if (i > 0 && i < cs.size - 1) {
            val den = cs[i - 1] - 2 * cs[i] + cs[i + 1]
            if (abs(den) > 1e-9) delta = 0.5 * (cs[i - 1] - cs[i + 1]) / den
        }
        return ShiftResult(best + delta, bestC)
    }

    /**
     * Doppler speed for a finished pass, or null when the measurement isn't trustworthy.
     * [groundPts] are the track points that carry a ground projection (any subset works);
     * their [TrackPoint.t] must share a clock with the [spectra] timestamps.
     */
    fun estimate(
        groundPts: List<TrackPoint>,
        spectra: List<Spectrum>,
        minSpeedKmh: Double = SpeedEstimator.MIN_SPEED_KMH,
        maxSpeedKmh: Double = SpeedEstimator.MAX_SPEED_KMH
    ): Result? {
        val gp = groundPts.filter { it.ground != null }
        if (gp.size < SpeedEstimator.MIN_TRACK_POINTS) return null

        // Straight-line constant-velocity fit of the pass on the ground plane
        val ts = gp.map { it.t }.toDoubleArray()
        val fl = LinFit.fit(ts, gp.map { it.ground!!.lat }.toDoubleArray()) ?: return null
        val fd = LinFit.fit(ts, gp.map { it.ground!!.depth }.toDoubleArray()) ?: return null
        val v = hypot(fl.slope, fd.slope)
        if (v < 1.0) return null

        // Closest approach: point on the fitted line nearest the camera (origin)
        val tCa = -(fl.intercept * fl.slope + fd.intercept * fd.slope) / (v * v)
        val d = hypot(fl.intercept + fl.slope * tCa, fd.intercept + fd.slope * tCa)
        if (d <= 0.5) return null

        // Signed geometry factor: range rate = v * gof(t); negative approaching
        fun gof(t: Double): Double {
            val u = v * (t - tCa)
            return u / hypot(d, u)
        }

        val winA = (tCa - WIN_GAP_S - WIN_LEN_S)..(tCa - WIN_GAP_S)
        val winB = (tCa + WIN_GAP_S)..(tCa + WIN_GAP_S + WIN_LEN_S)
        val a = spectra.filter { it.t in winA }
        val b = spectra.filter { it.t in winB }
        if (a.size < MIN_SPECTRA || b.size < MIN_SPECTRA) return null

        fun muOf(list: List<Spectrum>) = list.sumOf { abs(gof(it.t)) } / list.size
        val mu = (muOf(a) + muOf(b)) / 2.0
        if (mu < MIN_MU) return null

        val (shift, corr) = logShift(avgSpectrum(a), avgSpectrum(b))
        if (corr < MIN_CORR) return null
        val lstep = ln(FMAX / FMIN) / (LOG_BINS - 1)
        val r = exp(shift * lstep) // f_approach / f_recede
        if (r <= 1.0) return null  // approach must be higher pitch
        val speedKmh = SOUND_SPEED * (r - 1) / (mu * (r + 1)) * 3.6
        if (speedKmh < minSpeedKmh || speedKmh > maxSpeedKmh) return null
        return Result(speedKmh, corr, mu, d)
    }
}
