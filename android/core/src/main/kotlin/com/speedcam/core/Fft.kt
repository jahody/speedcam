package com.speedcam.core

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.log10
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

/** Minimal radix-2 FFT for the audio Doppler spectra (power-of-two input). */
object Fft {

    /** Hann-windowed magnitude spectrum in dB. Input length must be a power of two;
     *  returns length/2 bins (DC..Nyquist-1). */
    fun magnitudeDb(samples: DoubleArray): DoubleArray {
        val n = samples.size
        require(n > 0 && (n and (n - 1)) == 0) { "FFT size must be a power of two" }
        val re = DoubleArray(n)
        val im = DoubleArray(n)
        for (i in 0 until n) {
            val w = 0.5 * (1 - cos(2.0 * PI * i / (n - 1))) // Hann
            re[i] = samples[i] * w
        }
        fft(re, im)
        val out = DoubleArray(n / 2)
        for (i in 0 until n / 2) {
            val mag = sqrt(re[i] * re[i] + im[i] * im[i]) / n
            out[i] = 20.0 * log10(max(mag, 1e-12))
        }
        return out
    }

    /** In-place iterative Cooley-Tukey radix-2 FFT. */
    fun fft(re: DoubleArray, im: DoubleArray) {
        val n = re.size
        // Bit-reversal permutation
        var j = 0
        for (i in 0 until n - 1) {
            if (i < j) {
                var tmp = re[i]; re[i] = re[j]; re[j] = tmp
                tmp = im[i]; im[i] = im[j]; im[j] = tmp
            }
            var m = n shr 1
            while (m in 1..j) { j -= m; m = m shr 1 }
            j += m
        }
        // Butterflies
        var len = 2
        while (len <= n) {
            val ang = -2.0 * PI / len
            val wr = cos(ang)
            val wi = sin(ang)
            var i = 0
            while (i < n) {
                var cwr = 1.0
                var cwi = 0.0
                for (k in 0 until len / 2) {
                    val a = i + k
                    val b = i + k + len / 2
                    val tr = re[b] * cwr - im[b] * cwi
                    val ti = re[b] * cwi + im[b] * cwr
                    re[b] = re[a] - tr
                    im[b] = im[a] - ti
                    re[a] += tr
                    im[a] += ti
                    val nwr = cwr * wr - cwi * wi
                    cwi = cwr * wi + cwi * wr
                    cwr = nwr
                }
                i += len
            }
            len = len shl 1
        }
    }
}
