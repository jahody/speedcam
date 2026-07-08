package com.speedcam.app

import android.annotation.SuppressLint
import android.media.AudioFormat
import android.media.AudioRecord
import android.media.MediaRecorder
import android.os.SystemClock
import com.speedcam.core.Doppler
import com.speedcam.core.Fft
import java.util.ArrayDeque

/**
 * Microphone -> rolling buffer of log-frequency spectra for the Doppler estimator.
 * Uses UNPROCESSED audio when available so AGC/noise suppression don't fight the
 * measurement. Timestamps use elapsedRealtimeNanos — the same clock CameraX sensor
 * timestamps use on devices with a REALTIME timestamp source.
 */
class AudioDoppler {

    companion object {
        const val SAMPLE_RATE = 48000
        const val FFT_SIZE = 4096
        const val HOP = 2048               // ~43 ms between spectra
        const val BUFFER_S = 8.0
    }

    private var record: AudioRecord? = null
    private var thread: Thread? = null
    @Volatile private var running = false
    private val spectra = ArrayDeque<Doppler.Spectrum>()

    val active: Boolean get() = running

    @SuppressLint("MissingPermission") // caller checks RECORD_AUDIO
    fun start() {
        if (running) return
        val minBuf = AudioRecord.getMinBufferSize(
            SAMPLE_RATE, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT
        )
        if (minBuf <= 0) return
        fun make(source: Int): AudioRecord? = try {
            AudioRecord(
                source, SAMPLE_RATE, AudioFormat.CHANNEL_IN_MONO,
                AudioFormat.ENCODING_PCM_16BIT, maxOf(minBuf, FFT_SIZE * 4)
            ).also { if (it.state != AudioRecord.STATE_INITIALIZED) { it.release() } }
                .takeIf { it.state == AudioRecord.STATE_INITIALIZED }
        } catch (e: Exception) {
            null
        }
        // UNPROCESSED avoids AGC/noise suppression; fall back to MIC where unsupported
        val rec = make(MediaRecorder.AudioSource.UNPROCESSED)
            ?: make(MediaRecorder.AudioSource.MIC)
            ?: return
        record = rec
        running = true
        rec.startRecording()

        thread = Thread {
            val ring = DoubleArray(FFT_SIZE)
            val hop = ShortArray(HOP)
            var filled = 0
            while (running) {
                val n = rec.read(hop, 0, HOP)
                if (n <= 0) continue
                // Slide the ring buffer and append the new hop
                System.arraycopy(ring, n, ring, 0, FFT_SIZE - n)
                for (i in 0 until n) ring[FFT_SIZE - n + i] = hop[i] / 32768.0
                filled += n
                if (filled < FFT_SIZE) continue

                val t = SystemClock.elapsedRealtimeNanos() / 1e9
                val mag = Fft.magnitudeDb(ring.copyOf())
                val logSpec = Doppler.logResample(mag, SAMPLE_RATE.toDouble())
                synchronized(spectra) {
                    spectra.addLast(Doppler.Spectrum(t, logSpec))
                    while (spectra.isNotEmpty() && t - spectra.first().t > BUFFER_S) {
                        spectra.removeFirst()
                    }
                }
            }
        }.also { it.priority = Thread.NORM_PRIORITY; it.start() }
    }

    fun stop() {
        running = false
        thread?.join(500)
        thread = null
        record?.let { it.stop(); it.release() }
        record = null
        synchronized(spectra) { spectra.clear() }
    }

    fun snapshot(): List<Doppler.Spectrum> = synchronized(spectra) { spectra.toList() }
}
