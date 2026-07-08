package com.speedcam.core

import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sqrt

data class Fit(val slope: Double, val se: Double, val intercept: Double)

data class Speed2D(val v: Double, val se: Double)

/** Least-squares fitting used by all estimators. Mirrors linFit/fitSpeed2D in the web app. */
object LinFit {

    /** Slope of v against t with the slope's standard error, or null when degenerate. */
    fun fit(ts: DoubleArray, vs: DoubleArray): Fit? {
        val n = ts.size
        if (n < 2 || vs.size != n) return null
        var tm = 0.0
        var vm = 0.0
        for (i in 0 until n) { tm += ts[i]; vm += vs[i] }
        tm /= n; vm /= n
        var stt = 0.0
        var stv = 0.0
        for (i in 0 until n) {
            val dt = ts[i] - tm
            stt += dt * dt
            stv += dt * (vs[i] - vm)
        }
        if (stt < 1e-9) return null
        val slope = stv / stt
        var ssr = 0.0
        for (i in 0 until n) {
            val r = (vs[i] - vm) - slope * (ts[i] - tm)
            ssr += r * r
        }
        val se = sqrt(ssr / max(1, n - 2) / stt)
        return Fit(slope, se, vm - slope * tm)
    }

    /** 2D speed (units of x,y per second) from independent x(t), y(t) fits;
     *  error combined via the delta method. */
    fun speed2D(ts: DoubleArray, xs: DoubleArray, ys: DoubleArray): Speed2D? {
        val fx = fit(ts, xs) ?: return null
        val fy = fit(ts, ys) ?: return null
        val v = hypot(fx.slope, fy.slope)
        val se = if (v > 1e-6) hypot(fx.slope * fx.se, fy.slope * fy.se) / v
                 else hypot(fx.se, fy.se)
        return Speed2D(v, se)
    }
}
