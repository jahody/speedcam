package com.speedcam.app

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.util.AttributeSet
import android.view.View

/** Draws the tracked vehicle box on top of the camera preview. */
class OverlayView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null
) : View(context, attrs) {

    private val boxPaint = Paint().apply {
        style = Paint.Style.STROKE
        strokeWidth = 6f
        color = Color.parseColor("#00e676")
    }

    /** Box in analysis-image coordinates; [imageW]/[imageH] are the upright analysis size. */
    var box: RectF? = null
    var imageW = 1
    var imageH = 1
    var overLimit = false

    fun update(box: RectF?, imageW: Int, imageH: Int, overLimit: Boolean) {
        this.box = box
        this.imageW = imageW
        this.imageH = imageH
        this.overLimit = overLimit
        postInvalidate()
    }

    override fun onDraw(canvas: Canvas) {
        val b = box ?: return
        boxPaint.color = Color.parseColor(if (overLimit) "#ff1744" else "#00e676")
        // PreviewView uses FILL_CENTER; map image coords via center-crop scale
        val scale = maxOf(width.toFloat() / imageW, height.toFloat() / imageH)
        val dx = (width - imageW * scale) / 2f
        val dy = (height - imageH * scale) / 2f
        canvas.drawRect(
            RectF(
                b.left * scale + dx, b.top * scale + dy,
                b.right * scale + dx, b.bottom * scale + dy
            ),
            boxPaint
        )
    }
}
