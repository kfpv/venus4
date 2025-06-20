package com.surendramaran.yolov8tflite

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.View

class OverlayView(context: Context?, attrs: AttributeSet?) : View(context, attrs) {
    private var results = listOf<BoundingBox>()
    private var tapePolygons = listOf<TapePolygon>()
    private var debugMaskBitmap: Bitmap? = null
    private var isDebugMode = false
    private var showContourDebug = false
    private var boxPaint = Paint()
    private var textBackgroundPaint = Paint()
    private var textPaint = Paint()
    private var tapeMaskPaint = Paint()
    private var centerLinePaint = Paint()
    private var deadZonePaint = Paint()
    private var bounds = Rect()
    private var overlaysEnabled = true
    private var currentCameraId: String? = null
    private var currentZoomRatio: Float? = null
    init {
        initPaints()
    }
    fun clear() {
        results = listOf()
        tapePolygons = listOf()
        invalidate()
    }
    private fun initPaints() {
        textBackgroundPaint.color = Color.BLACK
        textBackgroundPaint.style = Paint.Style.FILL
        textBackgroundPaint.textSize = 50f
        textPaint.color = Color.WHITE
        textPaint.style = Paint.Style.FILL
        textPaint.textSize = 50f
        boxPaint.strokeWidth = 8F
        boxPaint.style = Paint.Style.STROKE
        tapeMaskPaint.color = Color.YELLOW
        tapeMaskPaint.alpha = 128
        tapeMaskPaint.style = Paint.Style.FILL
        centerLinePaint.color = Color.WHITE
        centerLinePaint.strokeWidth = 5F
        centerLinePaint.style = Paint.Style.STROKE
        deadZonePaint.color = Color.RED
        deadZonePaint.alpha = 64
        deadZonePaint.style = Paint.Style.FILL
    }
    fun setOverlaysEnabled(enabled: Boolean) {
        overlaysEnabled = enabled
        invalidate()
    }
    fun setDebugMode(enabled: Boolean) {
        isDebugMode = enabled
        invalidate()
    }
    fun setContourDebugMode(enabled: Boolean) {
        showContourDebug = enabled
        invalidate()
    }
    fun setDebugMask(bitmap: Bitmap?) {
        debugMaskBitmap?.recycle()
        debugMaskBitmap = bitmap
        invalidate()
    }
    fun setTapePolygons(polygons: List<TapePolygon>) {
        tapePolygons = polygons
        invalidate()
    }
    fun setCameraState(cameraId: String?, zoomRatio: Float?) {
        currentCameraId = cameraId
        currentZoomRatio = zoomRatio
        invalidate()
    }
    @Deprecated("Use setTapePolygons instead")
    fun setTapeContours(contours: List<TapeContour>) {
        tapePolygons = contours.map { TapePolygon(it.points, it.area) }
        invalidate()
    }
    fun getCurrentResults(): List<BoundingBox> {
        return results
    }
    override fun draw(canvas: Canvas) {
        super.draw(canvas)
        if (isDebugMode && debugMaskBitmap != null) {
            val srcRect = Rect(0, 0, debugMaskBitmap!!.width, debugMaskBitmap!!.height)
            val dstRect = Rect(0, 0, width, height)
            canvas.drawBitmap(debugMaskBitmap!!, srcRect, dstRect, null)
            val debugText = if (showContourDebug) {
                "Debug Contours: Red=TooSmall, Green=Valid, Magenta=TooLarge"
            } else {
                "Debug Tape Mask"
            }
            textBackgroundPaint.getTextBounds(debugText, 0, debugText.length, bounds)
            val textWidth = bounds.width()
            val textHeight = bounds.height()
            canvas.drawRect(
                16f,
                height - textHeight - 32f,
                16f + textWidth + BOUNDING_RECT_TEXT_PADDING,
                height - 16f,
                textBackgroundPaint
            )
            canvas.drawText(
                debugText,
                16f,
                height - 32f,
                textPaint
            )
            return
        }
        if (!overlaysEnabled) {
            canvas.drawLine(width / 2f, 0f, width / 2f, height.toFloat(), centerLinePaint)
            return
        }
        results.forEach { boundingBox ->
            val color = getCubeColor(boundingBox.cubeType)
            boxPaint.color = color
            textBackgroundPaint.color = Color.argb(180, 0, 0, 0)
            val left = boundingBox.x1 * width
            val top = boundingBox.y1 * height
            val right = boundingBox.x2 * width
            val bottom = boundingBox.y2 * height
            canvas.drawRect(left, top, right, bottom, boxPaint)
            val drawableText = "${boundingBox.clsName} ${Math.round(boundingBox.cnf * 100)}%"
            textBackgroundPaint.getTextBounds(drawableText, 0, drawableText.length, bounds)
            val textWidth = bounds.width()
            val textHeight = bounds.height()
            val textBackgroundRect = RectF(
                left,
                top,
                left + textWidth + BOUNDING_RECT_TEXT_PADDING,
                top + textHeight + BOUNDING_RECT_TEXT_PADDING
            )
            canvas.drawRect(textBackgroundRect, textBackgroundPaint)
            canvas.drawText(drawableText, left, top + bounds.height(), textPaint)
        }
        tapePolygons.forEach { tapePolygon ->
            val path = Path()
            if (tapePolygon.points.isNotEmpty()) {
                val firstPoint = tapePolygon.points[0]
                path.moveTo((firstPoint.x * width).toFloat(), (firstPoint.y * height).toFloat())
                for (i in 1 until tapePolygon.points.size) {
                    val point = tapePolygon.points[i]
                    path.lineTo((point.x * width).toFloat(), (point.y * height).toFloat())
                }
                path.close()
                canvas.drawPath(path, tapeMaskPaint)
            }
        }
        val deadZones = Constants.getPhoneDeadZones(currentCameraId, currentZoomRatio)
        deadZones.forEach { deadZonePoints ->
            val path = Path()
            if (deadZonePoints.isNotEmpty()) {
                val firstPoint = deadZonePoints[0]
                path.moveTo((firstPoint.x * width).toFloat(), (firstPoint.y * height).toFloat())
                for (i in 1 until deadZonePoints.size) {
                    val point = deadZonePoints[i]
                    path.lineTo((point.x * width).toFloat(), (point.y * height).toFloat())
                }
                path.close()
                canvas.drawPath(path, deadZonePaint)
            }
        }
    }
    private fun getCubeColor(cubeType: CubeType): Int {
        return when (cubeType) {
            CubeType.WHITE -> Color.WHITE
            CubeType.BLACK -> Color.rgb(64, 64, 64)
            CubeType.RED -> Color.RED
            CubeType.GREEN -> Color.GREEN
            CubeType.BLUE -> Color.BLUE
            CubeType.BOX -> Color.rgb(255, 165, 0)
            CubeType.ROBOT -> Color.rgb(128, 0, 128)
        }
    }
    fun setResults(boundingBoxes: List<BoundingBox>) {
        results = boundingBoxes
        invalidate()
    }
    companion object {
        private const val BOUNDING_RECT_TEXT_PADDING = 8
    }
}