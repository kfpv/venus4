package com.surendramaran.yolov8tflite

data class BoundingBox(
    val x1: Float,
    val y1: Float,
    val x2: Float,
    val y2: Float,
    val cx: Float,
    val cy: Float,
    val w: Float,
    val h: Float,
    val cnf: Float,
    val cls: Int,
    val clsName: String,
    val cubeType: CubeType
) {
    fun getExpandedBounds(expansionFactor: Float): ExpandedBounds {
        val expandedW = w * (1 + expansionFactor)
        val expandedH = h * (1 + expansionFactor)
        val expandedX1 = cx - (expandedW / 2f)
        val expandedY1 = cy - (expandedH / 2f)
        val expandedX2 = cx + (expandedW / 2f)
        val expandedY2 = cy + (expandedH / 2f)

        return ExpandedBounds(
            x1 = expandedX1.coerceIn(0f, 1f),
            y1 = expandedY1.coerceIn(0f, 1f),
            x2 = expandedX2.coerceIn(0f, 1f),
            y2 = expandedY2.coerceIn(0f, 1f)
        )
    }
    
    fun containsCoordinate(targetX: Int, targetY: Int, imageWidth: Int, imageHeight: Int): Boolean {
        val pixelX1 = (x1 * imageWidth).toInt()
        val pixelY1 = (y1 * imageHeight).toInt()
        val pixelX2 = (x2 * imageWidth).toInt()
        val pixelY2 = (y2 * imageHeight).toInt()
        
        return targetX >= pixelX1 && targetX <= pixelX2 && targetY >= pixelY1 && targetY <= pixelY2
    }
}

data class ExpandedBounds(
    val x1: Float,
    val y1: Float,
    val x2: Float,
    val y2: Float
)

enum class CubeType {
    WHITE, BLACK, RED, GREEN, BLUE, BOX, ROBOT
}
