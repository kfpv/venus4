package com.surendramaran.yolov8tflite

import android.graphics.Bitmap
import org.opencv.android.Utils
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import kotlin.math.pow

class TapeDetector {
    private val hsvLower = Scalar(0.0, 0.0, 0.0)
    private val hsvUpper = Scalar(180.0, 255.0, 80.0)
    private val minArea = 500.0
    private val maxArea = 500000.0
    private val blurKernel = 3
    private val morphologyKernel = 2
    private val shadowCompensation = 1.20
    private val adaptiveThresh = true
    private val dilationIterations = 1
    private val dilationKernelSize = 5

    fun detectTapeObjects(bitmap: Bitmap, cubeBounds: List<ExpandedBounds> = emptyList(), cameraId: String? = null, zoomRatio: Float? = null): List<TapePolygon> {
        val mat = Mat()
        Utils.bitmapToMat(bitmap, mat)
        
        val compensated = Mat()
        mat.convertTo(compensated, CvType.CV_8UC3, shadowCompensation, 0.0)
        
        val blurred = Mat()
        val kernelSize = Size(blurKernel.toDouble(), blurKernel.toDouble())
        Imgproc.GaussianBlur(compensated, blurred, kernelSize, 0.0)
        
        val masks = mutableListOf<Mat>()
        
        val hsv = Mat()
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_BGR2HSV)
        val hsvMask = Mat()
        Core.inRange(hsv, hsvLower, hsvUpper, hsvMask)
        masks.add(hsvMask)
        
        if (adaptiveThresh) {
            val gray = Mat()
            Imgproc.cvtColor(blurred, gray, Imgproc.COLOR_BGR2GRAY)
            val adaptiveMask = Mat()
            Imgproc.adaptiveThreshold(gray, adaptiveMask, 255.0, 
                Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY_INV, 21, 10.0)
            masks.add(adaptiveMask)
        }
        
        val finalMask = masks[0].clone()
        for (i in 1 until masks.size) {
            Core.bitwise_or(finalMask, masks[i], finalMask)
        }
        
        if (cubeBounds.isNotEmpty()) {
            val imageWidth = bitmap.width
            val imageHeight = bitmap.height
            
            for (expandedBounds in cubeBounds) {
                val x1 = (expandedBounds.x1 * imageWidth).toInt().coerceIn(0, imageWidth - 1)
                val y1 = (expandedBounds.y1 * imageHeight).toInt().coerceIn(0, imageHeight - 1)
                val x2 = (expandedBounds.x2 * imageWidth).toInt().coerceIn(0, imageWidth - 1)
                val y2 = (expandedBounds.y2 * imageHeight).toInt().coerceIn(0, imageHeight - 1)
                
                val rect = Rect(x1, y1, x2 - x1, y2 - y1)
                Imgproc.rectangle(finalMask, rect, Scalar(0.0), -1)
            }
        }
        
        val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, 
            Size(morphologyKernel.toDouble(), morphologyKernel.toDouble()))
        Imgproc.morphologyEx(finalMask, finalMask, Imgproc.MORPH_CLOSE, kernel)
        Imgproc.morphologyEx(finalMask, finalMask, Imgproc.MORPH_OPEN, kernel)
        
        val erosionKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.0, 3.0))
        
        
        val dilationKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
            Size(dilationKernelSize.toDouble(), dilationKernelSize.toDouble()))
        
        applyPhoneDeadZoneMask(finalMask, bitmap.width, bitmap.height, cameraId, zoomRatio)
        
        fillAllDetectedAreas(finalMask, bitmap.width, bitmap.height)
        
        val contours = mutableListOf<MatOfPoint>()
        val hierarchy = Mat()
        Imgproc.findContours(finalMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)
        
        val filteredContours = mutableListOf<TapePolygon>()
        val imageWidth = bitmap.width.toFloat()
        val imageHeight = bitmap.height.toFloat()
        
        var totalContours = 0
        var areaFilteredOut = 0
        var edgeTouchingContours = 0
        
        for (contour in contours) {
            totalContours++
            val area = Imgproc.contourArea(contour)
            
            val touchesEdge = checkIfContourTouchesEdge(contour, bitmap.width, bitmap.height)
            
            if (touchesEdge) {
                edgeTouchingContours++
                if (area < minArea * 0.3) { 
                    areaFilteredOut++
                    continue
                }
            } else {
                if (area < minArea) {
                    areaFilteredOut++
                    continue
                }
                if (area > maxArea) {
                    areaFilteredOut++
                    continue
                }
            }
            
            val epsilon = Constants.POLYGON_APPROXIMATION_ACCURACY * Imgproc.arcLength(MatOfPoint2f(*contour.toArray()), true)
            val approxCurve = MatOfPoint2f()
            Imgproc.approxPolyDP(MatOfPoint2f(*contour.toArray()), approxCurve, epsilon, true)
            
            val polygonPoints = approxCurve.toArray()
            val normalizedPoints = polygonPoints.map { point ->
                Point(
                    (point.x / imageWidth).coerceIn(0.0, 1.0), 
                    (point.y / imageHeight).coerceIn(0.0, 1.0)
                )
            }.toTypedArray()
            
            
            if (normalizedPoints.size >= 3) {
                filteredContours.add(TapePolygon(normalizedPoints, area))
            }
            
            
            approxCurve.release()
        }
        
        
        android.util.Log.d("TapeDetector", "Total contours found: $totalContours")
        android.util.Log.d("TapeDetector", "Edge-touching contours: $edgeTouchingContours")
        android.util.Log.d("TapeDetector", "Area filtered out: $areaFilteredOut")
        android.util.Log.d("TapeDetector", "Final contours: ${filteredContours.size}")
        
        
        mat.release()
        compensated.release()
        blurred.release()
        hsv.release()
        finalMask.release()
        masks.forEach { it.release() }
        hierarchy.release()
        contours.forEach { it.release() }
        
        return filteredContours
    }

    private fun checkIfContourTouchesEdge(contour: MatOfPoint, imageWidth: Int, imageHeight: Int): Boolean {
        val points = contour.toArray()
        val edgeThreshold = 10.0 
        
        for (point in points) {
            if (point.x <= edgeThreshold || 
                point.x >= imageWidth - edgeThreshold ||
                point.y <= edgeThreshold || 
                point.y >= imageHeight - edgeThreshold) {
                return true
            }
        }
        return false
    }
    
    private fun fillAllDetectedAreas(mask: Mat, imageWidth: Int, imageHeight: Int) {
        
        val closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(5.0, 5.0))
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, closeKernel)
        
        
        val dilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.0, 3.0))
        Imgproc.dilate(mask, mask, dilateKernel, Point(-1.0, -1.0), 2)
        
        
        val tempContours = mutableListOf<MatOfPoint>()
        val tempHierarchy = Mat()
        Imgproc.findContours(mask, tempContours, tempHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)
        
        
        for (contour in tempContours) {
            val area = Imgproc.contourArea(contour)
            
            if (area > 100.0) { 
                
                val polygons = listOf(contour)
                Imgproc.fillPoly(mask, polygons, Scalar(255.0))
            }
        }
        
        
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, closeKernel)
        
        
        val finalErosionKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(8.0, 8.0))
        Imgproc.erode(mask, mask, finalErosionKernel, Point(-1.0, -1.0), 1)
        Imgproc.dilate(mask, mask, finalErosionKernel, Point(-1.0, -1.0), 1)
        
        tempHierarchy.release()
        tempContours.forEach { it.release() }
        closeKernel.release()
        dilateKernel.release()
        finalErosionKernel.release()
    }

    private fun applyPhoneDeadZoneMask(mask: Mat, imageWidth: Int, imageHeight: Int, cameraId: String? = null, zoomRatio: Float? = null) {
        val deadZones = Constants.getPhoneDeadZones(cameraId, zoomRatio)
        
        for (deadZonePoints in deadZones) {
            if (deadZonePoints.isNotEmpty()) {
                
                val pixelPoints = deadZonePoints.map { point ->
                    Point(
                        (point.x * imageWidth).coerceIn(0.0, imageWidth.toDouble()),
                        (point.y * imageHeight).coerceIn(0.0, imageHeight.toDouble())
                    )
                }
                
                
                val matOfPoint = MatOfPoint()
                matOfPoint.fromArray(*pixelPoints.toTypedArray())
                
                
                val polygons = listOf(matOfPoint)
                Imgproc.fillPoly(mask, polygons, Scalar(0.0))
                
                
                matOfPoint.release()
            }
        }
    }

    fun getDebugMaskWithContours(bitmap: Bitmap, cameraId: String? = null, zoomRatio: Float? = null): Bitmap? {
        try {
            val mat = Mat()
            Utils.bitmapToMat(bitmap, mat)
            
            
            val compensated = Mat()
            mat.convertTo(compensated, CvType.CV_8UC3, shadowCompensation, 0.0)
            
            
            val blurred = Mat()
            val kernelSize = Size(blurKernel.toDouble(), blurKernel.toDouble())
            Imgproc.GaussianBlur(compensated, blurred, kernelSize, 0.0)
            
            val masks = mutableListOf<Mat>()
            
            
            val hsv = Mat()
            Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_BGR2HSV)
            val hsvMask = Mat()
            Core.inRange(hsv, hsvLower, hsvUpper, hsvMask)
            masks.add(hsvMask)
            
            
            if (adaptiveThresh) {
                val gray = Mat()
                Imgproc.cvtColor(blurred, gray, Imgproc.COLOR_BGR2GRAY)
                val adaptiveMask = Mat()
                Imgproc.adaptiveThreshold(gray, adaptiveMask, 255.0, 
                    Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY_INV, 21, 10.0)
                masks.add(adaptiveMask)
            }
            
            
            val finalMask = masks[0].clone()
            for (i in 1 until masks.size) {
                Core.bitwise_or(finalMask, masks[i], finalMask)
            }
            
            
            val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, 
                Size(morphologyKernel.toDouble(), morphologyKernel.toDouble()))
            Imgproc.morphologyEx(finalMask, finalMask, Imgproc.MORPH_CLOSE, kernel)
            Imgproc.morphologyEx(finalMask, finalMask, Imgproc.MORPH_OPEN, kernel)
            
            
            val erosionKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.0, 3.0))
            Imgproc.erode(finalMask, finalMask, erosionKernel, Point(-1.0, -1.0), 1)
            
            
            val dilationKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                Size(dilationKernelSize.toDouble(), dilationKernelSize.toDouble()))
            Imgproc.dilate(finalMask, finalMask, dilationKernel, Point(-1.0, -1.0), dilationIterations)
            
            
            applyPhoneDeadZoneMask(finalMask, bitmap.width, bitmap.height, cameraId, zoomRatio)
            
            
            val contours = mutableListOf<MatOfPoint>()
            val hierarchy = Mat()
            Imgproc.findContours(finalMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)
            
            
            val debugMat = Mat.zeros(finalMask.size(), CvType.CV_8UC3)
            
            
            for (i in contours.indices) {
                val area = Imgproc.contourArea(contours[i])
                val touchesEdge = checkIfContourTouchesEdge(contours[i], bitmap.width, bitmap.height)
                
                val color = when {
                    touchesEdge && area < minArea * 0.3 -> Scalar(255.0, 0.0, 0.0) 
                    touchesEdge -> Scalar(0.0, 255.0, 255.0) 
                    area < minArea -> Scalar(0.0, 0.0, 255.0) 
                    area > maxArea -> Scalar(255.0, 0.0, 255.0) 
                    else -> Scalar(0.0, 255.0, 0.0) 
                }
                Imgproc.drawContours(debugMat, contours, i, color, -1)
            }
            
            
            val debugBitmap = Bitmap.createBitmap(bitmap.width, bitmap.height, Bitmap.Config.ARGB_8888)
            Utils.matToBitmap(debugMat, debugBitmap)
            
            
            mat.release()
            compensated.release()
            blurred.release()
            hsv.release()
            finalMask.release()
            debugMat.release()
            masks.forEach { it.release() }
            hierarchy.release()
            contours.forEach { it.release() }
            
            return debugBitmap
        } catch (e: Exception) {
            android.util.Log.e("TapeDetector", "Debug mask with contours failed", e)
            return null
        }
    }
}

data class TapePolygon(
    val points: Array<Point>,
    val area: Double
) {
    
    fun overlapsWithBounds(expandedBounds: ExpandedBounds): Boolean {
        
        return points.any { point ->
            point.x >= expandedBounds.x1 && point.x <= expandedBounds.x2 &&
            point.y >= expandedBounds.y1 && point.y <= expandedBounds.y2
        }
    }

    
    fun centroidInsideBounds(expandedBounds: ExpandedBounds): Boolean {
        val centroidX = points.map { it.x }.average()
        val centroidY = points.map { it.y }.average()
        
        return centroidX >= expandedBounds.x1 && centroidX <= expandedBounds.x2 &&
               centroidY >= expandedBounds.y1 && centroidY <= expandedBounds.y2
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as TapePolygon
        if (!points.contentEquals(other.points)) return false
        if (area != other.area) return false
        return true
    }

    override fun hashCode(): Int {
        var result = points.contentHashCode()
        result = 31 * result + area.hashCode()
        return result
    }
}


@Deprecated("Use TapePolygon instead", ReplaceWith("TapePolygon"))
data class TapeContour(
    val points: Array<Point>,
    val area: Double
) {
    
    fun overlapsWithBounds(expandedBounds: ExpandedBounds): Boolean {
        
        return points.any { point ->
            point.x >= expandedBounds.x1 && point.x <= expandedBounds.x2 &&
            point.y >= expandedBounds.y1 && point.y <= expandedBounds.y2
        }
    }

    
    fun centroidInsideBounds(expandedBounds: ExpandedBounds): Boolean {
        val centroidX = points.map { it.x }.average()
        val centroidY = points.map { it.y }.average()
        
        return centroidX >= expandedBounds.x1 && centroidX <= expandedBounds.x2 &&
               centroidY >= expandedBounds.y1 && centroidY <= expandedBounds.y2
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as TapeContour
        if (!points.contentEquals(other.points)) return false
        if (area != other.area) return false
        return true
    }

    override fun hashCode(): Int {
        var result = points.contentHashCode()
        result = 31 * result + area.hashCode()
        return result
    }
}
