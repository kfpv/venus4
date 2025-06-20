package com.surendramaran.yolov8tflite

import org.opencv.core.Point

object Constants {
    const val AIO_MODEL_PATH = "aiov7_32.tflite"
    const val AIO_LABELS_PATH = "aio_labels_v6.txt"
    const val USE_AIO_MODEL = true
    const val CUBE_EXPANSION_FACTOR = 0.42f
    const val SHOW_INFERENCE_TIME = false
    const val POLYGON_APPROXIMATION_ACCURACY = 0.002
    const val DETECTION_LOG_TAG = "YOLO_DETECTION_JSON"
    const val PHONE_MODEL = "oneplus_uw"
    private val BASE_DEAD_ZONE_POLYGON = arrayOf(
        Point(1309.0 / 4000.0, 3058.0 / 3000.0),
        Point(1419.0 / 4000.0, 2557.0 / 3000.0),
        Point(2750.0 / 4000.0, 2541.0 / 3000.0),
        Point(2887.0 / 4000.0, 3036.0 / 3000.0)
    )
    private val CAMERA_0_LOW_ZOOM_BLOCKING_POLYGON = arrayOf(
        Point(1127.0 / 4000.0, 3124.0 / 3000.0),
        Point(1380.0 / 4000.0, 2326.0 / 3000.0),
        Point(2717.0 / 4000.0, 2387.0 / 3000.0),
        Point(2904.0 / 4000.0, 3102.0 / 3000.0)
    )
    private val CAMERA_2_HIGH_ZOOM_BLOCKING_POLYGON = arrayOf(
        Point(1200.0 / 4000.0, 3010.0 / 3000.0),
        Point(1310.0 / 4000.0, 2660.0 / 3000.0),
        Point(2952.0 / 4000.0, 2660.0 / 3000.0),
        Point(3077.0 / 4000.0, 3010.0 / 3000.0)
    )
    private val DEAD_ZONES = mapOf(
        "oneplus_uw" to listOf(BASE_DEAD_ZONE_POLYGON)
    )
    fun getPhoneDeadZones(): List<Array<Point>> {
        return DEAD_ZONES[PHONE_MODEL] ?: emptyList()
    }
    fun getPhoneDeadZones(cameraId: String?, zoomRatio: Float?): List<Array<Point>> {
        val baseDeadZones = DEAD_ZONES[PHONE_MODEL] ?: emptyList()
        if (baseDeadZones.isEmpty()) return emptyList()
        return when (cameraId) {
            "2" -> {
                val currentZoom = zoomRatio ?: 1.0f
                when {
                    currentZoom > 1.1f -> {
                        listOf(CAMERA_2_HIGH_ZOOM_BLOCKING_POLYGON)
                    }
                    else -> {
                        baseDeadZones
                    }
                }
            }
            "0" -> {
                val currentZoom = zoomRatio ?: 1.0f
                when {
                    currentZoom >= 1.0f -> {
                        emptyList()
                    }
                    currentZoom < 0.6f -> {
                        listOf(CAMERA_0_LOW_ZOOM_BLOCKING_POLYGON)
                    }
                    else -> {
                        emptyList()
                    }
                }
            }
            else -> {
                baseDeadZones
            }
        }
    }
    fun getCubeTypeFromAioIndex(index: Int): CubeType {
        return when (index) {
            0 -> CubeType.BLACK
            1 -> CubeType.BLUE
            2 -> CubeType.GREEN
            3 -> CubeType.RED
            4 -> CubeType.WHITE
            5 -> CubeType.BOX
            6 -> CubeType.ROBOT
            else -> CubeType.WHITE
        }
    }
}
