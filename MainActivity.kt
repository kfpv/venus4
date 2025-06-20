package com.surendramaran.yolov8tflite

import android.Manifest
import android.content.ContentValues
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.Matrix
import android.os.Bundle
import android.os.Message
import android.provider.MediaStore
import android.util.Log
import android.view.View
import android.view.WindowInsets
import android.view.WindowInsetsController
import android.view.WindowManager
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.core.AspectRatio
import androidx.camera.core.Camera
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageCapture
import androidx.camera.core.ImageCaptureException
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.surendramaran.yolov8tflite.Constants.CUBE_EXPANSION_FACTOR
import org.json.JSONArray
import org.json.JSONObject
import org.opencv.android.OpenCVLoader
import yolov8tflite.R
import yolov8tflite.databinding.ActivityMainBinding
import java.text.SimpleDateFormat
import java.util.Locale
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.camera2.interop.Camera2Interop
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraMetadata
import androidx.camera.core.CameraInfo

@ExperimentalCamera2Interop
class MainActivity : AppCompatActivity(), Detector.DetectorListener {
    private lateinit var binding: ActivityMainBinding
    private val isFrontCamera = false
    private var preview: Preview? = null
    private var imageAnalyzer: ImageAnalysis? = null
    private var imageCapture: ImageCapture? = null
    private var camera: Camera? = null
    private var cameraProvider: ProcessCameraProvider? = null
    private var detector: Detector? = null
    private var tapeDetector: TapeDetector? = null
    private var isFlashlightOn = false
    private var isOpenCVInitialized = false
    private lateinit var cameraExecutor: ExecutorService
    private var isUltrawideMode = false
    private var minZoomRatio = 0.5f
    private var normalZoomRatio = 1.0f
    private var isDebugContoursMode = false
    private var areOverlaysVisible = true
    private var isAtMinZoom = true
    private var isAtOnePointOneFiveZoom = false
    private var isSendingInProgress = false
    private var messageSequenceId = 0L
    private data class PhysicalCameraInfo(
        val cameraInfo: CameraInfo,
        val cameraId: String,
        val lensFacing: Int,
        val minZoomRatio: Float,
        val maxZoomRatio: Float,
        val isUltraWide: Boolean = false
    )
    private var availablePhysicalCameras = mutableListOf<PhysicalCameraInfo>()
    private var currentPhysicalCameraIndex = 0
    private var currentPhysicalCamera: PhysicalCameraInfo? = null
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        makeAppImmersive()
        toast("Phone Model: ${Constants.PHONE_MODEL}")
        binding.overlay.setOverlaysEnabled(areOverlaysVisible)
        if (OpenCVLoader.initLocal()) {
            isOpenCVInitialized = true
            tapeDetector = TapeDetector()
        } else {
            toast("OpenCV initialization failed!")
        }
        cameraExecutor = Executors.newSingleThreadExecutor()
        cameraExecutor.execute {
            detector = Detector(baseContext, this) {
                toast(it)
            }
        }
        if (allPermissionsGranted()) {
            startCamera()
        } else {
            ActivityCompat.requestPermissions(this, REQUIRED_PERMISSIONS, REQUEST_CODE_PERMISSIONS)
        }
        bindListeners()
        enumeratePhysicalCameras()
    }
    override fun onResume() {
        super.onResume()
        if (allPermissionsGranted()){
            startCamera()
        } else {
            requestPermissionLauncher.launch(REQUIRED_PERMISSIONS)
        }
    }
    private fun bindListeners() {
        binding.apply {
            flashlightButton.setOnClickListener {
                toggleFlashlight()
            }
            captureButton.setOnClickListener {
                captureImage()
            }
            ultrawideButton.setOnClickListener {
                toggleUltrawide()
            }
            ultrawideButton.setOnLongClickListener {
                toggleZoom()
                true
            }
            debugContoursButton.setOnClickListener {
                toggleDebugContours()
            }
            toggleOverlaysButton.setOnClickListener {
                toggleAllOverlays()
            }
        }
    }
    private fun makeAppImmersive() {
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.R) {
            window.insetsController?.let { controller ->
                controller.hide(WindowInsets.Type.statusBars() or WindowInsets.Type.navigationBars())
                controller.systemBarsBehavior = WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
            }
        } else {
            @Suppress("DEPRECATION")
            window.decorView.systemUiVisibility = (
                View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                or View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                or View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                or View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                or View.SYSTEM_UI_FLAG_FULLSCREEN
            )
        }
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
    }
    private fun toggleFlashlight() {
        camera?.let { camera ->
            if (camera.cameraInfo.hasFlashUnit()) {
                isFlashlightOn = !isFlashlightOn
                camera.cameraControl.enableTorch(isFlashlightOn)
                binding.flashlightButton.apply {
                    if (isFlashlightOn) {
                        text = "🔦"
                    } else {
                        text = "💡"
                    }
                }
            } else {
                toast("Flash not available")
            }
        }
    }
    private fun captureImage() {
        val imageCapture = imageCapture ?: return
        val name = SimpleDateFormat(FILENAME_FORMAT, Locale.US)
            .format(System.currentTimeMillis())
        val contentValues = ContentValues().apply {
            put(MediaStore.MediaColumns.DISPLAY_NAME, name)
            put(MediaStore.MediaColumns.MIME_TYPE, "image/jpeg")
            put(MediaStore.MediaColumns.RELATIVE_PATH, "Pictures/YOLOv8-Detector")
        }
        val outputOptions = ImageCapture.OutputFileOptions
            .Builder(contentResolver, MediaStore.Images.Media.EXTERNAL_CONTENT_URI, contentValues)
            .build()
        imageCapture.takePicture(
            outputOptions,
            ContextCompat.getMainExecutor(this),
            object : ImageCapture.OnImageSavedCallback {
                override fun onError(exception: ImageCaptureException) {
                    toast("Photo capture failed")
                }
                override fun onImageSaved(output: ImageCapture.OutputFileResults) {
                    toast("Photo captured successfully")
                }
            }
        )
    }
    private fun startCamera() {
        val cameraProviderFuture = ProcessCameraProvider.getInstance(this)
        cameraProvider  = cameraProviderFuture.get()
        enumeratePhysicalCameras()
        bindCameraUseCases()
    }
    private fun toggleDebugContours() {
        isDebugContoursMode = !isDebugContoursMode
        binding.debugContoursButton.apply {
            if (isDebugContoursMode) {
                text = "🔍"
            } else {
                text = "🐛"
            }
        }
        binding.overlay.setDebugMode(isDebugContoursMode)
        binding.overlay.setContourDebugMode(isDebugContoursMode)
        if (!isDebugContoursMode) {
            binding.overlay.setDebugMask(null)
        }
    }
    private fun toggleAllOverlays() {
        areOverlaysVisible = !areOverlaysVisible
        binding.toggleOverlaysButton.text = if (areOverlaysVisible) "🚫" else "👁️"
        binding.overlay.setOverlaysEnabled(areOverlaysVisible)
        if (!areOverlaysVisible) {
            binding.overlay.clear()
            binding.inferenceTime.visibility = View.GONE
        }
    }
    private fun logDetectionJson(boundingBoxes: List<BoundingBox>, tapePolygons: List<TapePolygon>) {
        if (isSendingInProgress) {
            return
        }
        try {
            val jsonObject = JSONObject()
            val timestamp = System.currentTimeMillis()
            jsonObject.put("timestamp", timestamp)
            jsonObject.put("frame_id", timestamp)
            val cubesArray = JSONArray()
            boundingBoxes.forEach { box ->
                val cubeObject = JSONObject()
                cubeObject.put("class", box.clsName)
                cubeObject.put("cube_type", box.cubeType.name)
                cubeObject.put("confidence", box.cnf)
                val positionObject = JSONObject()
                positionObject.put("x1", box.x1)
                positionObject.put("y1", box.y1)
                positionObject.put("x2", box.x2)
                positionObject.put("y2", box.y2)
                positionObject.put("center_x", box.cx)
                positionObject.put("center_y", box.cy)
                positionObject.put("width", box.w)
                positionObject.put("height", box.h)
                cubeObject.put("position", positionObject)
                cubesArray.put(cubeObject)
            }
            jsonObject.put("cubes", cubesArray)
            val tapeArray = JSONArray()
            tapePolygons.forEach { polygon ->
                val tapeObject = JSONObject()
                tapeObject.put("area", polygon.area)
                val pointsArray = JSONArray()
                polygon.points.forEach { point ->
                    val pointObject = JSONObject()
                    pointObject.put("x", point.x)
                    pointObject.put("y", point.y)
                    pointsArray.put(pointObject)
                }
                tapeObject.put("polygon", pointsArray)
                tapeArray.put(tapeObject)
            }
            jsonObject.put("tape_masks", tapeArray)
            val fullMessage = jsonObject.toString()
            sendDetectionMessage(fullMessage)
        } catch (e: Exception) {
        }
    }
    private fun sendDetectionMessage(message: String) {
        val maxChunkSize = 900
        if (message.length <= maxChunkSize) {
            Log.d(Constants.DETECTION_LOG_TAG, message)
        } else {
            isSendingInProgress = true
            val sequenceId = ++messageSequenceId
            val chunks = splitMessage(message, maxChunkSize)
            Thread {
                try {
                    for (i in chunks.indices) {
                        val chunkData = JSONObject().apply {
                            put("seq_id", sequenceId)
                            put("part", i + 1)
                            put("total", chunks.size)
                            put("data", chunks[i])
                        }
                        Log.d(Constants.DETECTION_LOG_TAG, chunkData.toString())
                        Thread.sleep(10)
                    }
                } catch (e: Exception) {
                } finally {
                    isSendingInProgress = false
                }
            }.start()
        }
    }
    private fun splitMessage(message: String, maxChunkSize: Int): List<String> {
        val chunks = mutableListOf<String>()
        var startIndex = 0
        while (startIndex < message.length) {
            val endIndex = minOf(startIndex + maxChunkSize, message.length)
            chunks.add(message.substring(startIndex, endIndex))
            startIndex = endIndex
        }
        return chunks
    }
    private fun processFrame(rotatedBitmap: Bitmap) {
        if (!areOverlaysVisible) {
            return
        }
        if (isDebugContoursMode) {
            if (isOpenCVInitialized && tapeDetector != null) {
                try {
                    val currentCameraId = currentPhysicalCamera?.cameraId
                    val currentZoomRatio = camera?.cameraInfo?.zoomState?.value?.zoomRatio
                    val debugMask = tapeDetector!!.getDebugMaskWithContours(rotatedBitmap, currentCameraId, currentZoomRatio)
                    runOnUiThread {
                        binding.overlay.setDebugMask(debugMask)
                    }
                } catch (e: Exception) {
                }
            }
            return
        }
        detector?.detect(rotatedBitmap)
        if (isOpenCVInitialized && tapeDetector != null) {
            try {
                val expandedBounds = binding.overlay.getCurrentResults().map {
                    it.getExpandedBounds(Constants.CUBE_EXPANSION_FACTOR)
                }
                val currentCameraId = currentPhysicalCamera?.cameraId
                val currentZoomRatio = camera?.cameraInfo?.zoomState?.value?.zoomRatio
                val tapePolygons = tapeDetector!!.detectTapeObjects(rotatedBitmap, expandedBounds, currentCameraId, currentZoomRatio)
                runOnUiThread {
                    if (areOverlaysVisible) {
                        binding.overlay.setTapePolygons(tapePolygons)
                    }
                    logDetectionJson(binding.overlay.getCurrentResults(), tapePolygons)
                }
            } catch (e: Exception) {
            }
        }
    }
    private fun enumeratePhysicalCameras() {
        if (cameraProvider == null) {
            return
        }
        try {
            val availableCameraInfos = cameraProvider!!.availableCameraInfos
            availablePhysicalCameras.clear()
            for (cameraInfo in availableCameraInfos) {
                try {
                    val camera2Info = Camera2CameraInfo.from(cameraInfo)
                    val cameraId = camera2Info.getCameraId()
                    val cameraManager = getSystemService(Context.CAMERA_SERVICE) as CameraManager
                    val characteristics = cameraManager.getCameraCharacteristics(cameraId)
                    val lensFacing = characteristics.get(CameraCharacteristics.LENS_FACING)
                    if (lensFacing == CameraCharacteristics.LENS_FACING_BACK) {
                        val zoomRange = characteristics.get(CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE)
                        val minZoom = zoomRange?.lower ?: 1.0f
                        val maxZoom = zoomRange?.upper ?: 1.0f
                        val isUltraWide = minZoom < 1.0f
                        val focalLengths = characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
                        val focalLength = focalLengths?.minOrNull() ?: 0f
                        val physicalCameraInfo = PhysicalCameraInfo(
                            cameraInfo = cameraInfo,
                            cameraId = cameraId,
                            lensFacing = lensFacing,
                            minZoomRatio = minZoom,
                            maxZoomRatio = maxZoom,
                            isUltraWide = isUltraWide
                        )
                        availablePhysicalCameras.add(physicalCameraInfo)
                    }
                } catch (e: Exception) {
                }
            }
            availablePhysicalCameras.sortWith(compareBy<PhysicalCameraInfo> { !it.isUltraWide }.thenBy { it.minZoomRatio })
            if (availablePhysicalCameras.isNotEmpty()) {
                val camera2Index = availablePhysicalCameras.indexOfFirst { it.cameraId == "2" }
                if (camera2Index != -1) {
                    currentPhysicalCameraIndex = camera2Index
                    currentPhysicalCamera = availablePhysicalCameras[camera2Index]
                    isAtMinZoom = false
                    isAtOnePointOneFiveZoom = true
                } else {
                    currentPhysicalCamera = availablePhysicalCameras[0]
                    currentPhysicalCameraIndex = 0
                    isAtMinZoom = true
                    isAtOnePointOneFiveZoom = false
                }
                updateUltrawideButton()
            } else {
                toast("No back-facing cameras found")
            }
        } catch (e: Exception) {
            toast("Failed to enumerate cameras, using default")
        }
    }
    private fun toggleUltrawide() {
        if (availablePhysicalCameras.isEmpty()) {
            toast("No physical cameras available")
            return
        }
        currentPhysicalCameraIndex = (currentPhysicalCameraIndex + 1) % availablePhysicalCameras.size
        currentPhysicalCamera = availablePhysicalCameras[currentPhysicalCameraIndex]
        isAtMinZoom = true
        updateUltrawideButton()
        cameraProvider?.unbindAll()
        bindCameraUseCases()
        toast("Switching to camera ${currentPhysicalCamera?.cameraId}")
    }
    private fun toggleZoom() {
        currentPhysicalCamera?.let { physicalCamera ->
            camera?.let { cam ->
                val targetZoom = when (physicalCamera.cameraId) {
                    "2" -> {
                        if (isAtMinZoom) {
                            1.15f
                        } else if (isAtOnePointOneFiveZoom) {
                            physicalCamera.minZoomRatio
                        } else {
                            1.15f
                        }
                    }
                    else -> {
                        if (isAtMinZoom) {
                            1.0f
                        } else {
                            physicalCamera.minZoomRatio
                        }
                    }
                }
                cam.cameraControl.setZoomRatio(targetZoom)
                if (physicalCamera.cameraId == "2") {
                    isAtOnePointOneFiveZoom = targetZoom == 1.15f
                    isAtMinZoom = targetZoom == physicalCamera.minZoomRatio
                } else {
                    isAtMinZoom = targetZoom == physicalCamera.minZoomRatio
                    isAtOnePointOneFiveZoom = false
                }
                binding.overlay.setCameraState(physicalCamera.cameraId, targetZoom)
                val zoomText = when {
                    physicalCamera.cameraId == "2" && isAtOnePointOneFiveZoom -> "1.15x"
                    isAtMinZoom -> "${String.format("%.1f", physicalCamera.minZoomRatio)}x"
                    else -> "1.0x"
                }
                toast("Zoom set to $zoomText")
                updateUltrawideButton()
            }
        } ?: run {
            toast("No camera available for zoom adjustment")
        }
    }
    private fun updateUltrawideButton() {
        currentPhysicalCamera?.let { camera ->
            val currentZoom = when {
                camera.cameraId == "2" && isAtOnePointOneFiveZoom -> 1.15f
                isAtMinZoom -> camera.minZoomRatio
                else -> 1.0f
            }
            val buttonText = "${String.format("%.1f", currentZoom)}x (${camera.cameraId})"
            binding.ultrawideButton.text = buttonText
        } ?: run {
            binding.ultrawideButton.text = "No Cam"
        }
    }
    private fun bindCameraUseCases() {
        val cameraProvider = cameraProvider ?: throw IllegalStateException("Camera initialization failed.")
        val rotation = binding.viewFinder.display.rotation
        val cameraSelector = currentPhysicalCamera?.cameraInfo?.cameraSelector ?: CameraSelector.DEFAULT_BACK_CAMERA
        preview =  Preview.Builder()
            .setTargetAspectRatio(AspectRatio.RATIO_4_3)
            .setTargetRotation(rotation)
            .build()
        imageAnalyzer = ImageAnalysis.Builder()
            .setTargetAspectRatio(AspectRatio.RATIO_4_3)
            .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
            .setTargetRotation(binding.viewFinder.display.rotation)
            .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
            .build()
        imageCapture = ImageCapture.Builder()
            .setTargetAspectRatio(AspectRatio.RATIO_4_3)
            .setTargetRotation(rotation)
            .build()
        imageAnalyzer?.setAnalyzer(cameraExecutor) { imageProxy ->
            val bitmapBuffer =
                Bitmap.createBitmap(
                    imageProxy.width,
                    imageProxy.height,
                    Bitmap.Config.ARGB_8888
                )
            imageProxy.use { bitmapBuffer.copyPixelsFromBuffer(imageProxy.planes[0].buffer) }
            imageProxy.close()
            val matrix = Matrix().apply {
                postRotate(imageProxy.imageInfo.rotationDegrees.toFloat())
                if (isFrontCamera) {
                    postScale(
                        -1f,
                        1f,
                        imageProxy.width.toFloat(),
                        imageProxy.height.toFloat()
                    )
                }
            }
            val rotatedBitmap = Bitmap.createBitmap(
                bitmapBuffer, 0, 0, bitmapBuffer.width, bitmapBuffer.height,
                matrix, true
            )
            processFrame(rotatedBitmap)
        }
        cameraProvider.unbindAll()
        try {
            camera = cameraProvider.bindToLifecycle(
                this,
                cameraSelector,
                preview,
                imageAnalyzer,
                imageCapture
            )
            preview?.surfaceProvider = binding.viewFinder.surfaceProvider
            currentPhysicalCamera?.let { physicalCamera ->
                val initialZoom = when {
                    physicalCamera.cameraId == "2" && isAtOnePointOneFiveZoom -> 1.15f
                    else -> physicalCamera.minZoomRatio
                }
                camera?.cameraControl?.setZoomRatio(initialZoom)
                binding.overlay.setCameraState(physicalCamera.cameraId, initialZoom)
                try {
                    val camera2Info = Camera2CameraInfo.from(camera!!.cameraInfo)
                    val boundCameraId = camera2Info.getCameraId()
                } catch (e: Exception) {
                }
            }
        } catch(exc: Exception) {
            val defaultSelector = CameraSelector.DEFAULT_BACK_CAMERA
            try {
                camera = cameraProvider.bindToLifecycle(
                    this,
                    defaultSelector,
                    preview,
                    imageAnalyzer,
                    imageCapture
                )
                preview?.surfaceProvider = binding.viewFinder.surfaceProvider
                toast("Fallback to default camera")
            } catch (fallbackExc: Exception) {
                toast("Camera binding failed")
            }
        }
    }
    private fun allPermissionsGranted() = REQUIRED_PERMISSIONS.all {
        ContextCompat.checkSelfPermission(baseContext, it) == PackageManager.PERMISSION_GRANTED
    }
    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()) {
        if (it[Manifest.permission.CAMERA] == true) { startCamera() }
    }
    private fun toast(message: String) {
        runOnUiThread {
            Toast.makeText(baseContext, message, Toast.LENGTH_LONG).show()
        }
    }
    override fun onDestroy() {
        super.onDestroy()
        detector?.close()
        cameraExecutor.shutdown()
    }
    companion object {
        private const val TAG = "Camera"
        private const val REQUEST_CODE_PERMISSIONS = 10
        private const val FILENAME_FORMAT = "yyyyMMddHHmmss"
        private val REQUIRED_PERMISSIONS = mutableListOf (
            Manifest.permission.CAMERA
        ).toTypedArray()
    }
    override fun onEmptyDetect() {
        runOnUiThread {
            if (!areOverlaysVisible) {
                return@runOnUiThread
            }
            binding.overlay.clear()
        }
    }
    override fun onDetect(boundingBoxes: List<BoundingBox>, inferenceTime: Long) {
        runOnUiThread {
            if (!areOverlaysVisible) {
                return@runOnUiThread
            }
            val filteredBoundingBoxes = boundingBoxes.filter { boundingBox ->
                val imageWidth = 4000
                val imageHeight = 3000
                val targetX = 2000
                val targetY = 2800
                !boundingBox.containsCoordinate(targetX, targetY, imageWidth, imageHeight)
            }
            if (Constants.SHOW_INFERENCE_TIME) {
                binding.inferenceTime.text = "${inferenceTime}ms"
                binding.inferenceTime.visibility = View.VISIBLE
            } else {
                binding.inferenceTime.visibility = View.GONE
            }
            binding.overlay.apply {
                setResults(filteredBoundingBoxes)
                invalidate()
            }
        }
    }
}
