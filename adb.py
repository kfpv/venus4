import json
import numpy as np
import cv2
import subprocess
import time
import shlex
from collections import defaultdict
import threading
from datetime import datetime
import queue
from shapely.geometry import Polygon
from camera_calibration import PHONE_CONFIGS, INCHES_TO_CM
_MATPLOTLIB_AVAILABLE = False
if False:
    try:
        import matplotlib
        backends_to_try = [  'Qt5Agg', 'TkAgg']
        for backend in backends_to_try:
            try:
                matplotlib.use(backend)
                import matplotlib.pyplot as plt
                import matplotlib.patches as patches
                from matplotlib.animation import FuncAnimation
                _MATPLOTLIB_AVAILABLE = True
                print(f"Using matplotlib backend: {backend}")
                break
            except ImportError:
                continue
        if not _MATPLOTLIB_AVAILABLE:
            print("No suitable matplotlib backend found")
    except ImportError:
        print("Matplotlib not available")
        pass
message_chunks = defaultdict(dict)
message_metadata = {}
latest_detection_data = None
latest_raw_data = None
visualization_lock = threading.Lock()
new_data_available = threading.Event()
visualization_ready = threading.Event()
visualization_stop_event = threading.Event()
last_visualization_update = 0
VISUALIZATION_UPDATE_INTERVAL = 0.1
PHONE = "pixel_uw"
specific=""
if PHONE not in PHONE_CONFIGS:
    raise ValueError(f"Unknown phone type: {PHONE}. Available options: {list(PHONE_CONFIGS.keys())}")
CURRENT_CONFIG = PHONE_CONFIGS[PHONE]
IMAGE_WIDTH_PIXELS = CURRENT_CONFIG["image_width"]
IMAGE_HEIGHT_PIXELS = CURRENT_CONFIG["image_height"]
SRC_PTS_PIXELS = CURRENT_CONFIG["calibration_points"]["src_pts_pixels"]
DST_PTS_REALWORLD = CURRENT_CONFIG["calibration_points"]["dst_pts_realworld"]
OFFSET_X_INCHES = CURRENT_CONFIG["offset"]["x_inches"]
OFFSET_Y_INCHES = CURRENT_CONFIG["offset"]["y_inches"]
print(f"Using phone configuration: {PHONE.upper()}")
print(f"Image dimensions: {IMAGE_WIDTH_PIXELS}x{IMAGE_HEIGHT_PIXELS}")
print(f"Calibration points: {len(SRC_PTS_PIXELS)} points")
print(f"Robot offset: X={OFFSET_X_INCHES:.1f}\", Y={OFFSET_Y_INCHES:.1f}\"")
HOMOGRAPHY_MATRIX, _ = cv2.findHomography(SRC_PTS_PIXELS, DST_PTS_REALWORLD)
if HOMOGRAPHY_MATRIX is None:
    print("Error: Could not compute homography matrix. Check your calibration points.")
ADB_LOGCAT_TAG = "YOLO_DETECTION_JSON"
ADB_COMMAND_BASE = f"adb {specific} logcat -s {ADB_LOGCAT_TAG}:D *:S"
RECONNECT_DELAY_SECONDS = 5
STARTUP_SKIP_DURATION = 5.0
CHUNK_TIMEOUT_SECONDS = 5
TAPE_PROCESSING_CONFIG = {
    "enable_post_processing": False,
    "mask_resolution": 0.1,
    "dilation_kernel_size": 9,
    "erosion_kernel_size": 7,
    "min_tape_area_sq_inches": 0.00,
    "approx_epsilon_factor": 0.002,
}
def create_tape_mask(polygon_points, bounds, resolution):
    if len(polygon_points) < 3:
        return None
    x_min, y_min, x_max, y_max = bounds
    width = int((x_max - x_min) / resolution) + 1
    height = int((y_max - y_min) / resolution) + 1
    if width <= 0 or height <= 0:
        return None
    mask_points = []
    for point in polygon_points:
        mask_x = int((point[0] - x_min) / resolution)
        mask_y = int((point[1] - y_min) / resolution)
        mask_points.append([mask_x, mask_y])
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [np.array(mask_points, dtype=np.int32)], 255)
    return mask
def mask_to_polygons(mask, bounds, resolution):
    if mask is None:
        return []
    x_min, y_min, x_max, y_max = bounds
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    polygons = []
    for contour in contours:
        if len(contour) < 3:
            continue
        polygon_points = []
        for point in contour:
            real_x = x_min + (point[0][0] * resolution)
            real_y = y_min + (point[0][1] * resolution)
            polygon_points.append([float(real_x), float(real_y)])
        epsilon = TAPE_PROCESSING_CONFIG["approx_epsilon_factor"] * cv2.arcLength(contour, True)
        approx_contour = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx_contour) >= 3:
            approx_polygon = []
            for point in approx_contour:
                real_x = x_min + (point[0][0] * resolution)
                real_y = y_min + (point[0][1] * resolution)
                approx_polygon.append([float(real_x), float(real_y)])
            polygons.append(approx_polygon)
    return polygons
def calculate_polygon_area(polygon_points):
    if len(polygon_points) < 3:
        return 0.0
    n = len(polygon_points)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += polygon_points[i][0] * polygon_points[j][1]
        area -= polygon_points[j][0] * polygon_points[i][1]
    return abs(area) / 2.0
def post_process_tape_polygons(tape_polygons):
    if not TAPE_PROCESSING_CONFIG["enable_post_processing"]:
        return tape_polygons
    if not tape_polygons:
        return tape_polygons
    processed_polygons = []
    resolution = TAPE_PROCESSING_CONFIG["mask_resolution"]
    all_x = []
    all_y = []
    for tape in tape_polygons:
        points = tape.get("points_realworld_xy", [])
        for point in points:
            all_x.append(point[0])
            all_y.append(point[1])
    if not all_x or not all_y:
        return tape_polygons
    padding = 2.0
    x_min = min(all_x) - padding
    x_max = max(all_x) + padding
    y_min = min(all_y) - padding
    y_max = max(all_y) + padding
    bounds = (x_min, y_min, x_max, y_max)
    width = int((x_max - x_min) / resolution) + 1
    height = int((y_max - y_min) / resolution) + 1
    combined_mask = np.zeros((height, width), dtype=np.uint8)
    for tape in tape_polygons:
        points = tape.get("points_realworld_xy", [])
        if len(points) >= 3:
            mask = create_tape_mask(points, bounds, resolution)
            if mask is not None:
                if mask.shape != combined_mask.shape:
                    mask = cv2.resize(mask, (combined_mask.shape[1], combined_mask.shape[0]))
                combined_mask = cv2.bitwise_or(combined_mask, mask)
    if np.sum(combined_mask) == 0:
        return tape_polygons
    dilation_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (TAPE_PROCESSING_CONFIG["dilation_kernel_size"], TAPE_PROCESSING_CONFIG["dilation_kernel_size"])
    )
    erosion_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (TAPE_PROCESSING_CONFIG["erosion_kernel_size"], TAPE_PROCESSING_CONFIG["erosion_kernel_size"])
    )
    processed_mask = cv2.erode(combined_mask, erosion_kernel, iterations=1)
    processed_mask = cv2.dilate(processed_mask, dilation_kernel, iterations=1)
    new_polygons = mask_to_polygons(processed_mask, bounds, resolution)
    min_area = TAPE_PROCESSING_CONFIG["min_tape_area_sq_inches"]
    for polygon in new_polygons:
        area = calculate_polygon_area(polygon)
        if area >= min_area:
            processed_polygons.append({
                "points_realworld_xy": polygon,
                "area_sq_inches": area,
                "post_processed": True
            })
    print(f"Tape post-processing: {len(tape_polygons)} -> {len(processed_polygons)} polygons")
    return processed_polygons
class DetectionProcessor:
    def __init__(self, phone_type=None, image_width=None, image_height=None,
                 homography_matrix=None, adb_tag=ADB_LOGCAT_TAG, offset_x=None, offset_y=None,
                 enable_tape_post_processing=False, adb_specific="", robot_id="robot1"):
        if phone_type is not None:
            if phone_type not in PHONE_CONFIGS:
                raise ValueError(f"Unknown phone type: {phone_type}. Available: {list(PHONE_CONFIGS.keys())}")
            config = PHONE_CONFIGS[phone_type]
            self.image_width = config["image_width"]
            self.image_height = config["image_height"]
            self.offset_x_inches = config["offset"]["x_inches"]
            self.offset_y_inches = config["offset"]["y_inches"]
            src_pts = config["calibration_points"]["src_pts_pixels"]
            dst_pts = config["calibration_points"]["dst_pts_realworld"]
            self.homography_matrix, _ = cv2.findHomography(src_pts, dst_pts)
            self.phone_type = phone_type
        else:
            self.image_width = image_width or IMAGE_WIDTH_PIXELS
            self.image_height = image_height or IMAGE_HEIGHT_PIXELS
            self.offset_x_inches = offset_x if offset_x is not None else OFFSET_X_INCHES
            self.offset_y_inches = offset_y if offset_y is not None else OFFSET_Y_INCHES
            self.homography_matrix = homography_matrix or HOMOGRAPHY_MATRIX
            self.phone_type = PHONE
        self.adb_tag = adb_tag
        self.message_chunks = defaultdict(dict)
        self.message_metadata = {}
        self.latest_detection_data = None
        self.latest_raw_data = None
        self.data_lock = threading.Lock()
        self.last_update_time = 0
        self.latest_frame_data = None
        self.frame_data_lock = threading.Lock()
        self.adb_process = None
        self.processing_thread = None
        self.stop_event = threading.Event()
        self.is_processing = False
        self.is_initialized = False
        self.initialization_complete = False
        self.initialization_event = threading.Event()
        self.adb_logcat_process = None
        self.camera_view_polygon = None
        self.processing_in_progress = threading.Event()
        self.robot_id = robot_id
        self.adb_specific = adb_specific
        self.imprinting_mode = False
        self.imprinting_start_time = None
        self.imprinting_duration = 1
        self.imprinting_result = None
        self.imprinting_complete_event = threading.Event()
        self.enable_tape_post_processing = enable_tape_post_processing
        self._camera_view_polygon = None
        self._update_camera_view_polygon()
        print(f"DetectionProcessor initialized for {self.phone_type.upper()} phone")
        print(f"Image dimensions: {self.image_width}x{self.image_height}")
        print(f"Robot offset: X={self.offset_x_inches:.1f}\", Y={self.offset_y_inches:.1f}\"")
        print(f"Tape post-processing: {'Enabled' if self.enable_tape_post_processing else 'Disabled'}")
    def _filter_polygons_dead_zone(self, polygons):
        dead_zone_polygons = DEAD_ZONES.get(self.phone_type, [])
        if not dead_zone_polygons:
            return polygons
        filtered = []
        dz_polys = [Polygon(zone) for zone in dead_zone_polygons]
        for poly in polygons:
            points = poly.get("polygon", poly.get("points", []))
            poly_points = []
            for point in points:
                if isinstance(point, dict):
                    norm_px, norm_py = point.get("x"), point.get("y")
                elif isinstance(point, (list, tuple)) and len(point) >= 2:
                    norm_px, norm_py = point[0], point[1]
                else:
                    continue
                if norm_px is None or norm_py is None:
                    continue
                pixel_px = norm_px * self.image_width
                pixel_py = norm_py * self.image_height
                poly_points.append((pixel_px, pixel_py))
            if len(poly_points) < 3:
                continue
            poly_shape = Polygon(poly_points)
            if any(poly_shape.intersects(dz) for dz in dz_polys):
                continue
            filtered.append(poly)
        return filtered
    def _update_camera_view_polygon(self):
        if self.homography_matrix is None or self.image_width is None or self.image_height is None:
            print("Cannot update camera view polygon - missing homography or dimensions")
            return
        print(f"Updating camera view polygon with image size: {self.image_width}x{self.image_height}")
        print(f"Homography matrix:\n{self.homography_matrix}")
        sample_interval = 100
        boundary_points = []
        for x in range(0, self.image_width + 1, sample_interval):
            boundary_points.append([x, 0])
        for y in range(0, self.image_height + 1, sample_interval):
            boundary_points.append([self.image_width, y])
        for x in range(self.image_width, -1, -sample_interval):
            boundary_points.append([x, self.image_height])
        for y in range(self.image_height, -1, -sample_interval):
            boundary_points.append([0, y])
        boundary_points = np.array(boundary_points, dtype=np.float32)
        print(f"Sampled {len(boundary_points)} points along image boundaries")
        valid_points = []
        for point in boundary_points:
            try:
                transformed_point = cv2.perspectiveTransform(
                    point.reshape(-1, 1, 2),
                    self.homography_matrix
                ).reshape(-1, 2)[0]
                if not np.isnan(transformed_point[0]) and not np.isnan(transformed_point[1]):
                    transformed_point[0] = (transformed_point[0] + self.offset_x_inches) * INCHES_TO_CM
                    transformed_point[1] = (transformed_point[1] + self.offset_y_inches) * INCHES_TO_CM
                    valid_points.append(transformed_point)
            except Exception as e:
                print(f"Failed to transform point {point}: {e}")
        if len(valid_points) >= 3:
            points_array = np.array(valid_points)
            hull = cv2.convexHull(points_array.astype(np.float32))
            self._camera_view_polygon = hull.reshape(-1, 2)
            print(f"Updated camera view polygon with {len(self._camera_view_polygon)} points")
            print(f"Camera view polygon points (in cm):\n{self._camera_view_polygon}")
        else:
            print("Not enough valid points for camera view polygon")
    def get_camera_view_polygon(self):
        return self._camera_view_polygon
    def set_homography_matrix(self, homography_matrix):
        self.homography_matrix = homography_matrix
        self._update_camera_view_polygon()
    def set_image_dimensions(self, width, height):
        self.image_width = width
        self.image_height = height
        self._update_camera_view_polygon()
    def wait_for_initialization(self, timeout=30):
        return self.initialization_event.wait(timeout)
    def is_ready(self):
        return self.is_initialized and self.is_processing
    def get_latest_detections(self):
        with self.data_lock:
            if self.latest_detection_data is None:
                return None
            return self.latest_detection_data.copy()
    def get_latest_raw_data(self):
        with self.data_lock:
            if self.latest_raw_data is None:
                return None
            return self.latest_raw_data.copy()
    def is_data_fresh(self, max_age_seconds=1.0):
        return (time.time() - self.last_update_time) <= max_age_seconds
    def start_processing(self):
        if self.processing_thread and self.processing_thread.is_alive():
            return
        self.stop_event.clear()
        self.is_processing = True
        self.processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.processing_thread.start()
    def stop_processing(self):
        self.stop_event.set()
        self.is_processing = False
        self.is_initialized = False
        self.initialization_event.clear()
        if self.adb_process:
            self.adb_process.kill()
            self.adb_process = None
        if self.processing_thread:
            self.processing_thread.join(timeout=5)
    def start_imprinting(self, duration_seconds=0.7):
        with self.frame_data_lock:
            self.imprinting_mode = True
            self.imprinting_start_time = time.time()
            self.imprinting_duration = duration_seconds
            self.imprinting_result = None
            self.imprinting_complete_event.clear()
            self.latest_frame_data = None
        print(f"Started imprinting mode - discarding data for {duration_seconds:.1f} seconds")
        return self.imprinting_complete_event
    def get_imprinting_result(self):
        with self.frame_data_lock:
            return self.imprinting_result.copy() if self.imprinting_result else None
    def is_imprinting(self):
        return self.imprinting_mode
    def _processing_loop(self):
        adb_start_time = None
        last_cleanup_time = time.time()
        while not self.stop_event.is_set():
            if self.adb_process is None or self.adb_process.poll() is not None:
                if self.adb_process is not None:
                    self.adb_process.kill()
                self.adb_process = self._start_adb_logcat()
                if self.adb_process is None:
                    time.sleep(RECONNECT_DELAY_SECONDS)
                    continue
                adb_start_time = time.time()
                with self.frame_data_lock:
                    self.latest_frame_data = None
                    if self.imprinting_mode:
                        self.imprinting_mode = False
                        self.imprinting_complete_event.set()
                self._purge_all_chunks()
                print(f"ADB process started, skipping backlog for {STARTUP_SKIP_DURATION} seconds...")
            try:
                current_time = time.time()
                if self.imprinting_mode:
                    elapsed_imprint_time = current_time - self.imprinting_start_time
                    if elapsed_imprint_time < self.imprinting_duration:
                        line = self.adb_process.stdout.readline()
                        if not line:
                            self.adb_process.kill()
                            self.adb_process = None
                            continue
                        with self.frame_data_lock:
                            self.latest_frame_data = None
                        self._purge_all_chunks()
                        continue
                    else:
                        line = self.adb_process.stdout.readline()
                        if not line:
                            self.adb_process.kill()
                            self.adb_process = None
                            continue
                        line = line.strip()
                        if line:
                            json_part = self._extract_json_from_logcat(line)
                            if json_part:
                                complete_message = self._process_chunked_message(json_part)
                                if complete_message:
                                    imprint_data = self._process_detection_data_for_imprinting(complete_message)
                                    with self.frame_data_lock:
                                        self.imprinting_result = imprint_data
                                        self.imprinting_mode = False
                                        self.imprinting_complete_event.set()
                                    print(f"Imprinting complete - captured fresh data after {elapsed_imprint_time:.1f}s")
                        continue
                if adb_start_time and (current_time - adb_start_time) < STARTUP_SKIP_DURATION:
                    line = self.adb_process.stdout.readline()
                    if not line:
                        self.adb_process.kill()
                        self.adb_process = None
                        continue
                    with self.frame_data_lock:
                        self.latest_frame_data = None
                    self._purge_all_chunks()
                    continue
                if adb_start_time and not self.is_initialized:
                    self.is_initialized = True
                    self.initialization_event.set()
                    print("Processor initialized and ready for fresh data!")
                    adb_start_time = None
                    with self.frame_data_lock:
                        self.latest_frame_data = None
                    self._purge_all_chunks()
                if current_time - last_cleanup_time > CHUNK_TIMEOUT_SECONDS:
                    self._cleanup_old_chunks()
                    last_cleanup_time = current_time
                line = self.adb_process.stdout.readline()
                if not line:
                    self.adb_process.kill()
                    self.adb_process = None
                    continue
                line = line.strip()
                if line:
                    json_part = self._extract_json_from_logcat(line)
                    if json_part:
                        complete_message = self._process_chunked_message(json_part)
                        if complete_message:
                            with self.frame_data_lock:
                                self.latest_frame_data = complete_message
                            if not self.processing_in_progress.is_set():
                                self._process_latest_frame()
            except Exception as e:
                if not self.stop_event.is_set():
                    print(f"Error in processing loop: {e}")
                if self.adb_process:
                    self.adb_process.kill()
                    self.adb_process = None
                with self.frame_data_lock:
                    self.latest_frame_data = None
                    if self.imprinting_mode:
                        self.imprinting_mode = False
                        self.imprinting_complete_event.set()
                self._purge_all_chunks()
                time.sleep(1)
    def _process_detection_data_for_imprinting(self, json_string):
        try:
            data = json.loads(json_string)
        except json.JSONDecodeError:
            return None
        if self.homography_matrix is None:
            return None
        output_data = {
            "timestamp": data.get("timestamp"),
            "cubes_realworld_cm": [],
            "tape_polygons_realworld_cm": []
        }
        raw_detections = {"cubes": [], "tape_polygons": []}
        cubes_data = data.get("cubes", data.get("detections", []))
        for cube in cubes_data:
            cube_result = self._process_cube(cube)
            if cube_result:
                raw_detections["cubes"].append(cube_result["raw"])
                if cube_result["realworld"]:
                    output_data["cubes_realworld_cm"].append(cube_result["realworld"])
        tape_polygons_inches = []
        tape_data = data.get("tape_masks", data.get("tape_polygons", data.get("tapes", [])))
        tape_data = self._filter_polygons_dead_zone(tape_data)
        for tape in tape_data:
            tape_result = self._process_tape(tape)
            if tape_result:
                raw_detections["tape_polygons"].append(tape_result["raw"])
                if tape_result["realworld"]:
                    points_inches = [[p[0] / INCHES_TO_CM, p[1] / INCHES_TO_CM]
                                   for p in tape_result["realworld"]["points_realworld_cm"]]
                    tape_polygons_inches.append({
                        "points_realworld_xy": points_inches,
                        "original_area_metric": tape_result["realworld"]["original_area_metric"]
                    })
        if self.enable_tape_post_processing and tape_polygons_inches:
            processed_tape_polygons = post_process_tape_polygons(tape_polygons_inches)
        else:
            processed_tape_polygons = tape_polygons_inches
        for tape in processed_tape_polygons:
            points_cm = [[p[0] * INCHES_TO_CM, p[1] * INCHES_TO_CM]
                        for p in tape["points_realworld_xy"]]
            tape_cm = {
                "points_realworld_cm": points_cm,
                "area_sq_cm": tape.get("area_sq_inches", 0) * (INCHES_TO_CM ** 2),
                "post_processed": tape.get("post_processed", False)
            }
            if "original_area_metric" in tape:
                tape_cm["original_area_metric"] = tape["original_area_metric"]
            output_data["tape_polygons_realworld_cm"].append(tape_cm)
        return {
            "detections": output_data,
            "raw": raw_detections
        }
    def _start_adb_logcat(self):
        try:
            cmd = f"adb {self.adb_specific} logcat -s {self.adb_tag}:D *:S"
            proc = subprocess.Popen(
                shlex.split(cmd),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            return proc
        except Exception as e:
            print(f"Error starting ADB logcat: {e}")
            return None
    def _extract_json_from_logcat(self, line):
        if f"D {self.adb_tag}: " in line:
            return line.split(f"D {self.adb_tag}: ", 1)[1]
        elif f"D/{self.adb_tag}: " in line:
            return line.split(f"D/{self.adb_tag}: ", 1)[1]
        elif f"{self.adb_tag}: " in line:
            return line.split(f"{self.adb_tag}: ", 1)[1]
        return None
    def _cleanup_old_chunks(self):
        current_time = time.time()
        expired_seq_ids = []
        for seq_id, metadata in self.message_metadata.items():
            if current_time - metadata['timestamp'] > CHUNK_TIMEOUT_SECONDS:
                expired_seq_ids.append(seq_id)
        for seq_id in expired_seq_ids:
            if seq_id in self.message_chunks:
                del self.message_chunks[seq_id]
            if seq_id in self.message_metadata:
                del self.message_metadata[seq_id]
    def _process_chunked_message(self, chunk_data):
        try:
            chunk_json = json.loads(chunk_data)
            if 'seq_id' in chunk_json and 'part' in chunk_json and 'total' in chunk_json:
                seq_id = chunk_json['seq_id']
                part_num = chunk_json['part']
                total_parts = chunk_json['total']
                data = chunk_json['data']
                if seq_id not in self.message_metadata:
                    self.message_metadata[seq_id] = {
                        'total_parts': total_parts,
                        'timestamp': time.time()
                    }
                self.message_chunks[seq_id][part_num] = data
                if len(self.message_chunks[seq_id]) == total_parts:
                    complete_message = ""
                    for i in range(1, total_parts + 1):
                        if i in self.message_chunks[seq_id]:
                            complete_message += self.message_chunks[seq_id][i]
                        else:
                            return None
                    del self.message_chunks[seq_id]
                    del self.message_metadata[seq_id]
                    return complete_message
                else:
                    return None
            else:
                return chunk_data
        except json.JSONDecodeError:
            return chunk_data
        except Exception:
            return None
    def _purge_all_chunks(self):
        self.message_chunks.clear()
        self.message_metadata.clear()
    def _process_latest_frame(self):
        if self.processing_in_progress.is_set():
            return
        try:
            self.processing_in_progress.set()
            while True:
                with self.frame_data_lock:
                    if self.latest_frame_data is None:
                        break
                    frame_data = self.latest_frame_data
                    self.latest_frame_data = None
                self._process_detection_data(frame_data)
                with self.frame_data_lock:
                    if self.latest_frame_data is None:
                        break
        finally:
            self.processing_in_progress.clear()
    def _process_detection_data(self, json_string):
        try:
            data = json.loads(json_string)
        except json.JSONDecodeError:
            return
        if self.homography_matrix is None:
            return
        def check_for_newer_data():
            with self.frame_data_lock:
                return self.latest_frame_data is not None
        if check_for_newer_data():
            return
        output_data = {
            "timestamp": data.get("timestamp"),
            "cubes_realworld_cm": [],
            "tape_polygons_realworld_cm": []
        }
        raw_detections = {"cubes": [], "tape_polygons": []}
        cubes_data = data.get("cubes", data.get("detections", []))
        for i, cube in enumerate(cubes_data):
            if i % 2 == 0 and check_for_newer_data():
                return
            cube_result = self._process_cube(cube)
            if cube_result:
                raw_detections["cubes"].append(cube_result["raw"])
                if cube_result["realworld"]:
                    output_data["cubes_realworld_cm"].append(cube_result["realworld"])
        if check_for_newer_data():
            return
        tape_data = data.get("tape_masks", data.get("tape_polygons", data.get("tapes", [])))
        tape_data = self._filter_polygons_dead_zone(tape_data)
        tape_polygons_inches = []
        for i, tape in enumerate(tape_data):
            if i % 1 == 0 and check_for_newer_data():
                return
            tape_result = self._process_tape(tape)
            if tape_result:
                raw_detections["tape_polygons"].append(tape_result["raw"])
                if tape_result["realworld"]:
                    points_inches = [[p[0] / INCHES_TO_CM, p[1] / INCHES_TO_CM]
                                   for p in tape_result["realworld"]["points_realworld_cm"]]
                    tape_polygons_inches.append({
                        "points_realworld_xy": points_inches,
                        "original_area_metric": tape_result["realworld"]["original_area_metric"]
                    })
        if check_for_newer_data():
            return
        if self.enable_tape_post_processing and tape_polygons_inches:
            if check_for_newer_data():
                return
            processed_tape_polygons = post_process_tape_polygons(tape_polygons_inches)
        else:
            processed_tape_polygons = tape_polygons_inches
        if check_for_newer_data():
            return
        for tape in processed_tape_polygons:
            points_cm = [[p[0] * INCHES_TO_CM, p[1] * INCHES_TO_CM]
                        for p in tape["points_realworld_xy"]]
            tape_cm = {
                "points_realworld_cm": points_cm,
                "area_sq_cm": tape.get("area_sq_inches", 0) * (INCHES_TO_CM ** 2),
                "post_processed": tape.get("post_processed", False)
            }
            if "original_area_metric" in tape:
                tape_cm["original_area_metric"] = tape["original_area_metric"]
            output_data["tape_polygons_realworld_cm"].append(tape_cm)
        if not check_for_newer_data():
            with self.data_lock:
                self.latest_detection_data = output_data
                self.latest_raw_data = raw_detections
                self.last_update_time = time.time()
    def _process_cube(self, cube):
        if "position" in cube:
            pos = cube["position"]
            norm_cx, norm_cy = pos.get("center_x"), pos.get("center_y")
            norm_w, norm_h = pos.get("width", pos.get("w")), pos.get("height", pos.get("h"))
        else:
            norm_cx, norm_cy = cube.get("center_x"), cube.get("center_y")
            if norm_cx is None or norm_cy is None:
                norm_cx, norm_cy = cube.get("cx"), cube.get("cy")
            norm_w, norm_h = cube.get("width", cube.get("w")), cube.get("height", cube.get("h"))
            if norm_w is None or norm_h is None:
                norm_w, norm_h = cube.get("bbox_width"), cube.get("bbox_height")
        if norm_cx is None or norm_cy is None:
            return None
        if norm_w is None or norm_h is None:
            norm_w, norm_h = 0.05, 0.05
        pixel_cx, pixel_cy = self._denormalize_coordinates(norm_cx, norm_cy)
        pixel_w, pixel_h = norm_w * self.image_width, norm_h * self.image_height
        pixel_x1 = max(0, min(pixel_cx - pixel_w/2, self.image_width))
        pixel_y1 = max(0, min(pixel_cy - pixel_h/2, self.image_height))
        pixel_x2 = max(0, min(pixel_cx + pixel_w/2, self.image_width))
        pixel_y2 = max(0, min(pixel_cy + pixel_h/2, self.image_height))
        bbox_corners_pixel = [
            [float(pixel_x1), float(pixel_y1)],
            [float(pixel_x2), float(pixel_y1)],
            [float(pixel_x2), float(pixel_y2)],
            [float(pixel_x1), float(pixel_y2)]
        ]
        pixel_area = (pixel_x2 - pixel_x1) * (pixel_y2 - pixel_y1)
        raw_data = {
            "pixel_x": float(pixel_cx),
            "pixel_y": float(pixel_cy),
            "pixel_x1": float(pixel_x1),
            "pixel_y1": float(pixel_y1),
            "pixel_x2": float(pixel_x2),
            "pixel_y2": float(pixel_y2),
            "pixel_width": float(pixel_w),
            "pixel_height": float(pixel_h),
            "pixel_area": float(pixel_area),
            "bbox_corners_pixel": bbox_corners_pixel,
            "norm_x": float(norm_cx),
            "norm_y": float(norm_cy),
            "norm_width": float(norm_w),
            "norm_height": float(norm_h),
            "class": cube.get("class", cube.get("clsName", "unknown")),
            "confidence": float(cube.get("confidence", cube.get("cnf", 0.0)))
        }
        real_world_center = self._pixel_to_realworld_2d(pixel_cx, pixel_cy)
        realworld_data = None
        if real_world_center is not None:
            center_cm = [real_world_center[0] * INCHES_TO_CM, real_world_center[1] * INCHES_TO_CM]
            bbox_polygon_realworld_cm = []
            for corner in bbox_corners_pixel:
                real_corner = self._pixel_to_realworld_2d(corner[0], corner[1])
                if real_corner is not None:
                    bbox_polygon_realworld_cm.append([
                        float(real_corner[0] * INCHES_TO_CM),
                        float(real_corner[1] * INCHES_TO_CM)
                    ])
                else:
                    bbox_polygon_realworld_cm = []
                    break
            area_sq_cm = None
            if len(bbox_polygon_realworld_cm) == 4:
                polygon_points = np.array(bbox_polygon_realworld_cm)
                x = polygon_points[:, 0]
                y = polygon_points[:, 1]
                area_sq_cm = float(0.5 * abs(sum(x[i]*y[(i+1)%4] - x[(i+1)%4]*y[i] for i in range(4))))
            realworld_data = {
                "class": cube.get("class", cube.get("clsName", "unknown")),
                "cube_type": cube.get("cube_type", cube.get("cubeType", "unknown")),
                "confidence": float(cube.get("confidence", cube.get("cnf", 0.0))),
                "center_realworld_cm": center_cm,
            }
            if len(bbox_polygon_realworld_cm) == 4 and area_sq_cm is not None:
                size_threshold_sq_cm = 5.5 * (INCHES_TO_CM ** 2)
                size_classification = "big" if area_sq_cm > size_threshold_sq_cm else "small"
                realworld_data["bbox_polygon_realworld_cm"] = bbox_polygon_realworld_cm
                realworld_data["area_sq_cm"] = area_sq_cm
                realworld_data["size_classification"] = size_classification
                min_x = min(corner[0] for corner in bbox_polygon_realworld_cm)
                max_x = max(corner[0] for corner in bbox_polygon_realworld_cm)
                min_y = min(corner[1] for corner in bbox_polygon_realworld_cm)
                max_y = max(corner[1] for corner in bbox_polygon_realworld_cm)
                realworld_data["bbox_realworld_cm"] = {
                    "x1": float(min_x),
                    "y1": float(min_y),
                    "x2": float(max_x),
                    "y2": float(max_y),
                    "width": float(max_x - min_x),
                    "height": float(max_y - min_y),
                    "area_sq_cm": area_sq_cm,
                    "size_classification": size_classification
                }
        return {"raw": raw_data, "realworld": realworld_data}
    def _process_tape(self, tape):
        polygon_points_realworld_cm = []
        polygon_points_pixel = []
        points = tape.get("polygon", tape.get("points", []))
        for point in points:
            if isinstance(point, dict):
                norm_px, norm_py = point.get("x"), point.get("y")
            elif isinstance(point, (list, tuple)) and len(point) >= 2:
                norm_px, norm_py = point[0], point[1]
            else:
                continue
            if norm_px is None or norm_py is None:
                continue
            pixel_px, pixel_py = self._denormalize_coordinates(norm_px, norm_py)
            polygon_points_pixel.append([float(pixel_px), float(pixel_py)])
            real_world_point = self._pixel_to_realworld_2d(pixel_px, pixel_py)
            if real_world_point is not None:
                polygon_points_realworld_cm.append([
                    float(real_world_point[0] * INCHES_TO_CM),
                    float(real_world_point[1] * INCHES_TO_CM)
                ])
        raw_data = None
        realworld_data = None
        if polygon_points_pixel:
            raw_data = {
                "points_pixel": polygon_points_pixel,
                "area": float(tape.get("area", 0.0))
            }
        if polygon_points_realworld_cm:
            realworld_data = {
                "points_realworld_cm": polygon_points_realworld_cm,
                "original_area_metric": float(tape.get("area", 0.0)),
            }
        return {"raw": raw_data, "realworld": realworld_data}
    def _denormalize_coordinates(self, norm_x, norm_y):
        pixel_x = norm_x * self.image_width
        pixel_y = norm_y * self.image_height
        return pixel_x, pixel_y
    def _pixel_to_realworld_2d(self, pixel_x, pixel_y):
        if self.homography_matrix is None:
            return None
        pixel_pt = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        transformed_pt = cv2.perspectiveTransform(pixel_pt, self.homography_matrix)
        if transformed_pt is not None and transformed_pt.shape == (1,1,2):
            adjusted_x = transformed_pt[0][0][0] + self.offset_x_inches
            adjusted_y = transformed_pt[0][0][1] + self.offset_y_inches
            return np.array([adjusted_x, adjusted_y])
        return None
def denormalize_coordinates(norm_x, norm_y, img_width, img_height):
    pixel_x = norm_x * img_width
    pixel_y = norm_y * img_height
    return pixel_x, pixel_y
def pixel_to_realworld_2d(pixel_x, pixel_y, H, offset_x_inches=None, offset_y_inches=None):
    if H is None:
        return None
    if offset_x_inches is None:
        offset_x_inches = OFFSET_X_INCHES
    if offset_y_inches is None:
        offset_y_inches = OFFSET_Y_INCHES
    pixel_pt = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
    transformed_pt = cv2.perspectiveTransform(pixel_pt, H)
    if transformed_pt is not None and transformed_pt.shape == (1,1,2):
        adjusted_x = transformed_pt[0][0][0] + offset_x_inches
        adjusted_y = transformed_pt[0][0][1] + offset_y_inches
        return np.array([adjusted_x, adjusted_y])
    return None
def cleanup_old_chunks():
    current_time = time.time()
    expired_seq_ids = []
    for seq_id, metadata in message_metadata.items():
        if current_time - metadata['timestamp'] > CHUNK_TIMEOUT_SECONDS:
            expired_seq_ids.append(seq_id)
    for seq_id in expired_seq_ids:
        if seq_id in message_chunks:
            del message_chunks[seq_id]
        if seq_id in message_metadata:
            del message_metadata[seq_id]
        print(f"Cleaned up expired message chunks for sequence {seq_id}")
def process_chunked_message(chunk_data):
    try:
        chunk_json = json.loads(chunk_data)
        if 'seq_id' in chunk_json and 'part' in chunk_json and 'total' in chunk_json:
            seq_id = chunk_json['seq_id']
            part_num = chunk_json['part']
            total_parts = chunk_json['total']
            data = chunk_json['data']
            if seq_id not in message_metadata:
                message_metadata[seq_id] = {
                    'total_parts': total_parts,
                    'timestamp': time.time()
                }
            message_chunks[seq_id][part_num] = data
            if len(message_chunks[seq_id]) == total_parts:
                complete_message = ""
                for i in range(1, total_parts + 1):
                    if i in message_chunks[seq_id]:
                        complete_message += message_chunks[seq_id][i]
                    else:
                        print(f"Missing part {i} for sequence {seq_id}")
                        return None
                del message_chunks[seq_id]
                del message_metadata[seq_id]
                print(f"Reassembled message from {total_parts} chunks (seq_id: {seq_id})")
                return complete_message
            else:
                print(f"Received chunk {part_num}/{total_parts} for sequence {seq_id}")
                return None
        else:
            return chunk_data
    except json.JSONDecodeError:
        return chunk_data
    except Exception as e:
        print(f"Error processing chunked message: {e}")
        return None
def setup_visualization():
    if not _MATPLOTLIB_AVAILABLE:
        print("Matplotlib not available. Visualization disabled.")
        return None, None, None
    try:
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        if fig is None or ax1 is None or ax2 is None:
            print("Error: Failed to create matplotlib figure")
            return None, None, None
        try:
            fig.canvas.manager.set_window_title('YOLO Detection Visualization')
        except AttributeError:
            pass
        ax1.set_title('Raw Detections (Pixel Coordinates)')
        ax1.set_xlim(0, IMAGE_WIDTH_PIXELS)
        ax1.set_ylim(IMAGE_HEIGHT_PIXELS, 0)
        ax1.set_xlabel('X (pixels)')
        ax1.set_ylabel('Y (pixels)')
        ax1.grid(True, alpha=0.3)
        ax2.set_title('Transformed Detections (Real World - Inches)')
        ax2.set_xlim(-20, 20)
        ax2.set_ylim(-5, 25)
        ax2.set_xlabel('X (inches)')
        ax2.set_ylabel('Y (inches)')
        ax2.grid(True, alpha=0.3)
        try:
            for i, (src_pt, dst_pt) in enumerate(zip(SRC_PTS_PIXELS, DST_PTS_REALWORLD)):
                ax1.plot(src_pt[0], src_pt[1], 'ro', markersize=8, alpha=0.7)
                ax1.annotate(f'Cal{i}', (src_pt[0], src_pt[1]), xytext=(5, 5), textcoords='offset points', fontsize=8)
                ax2.plot(dst_pt[0], dst_pt[1], 'ro', markersize=8, alpha=0.7)
                ax2.annotate(f'Cal{i}', (dst_pt[0], dst_pt[1]), xytext=(5, 5), textcoords='offset points', fontsize=8)
        except Exception as e:
            print(f"Warning: Could not plot calibration points: {e}")
        plt.tight_layout()
        plt.show(block=False)
        plt.draw()
        return fig, ax1, ax2
    except Exception as e:
        print(f"Error setting up visualization: {e}")
        return None, None, None
def clear_detection_markers(ax):
    artists_to_remove = []
    for artist in ax.collections + ax.patches:
        if hasattr(artist, '_detection_marker'):
            artists_to_remove.append(artist)
    for text in ax.texts:
        if hasattr(text, '_detection_marker'):
            artists_to_remove.append(text)
    for artist in artists_to_remove:
        artist.remove()
def update_visualization_display(fig, ax1, ax2):
    global last_visualization_update
    try:
        current_time = time.time()
        if current_time - last_visualization_update < VISUALIZATION_UPDATE_INTERVAL:
            return
        with visualization_lock:
            if latest_raw_data is None or latest_detection_data is None:
                return
            raw_data = latest_raw_data.copy()
            real_world_data = latest_detection_data.copy()
            new_data_available.clear()
        if not plt.fignum_exists(fig.number):
            print("Figure window was closed")
            return
        clear_detection_markers(ax1)
        clear_detection_markers(ax2)
        for cube in raw_data["cubes"]:
            try:
                if "bbox_corners_pixel" in cube and len(cube["bbox_corners_pixel"]) == 4:
                    polygon_points = np.array(cube["bbox_corners_pixel"])
                    polygon1 = patches.Polygon(polygon_points, closed=True, fill=False,
                                             edgecolor='blue', linewidth=2)
                    polygon1._detection_marker = True
                    ax1.add_patch(polygon1)
                else:
                    bbox_width = cube["pixel_x2"] - cube["pixel_x1"]
                    bbox_height = cube["pixel_y2"] - cube["pixel_y1"]
                    rect1 = patches.Rectangle(
                        (cube["pixel_x1"], cube["pixel_y1"]),
                        bbox_width, bbox_height,
                        linewidth=2, edgecolor='blue', facecolor='none'
                    )
                    rect1._detection_marker = True
                    ax1.add_patch(rect1)
                area_text = f"Area: {cube['pixel_area']:.0f}px²"
                text1 = ax1.annotate(f'{cube["class"]}\n{cube["confidence"]:.2f}\n{area_text}',
                            (cube["pixel_x1"], cube["pixel_y1"]),
                            xytext=(5, 5), textcoords='offset points',
                            bbox=dict(boxstyle='round,pad=0.3', facecolor='blue', alpha=0.7),
                            color='white', fontsize=8)
                text1._detection_marker = True
            except Exception as e:
                print(f"Error plotting cube: {e}")
        for tape in raw_data["tape_polygons"]:
            try:
                if len(tape["points_pixel"]) >= 3:
                    polygon_points = np.array(tape["points_pixel"])
                    polygon1 = patches.Polygon(polygon_points, closed=True, fill=False,
                                             edgecolor='green', linewidth=2)
                    polygon1._detection_marker = True
                    ax1.add_patch(polygon1)
            except Exception as e:
                print(f"Error plotting tape polygon: {e}")
        for cube in real_world_data["cubes_realworld_2d"]:
            try:
                x, y = cube["center_realworld_xy"]
                if "bbox_polygon_realworld" in cube and len(cube["bbox_polygon_realworld"]) == 4:
                    polygon_points = np.array(cube["bbox_polygon_realworld"])
                    polygon2 = patches.Polygon(polygon_points, closed=True, fill=False,
                                             edgecolor='blue', linewidth=2)
                    polygon2._detection_marker = True
                    ax2.add_patch(polygon2)
                    area_sq_inches = cube.get("area_sq_inches", 0.0)
                    size_label = "Big" if area_sq_inches > 5.5 else "Small"
                    area_text = f"Area: {area_sq_inches:.2f}in²\n{size_label} Cube"
                    text_x, text_y = polygon_points[0]
                    text2 = ax2.annotate(f'{cube["class"]}\n{cube["confidence"]:.2f}\n{area_text}',
                                (text_x, text_y), xytext=(5, 5), textcoords='offset points',
                                bbox=dict(boxstyle='round,pad=0.3', facecolor='blue', alpha=0.7),
                                color='white', fontsize=8)
                else:
                    circle2 = plt.Circle((x, y), 0.5, color='blue', fill=False, linewidth=2)
                    circle2._detection_marker = True
                    ax2.add_patch(circle2)
                    text2 = ax2.annotate(f'{cube["class"]}\n{cube["confidence"]:.2f}',
                                (x, y), xytext=(10, 10), textcoords='offset points',
                                bbox=dict(boxstyle='round,pad=0.3', facecolor='blue', alpha=0.7),
                                color='white', fontsize=8)
                text2._detection_marker = True
            except Exception as e:
                print(f"Error plotting real world cube: {e}")
        for tape in real_world_data["tape_polygons_realworld_2d"]:
            try:
                if len(tape["points_realworld_xy"]) >= 3:
                    polygon_points = np.array(tape["points_realworld_xy"])
                    polygon2 = patches.Polygon(polygon_points, closed=True, fill=False,
                                             edgecolor='green', linewidth=2)
                    polygon2._detection_marker = True
                    ax2.add_patch(polygon2)
            except Exception as e:
                print(f"Error plotting real world tape: {e}")
        current_time_str = datetime.now().strftime("%H:%M:%S")
        ax1.set_title(f'Raw Detections (Pixel Coordinates) - {current_time_str}')
        ax2.set_title(f'Transformed Detections (Real World - Inches) - {current_time_str}')
        fig.canvas.draw()
        fig.canvas.flush_events()
        last_visualization_update = current_time
    except Exception as e:
        print(f"Error updating visualization: {e}")
def start_visualization():
    global visualization_ready
    try:
        fig, ax1, ax2 = setup_visualization()
        if fig is None:
            print("Failed to setup visualization")
            return
        visualization_ready.set()
        print("Visualization window opened. Waiting for detection data...")
        while not visualization_stop_event.is_set():
            try:
                if not plt.fignum_exists(fig.number):
                    print("Visualization window was closed by user")
                    break
                if new_data_available.is_set():
                    update_visualization_display(fig, ax1, ax2)
                time.sleep(0.05)
                try:
                    plt.pause(0.001)
                except Exception as e:
                    if "invalid" in str(e).lower() or "destroyed" in str(e).lower():
                        break
            except Exception as e:
                print(f"Error in visualization loop: {e}")
                time.sleep(1)
    except KeyboardInterrupt:
        print("Visualization stopped by user")
    except Exception as e:
        print(f"Error in visualization: {e}")
    finally:
        try:
            plt.ioff()
            plt.close('all')
        except:
            pass
def start_adb_logcat():
    print(f"Attempting to start ADB logcat with command: {ADB_COMMAND_BASE}")
    try:
        proc = subprocess.Popen(
            shlex.split(ADB_COMMAND_BASE),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        print(f"ADB logcat process started (PID: {proc.pid}).")
        return proc
    except FileNotFoundError:
        print("Error: 'adb' command not found. Make sure ADB is installed and in your PATH.")
        return None
    except Exception as e:
        print(f"Error starting ADB logcat: {e}")
        return None
def process_adb_json_data_2d(json_string):
    global latest_detection_data, latest_raw_data
    try:
        data = json.loads(json_string)
    except json.JSONDecodeError as e:
        print(f"JSON decode error: {e} for line: {json_string[:100]}")
        return None
    if HOMOGRAPHY_MATRIX is None:
        print("Homography matrix not available. Cannot process.")
        return None
    output_data = {"timestamp": data.get("timestamp"), "cubes_realworld_2d": [], "tape_polygons_realworld_2d": []}
    raw_detections = {"cubes": [], "tape_polygons": []}
    cubes_data = data.get("cubes", data.get("detections", []))
    for cube in cubes_data:
        if "position" in cube:
            pos = cube["position"]
            norm_cx, norm_cy = pos.get("center_x"), pos.get("center_y")
            norm_w, norm_h = pos.get("width", pos.get("w")), pos.get("height", pos.get("h"))
        else:
            norm_cx, norm_cy = cube.get("center_x"), cube.get("center_y")
            if norm_cx is None or norm_cy is None:
                norm_cx, norm_cy = cube.get("cx"), cube.get("cy")
            norm_w, norm_h = cube.get("width", cube.get("w")), cube.get("height", cube.get("h"))
            if norm_w is None or norm_h is None:
                norm_w, norm_h = cube.get("bbox_width"), cube.get("bbox_height")
        if norm_cx is None or norm_cy is None:
            print(f"Skipping cube with missing center coordinates: {cube}")
            continue
        if norm_w is None or norm_h is None:
            norm_w, norm_h = 0.05, 0.05
            print(f"Using default dimensions for cube: {cube.get('class', 'unknown')}")
        print(f"DEBUG: Cube {cube.get('class', 'unknown')} - norm center: ({norm_cx:.3f}, {norm_cy:.3f}), norm size: ({norm_w:.3f}, {norm_h:.3f})")
        pixel_cx, pixel_cy = denormalize_coordinates(norm_cx, norm_cy, IMAGE_WIDTH_PIXELS, IMAGE_HEIGHT_PIXELS)
        pixel_w, pixel_h = norm_w * IMAGE_WIDTH_PIXELS, norm_h * IMAGE_HEIGHT_PIXELS
        pixel_x1 = pixel_cx - (pixel_w / 2)
        pixel_y1 = pixel_cy - (pixel_h / 2)
        pixel_x2 = pixel_cx + (pixel_w / 2)
        pixel_y2 = pixel_cy + (pixel_h / 2)
        pixel_x1 = max(0, min(pixel_x1, IMAGE_WIDTH_PIXELS))
        pixel_y1 = max(0, min(pixel_y1, IMAGE_HEIGHT_PIXELS))
        pixel_x2 = max(0, min(pixel_x2, IMAGE_WIDTH_PIXELS))
        pixel_y2 = max(0, min(pixel_y2, IMAGE_HEIGHT_PIXELS))
        bbox_corners_pixel = [
            [float(pixel_x1), float(pixel_y1)],
            [float(pixel_x2), float(pixel_y1)],
            [float(pixel_x2), float(pixel_y2)],
            [float(pixel_x1), float(pixel_y2)]
        ]
        print(f"DEBUG: Cube pixel coords - center: ({pixel_cx:.1f}, {pixel_cy:.1f}), bbox: ({pixel_x1:.1f}, {pixel_y1:.1f}) to ({pixel_x2:.1f}, {pixel_y2:.1f})")
        pixel_area = (pixel_x2 - pixel_x1) * (pixel_y2 - pixel_y1)
        real_world_center = pixel_to_realworld_2d(pixel_cx, pixel_cy, HOMOGRAPHY_MATRIX, OFFSET_X_INCHES, OFFSET_Y_INCHES)
        bbox_polygon_realworld = []
        for corner in bbox_corners_pixel:
            real_world_corner = pixel_to_realworld_2d(corner[0], corner[1], HOMOGRAPHY_MATRIX, OFFSET_X_INCHES, OFFSET_Y_INCHES)
            if real_world_corner is not None:
                bbox_polygon_realworld.append([float(real_world_corner[0]), float(real_world_corner[1])])
            else:
                bbox_polygon_realworld = []
                break
        real_world_area = None
        if len(bbox_polygon_realworld) == 4:
            polygon_points = np.array(bbox_polygon_realworld)
            x = polygon_points[:, 0]
            y = polygon_points[:, 1]
            real_world_area = float(0.5 * abs(sum(x[i]*y[(i+1)%4] - x[(i+1)%4]*y[i] for i in range(4))))
            print(f"DEBUG: Real world polygon area: {real_world_area:.3f} sq inches")
        if real_world_center is not None:
            print(f"DEBUG: Real world center: ({real_world_center[0]:.3f}, {real_world_center[1]:.3f})")
        if len(bbox_polygon_realworld) == 4:
            print(f"DEBUG: Real world polygon corners: {bbox_polygon_realworld}")
        raw_detections["cubes"].append({
            "pixel_x": float(pixel_cx),
            "pixel_y": float(pixel_cy),
            "pixel_x1": float(pixel_x1),
            "pixel_y1": float(pixel_y1),
            "pixel_x2": float(pixel_x2),
            "pixel_y2": float(pixel_y2),
            "pixel_width": float(pixel_w),
            "pixel_height": float(pixel_h),
            "pixel_area": float(pixel_area),
            "bbox_corners_pixel": bbox_corners_pixel,
            "norm_x": float(norm_cx),
            "norm_y": float(norm_cy),
            "norm_width": float(norm_w),
            "norm_height": float(norm_h),
            "class": cube.get("class", cube.get("clsName", "unknown")),
            "confidence": float(cube.get("confidence", cube.get("cnf", 0.0)))
        })
        if real_world_center is not None:
            cube_data = {
                "class": cube.get("class", cube.get("clsName", "unknown")),
                "cube_type": cube.get("cube_type", cube.get("cubeType", "unknown")),
                "confidence": float(cube.get("confidence", cube.get("cnf", 0.0))),
                "center_realworld_xy": [float(real_world_center[0]), float(real_world_center[1])],
            }
            if len(bbox_polygon_realworld) == 4 and real_world_area is not None:
                size_classification = "big" if real_world_area > 5.5 else "small"
                cube_data["bbox_polygon_realworld"] = bbox_polygon_realworld
                cube_data["area_sq_inches"] = real_world_area
                cube_data["size_classification"] = size_classification
                min_x = min(corner[0] for corner in bbox_polygon_realworld)
                max_x = max(corner[0] for corner in bbox_polygon_realworld)
                min_y = min(corner[1] for corner in bbox_polygon_realworld)
                max_y = max(corner[1] for corner in bbox_polygon_realworld)
                cube_data["bbox_realworld"] = {
                    "x1": float(min_x),
                    "y1": float(min_y),
                    "x2": float(max_x),
                    "y2": float(max_y),
                    "width": float(max_x - min_x),
                    "height": float(max_y - min_y),
                    "area_sq_inches": real_world_area,
                    "size_classification": size_classification
                }
            output_data["cubes_realworld_2d"].append(cube_data)
    tape_polygons_inches = []
    tape_data = data.get("tape_masks", data.get("tape_polygons", data.get("tapes", [])))
    tape_data = self._filter_polygons_dead_zone(tape_data)
    for tape in tape_data:
        polygon_points_realworld = []
        polygon_points_pixel = []
        points = tape.get("polygon", tape.get("points", []))
        for point in points:
            if isinstance(point, dict):
                norm_px, norm_py = point.get("x"), point.get("y")
            elif isinstance(point, (list, tuple)) and len(point) >= 2:
                norm_px, norm_py = point[0], point[1]
            else:
                print(f"Skipping invalid point format: {point}")
                continue
            if norm_px is None or norm_py is None:
                print(f"Skipping point with missing coordinates: {point}")
                continue
            pixel_px, pixel_py = denormalize_coordinates(norm_px, norm_py, IMAGE_WIDTH_PIXELS, IMAGE_HEIGHT_PIXELS)
            polygon_points_pixel.append([float(pixel_px), float(pixel_py)])
            real_world_point = pixel_to_realworld_2d(pixel_px, pixel_py, HOMOGRAPHY_MATRIX, OFFSET_X_INCHES, OFFSET_Y_INCHES)
            if real_world_point is not None:
                polygon_points_realworld.append([float(real_world_point[0]), float(real_world_point[1])])
        if polygon_points_pixel:
            raw_detections["tape_polygons"].append({
                "points_pixel": polygon_points_pixel,
                "area": float(tape.get("area", 0.0))
            })
        if polygon_points_realworld:
            tape_polygons_inches.append({
                "points_realworld_xy": polygon_points_realworld,
                "original_area_metric": float(tape.get("area", 0.0)),
            })
    if TAPE_PROCESSING_CONFIG["enable_post_processing"] and tape_polygons_inches:
        processed_tape_polygons = post_process_tape_polygons(tape_polygons_inches)
    else:
        processed_tape_polygons = tape_polygons_inches
    for tape in processed_tape_polygons:
        output_data["tape_polygons_realworld_2d"].append(tape)
    with visualization_lock:
        latest_detection_data = output_data
        latest_raw_data = raw_detections
        new_data_available.set()
    return output_data
def main_loop():
    if not _MATPLOTLIB_AVAILABLE:
        print("Warning: Matplotlib not available. Running in library mode.")
        print("--- Real-time ADB Log Parser for 2D Coordinates (Library Mode) ---")
        print(f"Using ADB Logcat Tag: {ADB_LOGCAT_TAG}")
        print(f"Image dimensions for denormalization: {IMAGE_WIDTH_PIXELS}x{IMAGE_HEIGHT_PIXELS}")
        print("Make sure your Android device is connected and USB debugging is enabled.")
        print("Coordinates will be displayed in centimeters.")
        print("----------------------------------------------------")
        processor = DetectionProcessor()
        processor.start_processing()
        print("Waiting for processor initialization...")
        if not processor.wait_for_initialization(timeout=30):
            print("Warning: Processor did not initialize within 30 seconds")
            return
        last_data_time = 0
        last_cube_count = 0
        last_tape_count = 0
        try:
            while True:
                time.sleep(0.5)
                detections = processor.get_latest_detections()
                if detections and processor.is_data_fresh(max_age_seconds=2.0):
                    cubes = detections.get('cubes_realworld_cm', [])
                    tapes = detections.get('tape_polygons_realworld_cm', [])
                    current_time = time.time()
                    cube_count = len(cubes)
                    tape_count = len(tapes)
                    if (cube_count != last_cube_count or tape_count != last_tape_count or
                        current_time - last_data_time > 10):
                        timestamp_str = datetime.now().strftime("%H:%M:%S")
                        print(f"\n[{timestamp_str}] Detection Update:")
                        print(f"  Cubes: {cube_count}, Tape polygons: {tape_count}")
                        for i, cube in enumerate(cubes):
                            center = cube['center_realworld_cm']
                            area = cube.get('area_sq_cm', 0)
                            size = cube.get('size_classification', 'unknown')
                            conf = cube.get('confidence', 0)
                            print(f"    Cube {i+1}: {center[0]:.1f}, {center[1]:.1f} cm, "
                                  f"{area:.1f} cm², {size}, conf: {conf:.2f}")
                        for i, tape in enumerate(tapes):
                            points = tape.get('points_realworld_cm', [])
                            print(f"    Tape {i+1}: {len(points)} points")
                        last_data_time = current_time
                        last_cube_count = cube_count
                        last_tape_count = tape_count
                    else:
                        print(f"Latest detections: {cube_count} cubes, {tape_count} tape polygons")
                elif processor.is_ready():
                    print("No fresh detection data available")
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            processor.stop_processing()
        return
    print("--- Real-time ADB Log Parser for 2D Coordinates (with Message Chunking and Visualization) ---")
    print(f"Using ADB Logcat Tag: {ADB_LOGCAT_TAG}")
    print(f"Image dimensions for denormalization: {IMAGE_WIDTH_PIXELS}x{IMAGE_HEIGHT_PIXELS}")
    print("Make sure your Android device is connected and USB debugging is enabled.")
    print("Ensure the app is running and logging JSON data with the specified tag.")
    print("----------------------------------------------------")
    fig, ax1, ax2 = setup_visualization()
    if fig is None:
        print("Failed to setup visualization, running without GUI")
        return
    adb_thread = threading.Thread(target=adb_processing_loop, daemon=True)
    adb_thread.start()
    print("Visualization window opened. ADB processing started in background...")
    try:
        while True:
            try:
                if not plt.fignum_exists(fig.number):
                    print("Visualization window was closed by user")
                    break
                if new_data_available.is_set():
                    update_visualization_display(fig, ax1, ax2)
                time.sleep(0.05)
                plt.pause(0.001)
            except Exception as e:
                print(f"Error in visualization loop: {e}")
                time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        visualization_stop_event.set()
        try:
            plt.ioff()
            plt.close('all')
        except:
            pass
def adb_processing_loop():
    adb_process = None
    last_cleanup_time = time.time()
    adb_start_time = None
    messages_skipped = 0
    try:
        while not visualization_stop_event.is_set():
            if adb_process is None or adb_process.poll() is not None:
                if adb_process is not None:
                    print("ADB logcat process terminated. Attempting to reconnect...")
                    adb_process.kill()
                adb_process = start_adb_logcat()
                if adb_process is None:
                    print(f"Failed to start ADB. Retrying in {RECONNECT_DELAY_SECONDS} seconds...")
                    time.sleep(RECONNECT_DELAY_SECONDS)
                    continue
                adb_start_time = time.time()
                messages_skipped = 0
                cleanup_old_chunks()
                with visualization_lock:
                    global latest_detection_data, latest_raw_data
                    latest_detection_data = None
                    latest_raw_data = None
                print(f"Skipping messages for first {STARTUP_SKIP_DURATION} seconds to avoid backlog...")
            try:
                if adb_start_time and (time.time() - adb_start_time) < STARTUP_SKIP_DURATION:
                    line = adb_process.stdout.readline()
                    if not line:
                        print("ADB logcat stream ended during startup skip.")
                        adb_process.kill()
                        adb_process = None
                        continue
                    line = line.strip()
                    if line and (f"D {ADB_LOGCAT_TAG}: " in line or f"D/{ADB_LOGCAT_TAG}: " in line or f"{ADB_LOGCAT_TAG}: " in line):
                        messages_skipped += 1
                        if messages_skipped % 20 == 0:
                            current_time = time.time()
                            remaining_time = STARTUP_SKIP_DURATION - (current_time - adb_start_time)
                            print(f"Discarding backlog messages... {remaining_time:.1f}s remaining (discarded {messages_skipped})")
                    cleanup_old_chunks()
                    continue
                current_time = time.time()
                if adb_start_time and (current_time - adb_start_time) >= STARTUP_SKIP_DURATION:
                    print(f"Finished discarding backlog. Discarded {messages_skipped} messages. Now processing fresh messages...")
                    adb_start_time = None
                    messages_skipped = 0
                    cleanup_old_chunks()
                if current_time - last_cleanup_time > CHUNK_TIMEOUT_SECONDS:
                    cleanup_old_chunks()
                    last_cleanup_time = current_time
                line = adb_process.stdout.readline()
                if not line:
                    print("ADB logcat stream ended.")
                    adb_process.kill()
                    adb_process = None
                    continue
                line = line.strip()
                if line:
                    json_part = None
                    if f"D {ADB_LOGCAT_TAG}: " in line:
                        json_part = line.split(f"D {ADB_LOGCAT_TAG}: ", 1)[1]
                    elif f"D/{ADB_LOGCAT_TAG}: " in line:
                        json_part = line.split(f"D/{ADB_LOGCAT_TAG}: ", 1)[1]
                    elif f"{ADB_LOGCAT_TAG}: " in line:
                        json_part = line.split(f"{ADB_LOGCAT_TAG}: ", 1)[1]
                    else:
                        continue
                    complete_message = process_chunked_message(json_part)
                    if complete_message:
                        with visualization_lock:
                            latest_detection_data = None
                            latest_raw_data = None
                        real_world_data = process_adb_json_data_2d(complete_message)
                        if real_world_data and (real_world_data["cubes_realworld_2d"] or real_world_data["tape_polygons_realworld_2d"]):
                            print("\n--- Converted Real-World 2D Data ---")
                            print(json.dumps(real_world_data, indent=2))
            except Exception as e:
                if not visualization_stop_event.is_set():
                    print(f"Error reading from ADB logcat: {e}")
                if adb_process:
                    adb_process.kill()
                    adb_process = None
                cleanup_old_chunks()
                with visualization_lock:
                    latest_detection_data = None
                    latest_raw_data = None
                time.sleep(1)
    finally:
        if adb_process is not None:
            adb_process.kill()
DEAD_ZONES = {
    "oneplus_uw": [
        [
            [1309.0, 3058.0],
            [1419.0, 2557.0],
            [2750.0, 2541.0],
            [2887.0, 3036.0],
        ],
    ],
    "pixel_uw": [
        [
            [1127.0, 3124.0],
            [1380.0, 2326.0],
            [2717.0, 2387.0],
            [2904.0, 3102.0],
        ],
    ],
    "oneplus_uw_zoom": [
        [
            [1200.0, 3010.0],
            [1310.0, 2660.0],
            [2952.0, 2660.0],
            [3077.0, 3010.0],
        ],
    ],
}
if __name__ == "__main__":
    main_loop()