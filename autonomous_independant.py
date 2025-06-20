import time
import threading
from main_integration import (
    publish_imprinted_data,
    transform_point_to_world,
    simplify_and_round_polygon,
    encode_polygon_polyline
)
from object_tracker import ObjectTracker
from camera_view_filter import CameraViewFilter
import numpy as np
import cv2
from shapely.geometry import Polygon, Point, LineString
import queue
def classify_tape_polygon(polygon_points):
    if len(polygon_points) < 3:
        return 'tape'
    try:
        points = np.array(polygon_points, dtype=np.float32)
        rect = cv2.minAreaRect(points)
        box = cv2.boxPoints(rect)
        box = np.array(box, dtype=np.int32)
        width = rect[1][0]
        height = rect[1][1]
        aspect_ratio = max(width, height) / min(width, height)
        polygon_area = 0
        n = len(polygon_points)
        for i in range(n):
            j = (i + 1) % n
            polygon_area += polygon_points[i][0] * polygon_points[j][1]
            polygon_area -= polygon_points[j][0] * polygon_points[i][1]
        polygon_area = abs(polygon_area) / 2
        rect_area = width * height
        if rect_area == 0:
            return 'tape'
        coverage_ratio = polygon_area / rect_area
        if aspect_ratio < 4.5 and coverage_ratio > 0.45:
            return 'crater'
        return 'tape'
    except Exception as e:
        print(f"Error in crater classification: {e}")
        return 'tape'
def run_autonomous_mode(robot_ctl, detection_proc, mqtt_client, robot_config):
    print("[AUTO] Autonomous mode started!")
    auto = AutonomousIndependant(robot_ctl, detection_proc, mqtt_client, robot_config)
    auto.run()
def run_simple_demo_mode(robot_ctl, detection_proc, mqtt_client, robot_config):
    print("[AUTO] Simple demo mode started!")
    auto = AutonomousIndependant(robot_ctl, detection_proc, mqtt_client, robot_config)
    auto.simple_demo()
class AutonomousIndependant:
    def __init__(self, robot_ctl, detection_proc, mqtt_client, robot_config):
        self.robot_ctl = robot_ctl
        self.detection_proc = detection_proc
        self.mqtt_client = mqtt_client
        self.robot_config = robot_config
        self.running = True
        self.object_tracker = ObjectTracker()
        self.camera_filter = CameraViewFilter()
        self.last_avoidance_direction = None
        self.avoidance_direction_preference = None
        self.avoidance_movement_count = 0
        self.avoidance_direction_mode = 1
        if self.detection_proc and self.detection_proc.is_ready():
            bounds = self.detection_proc.get_camera_view_polygon()
            if bounds is not None and len(bounds) > 0:
                self.camera_filter.set_camera_bounds(bounds)
        self._publish_queue = queue.Queue()
        self._publish_worker_thread = threading.Thread(target=self._publish_worker, daemon=True)
        self._publish_worker_thread.start()
    def update_camera_bounds(self):
        if self.detection_proc and self.detection_proc.is_ready():
            camera_view = self.detection_proc.get_camera_view_polygon()
            if camera_view is not None and len(camera_view) > 0:
                world_points = [
                    transform_point_to_world(self.robot_ctl.x, self.robot_ctl.y, self.robot_ctl.heading, x, y)
                    for x, y in camera_view
                ]
                self.camera_filter.set_camera_bounds(world_points)
    def detect_and_update(self):
        return self.detect_and_send()
        self.update_camera_bounds()
        imprint_result = self.imprint_and_detect()
        detections = None
        if imprint_result:
            detections = imprint_result.get('detections', {})
            self.update_tracked_objects(detections)
            return detections
    def detect_and_send(self):
        self.update_camera_bounds()
        imprint_result = self.imprint_and_detect()
        detections = None
        if imprint_result:
            detections = imprint_result.get('detections', {})
            self.update_tracked_objects(detections)
            self.send_detections(detections)
            return detections
    def simple_demo(self):
        self.detect_and_send()
        self.turn(-40)
        self.detect_and_send()
        self.turn(-40)
        self.detect_and_send()
        self.turn(120)
        self.detect_and_send()
        self.turn(40)
        self.detect_and_send()
        self.turn(-80)
    def run(self):
        print("[AUTO] Running autonomous logic loop: go straight until tape with avoidance...")
        self.go_straight_until_tape_with_avoidance(max_distance=1e6, max_steps=1000000)
        print("[AUTO] Autonomous mode finished.")
    def move_forward(self, distance_cm):
        print(f"[AUTO] Moving forward {distance_cm}cm (stub)")
        if self.robot_ctl:
            self.robot_ctl.move_distance(distance_cm)
            while getattr(self.robot_ctl, 'moving', False):
                time.sleep(0.1)
    def turn(self, angle_deg):
        print(f"[AUTO] Turning {angle_deg} degrees (stub)")
        if self.robot_ctl:
            self.robot_ctl.turn_angle(angle_deg)
            while getattr(self.robot_ctl, 'moving', False):
                time.sleep(0.1)
    def imprint_and_detect(self):
        print("[AUTO] Starting imprint (object detection)...")
        print("[DEBUG] Detection processor ready:", self.detection_proc.is_ready() if self.detection_proc else None)
        if not self.detection_proc or not self.detection_proc.is_ready():
            print("[AUTO] Detection processor not ready!")
            return None
        imprint_event = self.detection_proc.start_imprinting(duration_seconds=0.7)
        print("[AUTO] Imprinting in progress...")
        if imprint_event.wait(timeout=5.0):
            imprint_result = self.detection_proc.get_imprinting_result()
            print("[AUTO] Imprinting complete!")
            return imprint_result
        else:
            print("[AUTO] Imprinting timed out!")
            return None
    def _publish_worker(self):
        while True:
            item = self._publish_queue.get()
            if item is None:
                break
            detections, done_event = item
            try:
                publish_imprinted_data(detections, self.robot_ctl.x, self.robot_ctl.y, self.robot_ctl.heading, self.mqtt_client, self.robot_config, send_latest_pose=True)
                print("[AUTO] Detections sent using publish_imprinted_data (queue worker)")
            except Exception as e:
                print(f"[AUTO] Failed to send detections (queue worker): {e}")
            if done_event:
                done_event.set()
    def send_detections(self, detections, wait=False):
        print("[AUTO] Queueing detections to base station using publish_imprinted_data..." if not wait else "[AUTO] Sending detections to base station using publish_imprinted_data (synchronous)...")
        if not detections or not self.robot_ctl:
            print("[AUTO] No detections or robot controller unavailable.")
            return
        if wait:
            try:
                publish_imprinted_data(detections, self.robot_ctl.x, self.robot_ctl.y, self.robot_ctl.heading, self.mqtt_client, self.robot_config, send_latest_pose=True)
                print("[AUTO] Detections sent using publish_imprinted_data (synchronous)")
            except Exception as e:
                print(f"[AUTO] Failed to send detections (synchronous): {e}")
        else:
            self._publish_queue.put((detections, None))
    def update_tracked_objects(self, detections):
        if not detections:
            print("[DEBUG] No detections to update tracked objects.")
            return
        cubes = detections.get('cubes_realworld_cm', [])
        tapes = detections.get('tape_polygons_realworld_cm', [])
        craters = detections.get('crater_polygons_realworld_cm', []) if 'crater_polygons_realworld_cm' in detections else []
        boxes = detections.get('box_polygons_realworld_cm', []) if 'box_polygons_realworld_cm' in detections else []
        print("[DEBUG] Raw cubes:", cubes)
        print("[DEBUG] Raw tapes:", tapes)
        print("[DEBUG] Raw craters:", craters)
        print("[DEBUG] Raw boxes:", boxes)
        filtered_cubes = []
        for cube in cubes:
            if 'center_realworld_cm' in cube:
                cam_x, cam_y = cube['center_realworld_cm']
                world_x, world_y = transform_point_to_world(
                    self.robot_ctl.x, self.robot_ctl.y, self.robot_ctl.heading, cam_x, cam_y)
                cube['world_x'] = world_x
                cube['world_y'] = world_y
            if 'bbox_polygon_realworld_cm' in cube:
                world_polygon = []
                for point_cam_x, point_cam_y in cube['bbox_polygon_realworld_cm']:
                    wx, wy = transform_point_to_world(
                        self.robot_ctl.x, self.robot_ctl.y, self.robot_ctl.heading, point_cam_x, point_cam_y)
                    world_polygon.append([wx, wy])
                cube['world_polygon'] = world_polygon
            filtered = self.camera_filter.filter_cube(cube)
            if filtered:
                filtered_cubes.append(filtered)
        filtered_tapes = []
        filtered_craters = []
        for tape in tapes:
            if 'points_realworld_cm' in tape:
                world_points = []
                for point_cam_x, point_cam_y in tape['points_realworld_cm']:
                    wx, wy = transform_point_to_world(
                        self.robot_ctl.x, self.robot_ctl.y, self.robot_ctl.heading, point_cam_x, point_cam_y)
                    world_points.append([wx, wy])
                tape['world_polygon'] = world_points
            if 'world_polygon' in tape and len(tape['world_polygon']) >= 3:
                classification = classify_tape_polygon(tape['world_polygon'])
                if classification == 'crater':
                    print("[DEBUG] Crater detected")
                    filtered = self.camera_filter.filter_crater(tape)
                    if filtered:
                        filtered_craters.append(filtered)
                else:
                    print("[DEBUG] Tape detected")
                    filtered = self.camera_filter.filter_tape(tape)
                    if filtered:
                        filtered_tapes.append(filtered)
        filtered_boxes = []
        for box in boxes:
            filtered = self.camera_filter.filter_box(box)
            if filtered:
                filtered_boxes.append(filtered)
        filtered_craters.extend([c for c in craters if self.camera_filter.filter_crater(c)])
        self.object_tracker.add_cubes(filtered_cubes)
        self.object_tracker.add_tapes(filtered_tapes)
        self.object_tracker.add_craters(filtered_craters)
        self.object_tracker.add_boxes(filtered_boxes)
    def get_tracked_objects(self):
        return self.object_tracker.get_all_tracked_objects()
    def check_objects_in_box(self, source='tracked', right_width=10.0, left_width=10.0, front_depth=15.0, back_depth=16.0, detections=None):
        robot_x = self.robot_ctl.x
        robot_y = self.robot_ctl.y
        robot_heading = self.robot_ctl.heading
        heading_rad = np.radians(robot_heading)
        cos_h = np.cos(heading_rad)
        sin_h = np.sin(heading_rad)
        box_polygon = [
            (front_depth, -right_width),
            (front_depth, left_width),
            (-back_depth, left_width),
            (-back_depth, -right_width)
        ]
        box_poly = Polygon(box_polygon)
        front_objects = []
        back_objects = []
        def to_robot_frame(x, y):
            dx = x - robot_x
            dy = y - robot_y
            rel_x = dx * cos_h + dy * sin_h
            rel_y = -dx * sin_h + dy * cos_h
            return (rel_x, rel_y)
        objects = []
        if source == 'raw' and detections is not None:
            cubes = detections.get('cubes_realworld_cm', []) if detections else []
            tapes = detections.get('tape_polygons_realworld_cm', []) if detections else []
            craters = detections.get('crater_polygons_realworld_cm', []) if 'crater_polygons_realworld_cm' in detections else []
            for cube in cubes:
                if 'center_realworld_cm' in cube:
                    cam_x, cam_y = cube['center_realworld_cm']
                    world_x, world_y = transform_point_to_world(robot_x, robot_y, robot_heading, cam_x, cam_y)
                    rel = to_robot_frame(world_x, world_y)
                    if box_poly.contains(Point(rel)):
                        dist = np.hypot(rel[0], rel[1])
                        front_objects.append({'object': {'type': 'cube', 'center': (world_x, world_y)}, 'distance': dist})
            for tape in tapes:
                if 'points_realworld_cm' in tape and tape['points_realworld_cm']:
                    world_points = [
                        transform_point_to_world(robot_x, robot_y, robot_heading, cam_x, cam_y)
                        for cam_x, cam_y in tape['points_realworld_cm']
                    ]
                    rel_points = [to_robot_frame(wx, wy) for wx, wy in world_points]
                    tape_poly = Polygon(rel_points)
                    if tape_poly.intersects(box_poly):
                        front_objects.append({'object': {'type': 'tape', 'polygon': rel_points}})
            for crater in craters:
                if 'points_realworld_cm' in crater and crater['points_realworld_cm']:
                    world_points = [
                        transform_point_to_world(robot_x, robot_y, robot_heading, cam_x, cam_y)
                        for cam_x, cam_y in crater['points_realworld_cm']
                    ]
                    rel_points = [to_robot_frame(wx, wy) for wx, wy in world_points]
                    crater_poly = Polygon(rel_points)
                    if crater_poly.intersects(box_poly):
                        front_objects.append({'object': {'type': 'crater', 'polygon': rel_points}})
        if source == 'raw' and detections is None:
            print("[AUTO] No detections provided")
            return {'front': [], 'back': []}
        else:
            tracked = self.get_tracked_objects()
            objects = []
            for cube in tracked.get('cubes', []):
                if hasattr(cube, 'center'):
                    objects.append({'type': 'cube', 'center': cube.center})
            for tape in tracked.get('tapes', []):
                if hasattr(tape, 'polygon') and tape.polygon:
                    for p in tape.polygon:
                        objects.append({'type': 'tape', 'center': p})
        print("[DEBUG] Objects considered for box check:", objects)
        for obj in objects:
            rel = to_robot_frame(obj['center'][0], obj['center'][1])
            if box_poly.contains(Point(rel)):
                dist = np.hypot(rel[0], rel[1])
                front_objects.append({'object': obj, 'distance': dist})
        return {'front': front_objects, 'back': back_objects}
    def check_objects_in_avoidance_boxes(self, source='tracked', detections=None):
        if detections is None:
            detections = {}
        else:
            detections = detections.get('detections', {})
        print(f"[DEBUG][BOTH] DETECTIONS: {detections}")
        robot_x = self.robot_ctl.x
        robot_y = self.robot_ctl.y
        robot_heading = self.robot_ctl.heading
        heading_rad = np.radians(robot_heading)
        cos_h = np.cos(heading_rad)
        sin_h = np.sin(heading_rad)
        front_box = [
            (15.0, -9.5),
            (15.0, 9.5),
            (0.0, 9.5),
            (0.0, -9.5)
        ]
        right_box = [
            (20.0, 9.5),
            (20.0, 20.0),
            (0.0, 20.0),
            (0.0, 9.5)
        ]
        left_box = [
            (20.0, -20.0),
            (20.0, -9.5),
            (0.0, -9.5),
            (0.0, -20.0)
        ]
        front_poly = Polygon(front_box)
        right_poly = Polygon(right_box)
        left_poly = Polygon(left_box)
        front_objects = []
        right_objects = []
        left_objects = []
        def to_robot_frame(x, y):
            dx = x - robot_x
            dy = y - robot_y
            rel_x = dx * cos_h + dy * sin_h
            rel_y = -dx * sin_h + dy * cos_h
            return (rel_x, rel_y)
        if source == 'both':
            if detections is None:
                print('[AUTO] No detections provided for both mode')
                return {'front': [], 'right': [], 'left': []}
            cubes = detections.get('cubes_realworld_cm', []) if detections else []
            print(f'[DEBUG][BOTH] Processing {len(cubes)} raw cubes for avoidance')
            print(f'[DEBUG][BOTH] Avoidance box polygons:')
            print(f'  FRONT: {front_box}')
            print(f'  RIGHT: {right_box}')
            print(f'  LEFT: {left_box}')
            cubes_in_front = cubes_in_right = cubes_in_left = 0
            for cube in cubes:
                if 'bbox_polygon_realworld_cm' in cube and cube['bbox_polygon_realworld_cm']:
                    world_points = [transform_point_to_world(robot_x, robot_y, robot_heading, x, y) for x, y in cube['bbox_polygon_realworld_cm']]
                    rel_points = [to_robot_frame(wx, wy) for wx, wy in world_points]
                    for pt in rel_points:
                        p = Point(pt)
                        if front_poly.contains(p):
                            front_objects.append({'type': 'cube', 'polygon': rel_points})
                            cubes_in_front += 1
                            print('[DEBUG][BOTH] RAW cube point in FRONT box')
                            break
                    for pt in rel_points:
                        p = Point(pt)
                        if right_poly.contains(p):
                            right_objects.append({'type': 'cube', 'polygon': rel_points})
                            cubes_in_right += 1
                            print('[DEBUG][BOTH] RAW cube point in RIGHT box')
                            break
                    for pt in rel_points:
                        p = Point(pt)
                        if left_poly.contains(p):
                            left_objects.append({'type': 'cube', 'polygon': rel_points})
                            cubes_in_left += 1
                            print('[DEBUG][BOTH] RAW cube point in LEFT box')
                            break
            print(f'[DEBUG][BOTH] Cubes detected in boxes: FRONT={cubes_in_front}, RIGHT={cubes_in_right}, LEFT={cubes_in_left}')
            robots = detections.get('robot_polygons_realworld_cm', []) if 'robot_polygons_realworld_cm' in detections else []
            for robot in robots:
                if 'points_realworld_cm' in robot and robot['points_realworld_cm']:
                    world_points = [transform_point_to_world(robot_x, robot_y, robot_heading, x, y) for x, y in robot['points_realworld_cm']]
                    rel_points = [to_robot_frame(wx, wy) for wx, wy in world_points]
                    centroid = Polygon(rel_points).centroid
                    rel_points_scaled = [((p[0] - centroid.x) * 3.0 + centroid.x, (p[1] - centroid.y) * 3.0 + centroid.y) for p in rel_points]
                    for pt in rel_points_scaled:
                        p = Point(pt)
                        if front_poly.contains(p):
                            front_objects.append({'type': 'robot', 'polygon': rel_points_scaled})
                            print('[DEBUG][BOTH] RAW robot point (3x) in FRONT box')
                            break
                    for pt in rel_points_scaled:
                        p = Point(pt)
                        if right_poly.contains(p):
                            right_objects.append({'type': 'robot', 'polygon': rel_points_scaled})
                            print('[DEBUG][BOTH] RAW robot point (3x) in RIGHT box')
                            break
                    for pt in rel_points_scaled:
                        p = Point(pt)
                        if left_poly.contains(p):
                            left_objects.append({'type': 'robot', 'polygon': rel_points_scaled})
                            print('[DEBUG][BOTH] RAW robot point (3x) in LEFT box')
                            break
            tracked = self.get_tracked_objects()
            for crater in tracked.get('craters', []):
                if hasattr(crater, 'polygon') and crater.polygon:
                    rel_points = [to_robot_frame(p[0], p[1]) for p in crater.polygon]
                    for pt in rel_points:
                        p = Point(pt)
                        if front_poly.contains(p):
                            front_objects.append({'type': 'crater', 'polygon': rel_points})
                            print('[DEBUG][BOTH] TRACKED crater point in FRONT box')
                            break
                    for pt in rel_points:
                        p = Point(pt)
                        if right_poly.contains(p):
                            right_objects.append({'type': 'crater', 'polygon': rel_points})
                            print('[DEBUG][BOTH] TRACKED crater point in RIGHT box')
                            break
                    for pt in rel_points:
                        p = Point(pt)
                        if left_poly.contains(p):
                            left_objects.append({'type': 'crater', 'polygon': rel_points})
                            print('[DEBUG][BOTH] TRACKED crater point in LEFT box')
                            break
            for tape in tracked.get('tapes', []):
                if hasattr(tape, 'polygon') and tape.polygon:
                    rel_points = [to_robot_frame(p[0], p[1]) for p in tape.polygon]
                    for pt in rel_points:
                        p = Point(pt)
                        if front_poly.contains(p):
                            front_objects.append({'type': 'tape', 'polygon': rel_points})
                            print('[DEBUG][BOTH] TRACKED tape point in FRONT box')
                            break
                    for pt in rel_points:
                        p = Point(pt)
                        if right_poly.contains(p):
                            right_objects.append({'type': 'tape', 'polygon': rel_points})
                            print('[DEBUG][BOTH] TRACKED tape point in RIGHT box')
                            break
                    for pt in rel_points:
                        p = Point(pt)
                        if left_poly.contains(p):
                            left_objects.append({'type': 'tape', 'polygon': rel_points})
                            print('[DEBUG][BOTH] TRACKED tape point in LEFT box')
                            break
            for box in tracked.get('boxes', []):
                if hasattr(box, 'center'):
                    rel = to_robot_frame(box.center[0], box.center[1])
                    circle = Point(rel).buffer(50.0)
                    if front_poly.intersects(circle):
                        front_objects.append({'type': 'box', 'center': box.center})
                        print('[DEBUG][BOTH] TRACKED box (50cm circle) in FRONT box')
                    if right_poly.intersects(circle):
                        right_objects.append({'type': 'box', 'center': box.center})
                        print('[DEBUG][BOTH] TRACKED box (50cm circle) in RIGHT box')
                    if left_poly.intersects(circle):
                        left_objects.append({'type': 'box', 'center': box.center})
                        print('[DEBUG][BOTH] TRACKED box (50cm circle) in LEFT box')
            return {'front': front_objects, 'right': right_objects, 'left': left_objects}
        elif source == 'raw':
            pass
        elif source == 'tracked':
            pass
        else:
            print("[AUTO] Invalid source for check_objects_in_avoidance_boxes")
            return {'front': [], 'right': [], 'left': []}
    def determine_avoidance_direction(self, blocking_objects, target_object=None):
        if getattr(self, 'avoidance_direction_mode', 0) == 1:
            print('[AVOIDANCE] Forced avoidance direction: right (CW)')
            return 'right'
        elif getattr(self, 'avoidance_direction_mode', 0) == -1:
            print('[AVOIDANCE] Forced avoidance direction: left (CCW)')
            return 'left'
        current_objects = self.check_objects_in_avoidance_boxes(source='tracked')
        if not current_objects['front']:
            return None
        print(f"[AVOIDANCE] Last successful avoidance direction: {self.last_avoidance_direction}")
        if self.last_avoidance_direction:
            print(f"[AVOIDANCE] Trying remembered direction: {self.last_avoidance_direction}")
            if self.last_avoidance_direction == 'right':
                if len(current_objects['right']) <= len(current_objects['left']):
                    print(f"[AVOIDANCE] Using remembered direction: {self.last_avoidance_direction}")
                    return self.last_avoidance_direction
            else:
                if len(current_objects['left']) <= len(current_objects['right']):
                    print(f"[AVOIDANCE] Using remembered direction: {self.last_avoidance_direction}")
                    return self.last_avoidance_direction
        right_obstacles = len(current_objects['right'])
        left_obstacles = len(current_objects['left'])
        if right_obstacles < left_obstacles:
            chosen_direction = 'right'
        elif left_obstacles < right_obstacles:
            chosen_direction = 'left'
        else:
            chosen_direction = self.avoidance_direction_preference or 'right'
        print(f"[AVOIDANCE] Choosing direction based on obstacles: {chosen_direction} (right: {right_obstacles}, left: {left_obstacles})")
        return chosen_direction
    def _perform_avoidance_maneuver(self, objects_in_boxes, target_object, side_name):
        print(f"[AVOIDANCE] Moving back 2cm before turning")
        while True:
            self.move_forward(-4.0)
            self.detect_and_update()
            detections = self.imprint_and_detect()
            objects_in_boxes_both = self.check_objects_in_avoidance_boxes(source='both', detections=detections)
            if not objects_in_boxes_both['front']:
                print("[AVOIDANCE] Path clear after moving back repeatedly, but will still perform avoidance turn")
                break
            print("[AVOIDANCE] Still blocked after moving back, will try turning")
            break
        avoidance_dir = self.determine_avoidance_direction(objects_in_boxes_both['front'], target_object)
        if avoidance_dir is None:
            print("[AVOIDANCE] No objects in front after moving back, continuing normally")
            return
        print(f"[AVOIDANCE] Chosen avoidance direction: {avoidance_dir}")
        total_turn_angle = 0
        max_turns_in_direction = 10
        turn_angle = 30.0 if avoidance_dir == 'right' else -30.0
        objects_after_turn = None
        for turn_attempt in range(max_turns_in_direction):
            print(f"[AVOIDANCE] Turn attempt {turn_attempt + 1}: turning {turn_angle:.1f}° {avoidance_dir}")
            self.turn(turn_angle)
            self.detect_and_update()
            detections = self.imprint_and_detect()
            objects_after_turn = self.check_objects_in_avoidance_boxes(source='both', detections=detections)
            if not objects_after_turn['front']:
                print(f"[AVOIDANCE] Path clear after {total_turn_angle + abs(turn_angle):.1f}° turn")
                self.last_avoidance_direction = avoidance_dir
                print(f"[AVOIDANCE] Remembered successful direction: {avoidance_dir}")
                self.move_forward(7.0)
                return
            else:
                print(f"[AVOIDANCE] Still blocked after {total_turn_angle + abs(turn_angle):.1f}° turn, trying more")
            total_turn_angle += abs(turn_angle)
        while True:
            print(f"[AVOIDANCE] Still blocked after turning, moving back 2cm and will immediately turn again")
            self.move_forward(-4.0)
            self.detect_and_update()
            detections = self.imprint_and_detect()
            objects_after_back = self.check_objects_in_avoidance_boxes(source='both', detections=detections)
            avoidance_dir = self.determine_avoidance_direction(objects_after_back['front'], target_object)
            if avoidance_dir is None:
                print(f"[AVOIDANCE] Path clear after repeated back moves")
                return
            print(f"[AVOIDANCE] Chosen avoidance direction after back: {avoidance_dir}")
            self.turn(turn_angle if avoidance_dir == 'right' else -turn_angle)
            self.detect_and_update()
            detections = self.imprint_and_detect()
            objects_after_turn = self.check_objects_in_avoidance_boxes(source='both', detections=detections)
            if not objects_after_turn['front']:
                print(f"[AVOIDANCE] Path clear after turn following repeated back moves")
                self.last_avoidance_direction = avoidance_dir
                print(f"[AVOIDANCE] Remembered successful direction: {avoidance_dir}")
                return
    def _perform_heading_correction(self, target_x, target_y, target_object, side_name, max_correction_turns=6):
        robot_x = self.robot_ctl.x
        robot_y = self.robot_ctl.y
        dx = target_x - robot_x
        dy = target_y - robot_y
        required_heading = np.degrees(np.arctan2(dy, dx))
        current_heading = self.robot_ctl.heading
        initial_delta = (required_heading - current_heading + 180) % 360 - 180
        print(f"[HEADING] Current: {current_heading:.1f}°, Required: {required_heading:.1f}°, Delta: {initial_delta:.1f}°")
        if abs(initial_delta) < 5:
            print(f"[HEADING] Already close to required heading ({abs(initial_delta):.1f}°), going straight")
            self.move_forward(7.0)
            self.avoidance_movement_count += 1
            return
        for turn_attempt in range(max_correction_turns):
            current_heading = self.robot_ctl.heading
            delta = (required_heading - current_heading + 180) % 360 - 180
            if abs(delta) < 5:
                print(f"[HEADING] Reached required heading after {turn_attempt} turns")
                break
            turn_angle = 15.0 if delta > 0 else -15.0
            print(f"[HEADING] Turn attempt {turn_attempt + 1}: turning {turn_angle:.1f}° toward required heading")
            self.turn(turn_angle)
            detections = self.detect_and_update()
            objects_after_turn = self.check_objects_in_avoidance_boxes(source='tracked', detections=detections)
            if objects_after_turn is None:
                objects_after_turn = {'front': [], 'right': [], 'left': []}
            in_front_after_turn = bool(objects_after_turn['front'])
            in_side_after_turn = target_object in objects_after_turn[side_name] if target_object else False
            print(f"[HEADING] After turn: Front={len(objects_after_turn['front'])}, Side({side_name})={len(objects_after_turn[side_name])}, Target in side={in_side_after_turn}")
            if in_front_after_turn:
                print(f"[HEADING] Object now in front after turn, will trigger avoidance next step")
                break
            elif in_side_after_turn:
                print(f"[HEADING] Target object back in {side_name} side box after turn!")
                break
        detections = self.detect_and_update()
        final_objects = self.check_objects_in_avoidance_boxes(source='tracked', detections=detections)
        if final_objects is None:
            final_objects = {'front': [], 'right': [], 'left': []}
        final_in_side = target_object in final_objects[side_name] if target_object else False
        if final_in_side:
            print(f"[HEADING] Target object in {side_name} side box after correction - going straight")
        elif final_objects['front']:
            print(f"[HEADING] Objects in front after correction - avoidance will trigger next step")
        else:
            print(f"[HEADING] Target object still not in {side_name} side box, but going straight toward target")
        self.move_forward(7.0)
        self.avoidance_movement_count += 1
    def move_with_object_avoidance(self, target_x, target_y, target_object=None, max_steps=50, stop_on_tape=True):
        print(f"[AVOIDANCE] Starting movement to ({target_x:.1f}, {target_y:.1f})")
        side_name = "right" if getattr(self, 'robot_id', None) == "robot2" else "left"
        print(f"[AVOIDANCE] Following {side_name} side")
        for step in range(max_steps):
            robot_x = self.robot_ctl.x
            robot_y = self.robot_ctl.y
            distance_to_target = np.hypot(target_x - robot_x, target_y - robot_y)
            if distance_to_target < 5.0:
                print(f"[AVOIDANCE] Reached target at step {step}")
                break
            self.detect_and_update()
            detections = self.imprint_and_detect()
            boxes = self.check_objects_in_avoidance_boxes(source='both', detections=detections)
            front_objects = boxes['front']
            right_objects = boxes['right']
            left_objects = boxes['left']
            objects_in_boxes = {'front': front_objects, 'right': right_objects, 'left': left_objects}
            print(f"[AVOIDANCE] Step {step}: Front={len(front_objects)}, Right={len(right_objects)}, Left={len(left_objects)}")
            in_front = bool(front_objects)
            in_side = target_object in (right_objects if side_name == 'right' else left_objects) if target_object else False
            print(f"[AVOIDANCE] Step {step}: In front={in_front}, Target in {side_name} side={in_side}")
            if in_front:
                print(f"[AVOIDANCE] Objects detected in front: {len(front_objects)} - starting avoidance")
                self._perform_avoidance_maneuver(objects_in_boxes, target_object, side_name)
            elif in_side:
                print(f"[AVOIDANCE] Target object in {side_name} side box - going straight")
                self.move_forward(7.0)
                self.avoidance_movement_count += 1
            else:
                print(f"[AVOIDANCE] Target object lost from {side_name} side box - attempting heading correction")
                self._perform_heading_correction(target_x, target_y, target_object, side_name, max_correction_turns=6)
            time.sleep(0.1)
        print(f"[AVOIDANCE] Movement complete after {step} steps")
    def move_to_nearest_tape_with_avoidance(self, max_steps=50):
        print("[TAPE_NAV] Starting move to nearest tape with avoidance...")
        tracked = self.get_tracked_objects()
        min_dist = None
        nearest_tape_point = None
        robot_x = self.robot_ctl.x
        robot_y = self.robot_ctl.y
        for tape in tracked.get('tapes', []):
            if hasattr(tape, 'polygon') and tape.polygon:
                for p in tape.polygon:
                    dist = np.hypot(p[0] - robot_x, p[1] - robot_y)
                    if min_dist is None or dist < min_dist:
                        min_dist = dist
                        nearest_tape_point = p
            elif hasattr(tape, 'center'):
                p = tape.center
                dist = np.hypot(p[0] - robot_x, p[1] - robot_y)
                if min_dist is None or dist < min_dist:
                    min_dist = dist
                    nearest_tape_point = p
        if nearest_tape_point is None:
            print("[TAPE_NAV] No tape found to move to.")
            return
        print(f"[TAPE_NAV] Nearest tape at ({nearest_tape_point[0]:.1f}, {nearest_tape_point[1]:.1f}), distance {min_dist:.1f}cm")
        self.move_with_object_avoidance(nearest_tape_point[0], nearest_tape_point[1], max_steps=max_steps, stop_on_tape=True)
        print("[TAPE_NAV] Finished move to nearest tape.")
    def go_straight_until_tape_with_avoidance(self, max_distance=200.0, max_steps=50):
        print("[AUTO] Going straight with object avoidance until tape is detected...")
        heading_rad = np.radians(self.robot_ctl.heading)
        target_x = self.robot_ctl.x + max_distance * np.cos(heading_rad)
        target_y = self.robot_ctl.y + max_distance * np.sin(heading_rad)
        self.move_with_object_avoidance(target_x, target_y, max_steps=max_steps, stop_on_tape=True)
        print("[AUTO] Finished go_straight_until_tape_with_avoidance.")