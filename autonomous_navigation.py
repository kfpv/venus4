import time
import json
from datetime import datetime
from typing import Callable, Optional, Tuple
import math
from shapely.geometry import Polygon, LineString, Point
AVOIDANCE_STEP_CM = 7.0
AVOIDANCE_OBJECT_UPDATE_INTERVAL = 1
TAPE_SIDE_BOX_WIDTH = 16
TAPE_FRONT_DETECTION_DEPTH = 14.0
TAPE_SIDE_DEFAULT_WIDTH = 9.2
TAPE_TURN_ANGLE_FULL = 25.0
TAPE_TURN_ANGLE_HALF = 12.5
TAPE_SEARCH_TURN_ANGLE = 15.0
TAPE_APPROACH_RECT_HEIGHT1 = 10
TAPE_APPROACH_RECT_HEIGHT = 15
TAPE_APPROACH_RECT1_WIDTH = 0.9
TAPE_APPROACH_RECT2_WIDTH = 0.7
TAPE_APPROACH_RECT3_WIDTH = 0.7
TAPE_APPROACH_FORWARD_STEP = 4.0
class AutonomousNavigation:
    def __init__(self, mqtt_client, robot_id: str, visualizer=None):
        self.client = mqtt_client
        self.robot_id = robot_id
        self.visualizer = visualizer
        self.movement_completed = False
        self.imprint_completed = False
        self.avoidance_movement_count = 0
        self.stuck_detection_enabled = True
        self.stuck_positions = []
        self.stuck_check_radius = 20.0
        self.stuck_time_threshold = 60.0
        self.last_stuck_check = 0
        self.last_avoidance_direction = None
        self.avoidance_direction_preference = None
    def move_distance(self, distance_cm: float) -> bool:
        if not self.client:
            print("MQTT client not available")
            return False
        if self.visualizer.debug:
            print(f"[DEBUG] Starting move_distance: {distance_cm}cm")
        command = {
            'command': 'move',
            'distance_cm': distance_cm
        }
        if self.visualizer.debug:
            print(f"[DEBUG] Publishing move command: {command}")
        self.client.publish(f"{self.robot_id}/manual_control", json.dumps(command))
        while not self.visualizer.robot_data[self.robot_id]["navigating"]:
            time.sleep(0.1)
        while self.visualizer.robot_data[self.robot_id]["navigating"]:
            time.sleep(0.1)
        print(f"[DEBUG] Move completed successfully")
        return True
    def turn_degrees(self, degrees: float) -> bool:
        if not self.client:
            print("MQTT client not available")
            return False
        print(f"[DEBUG] Starting turn_degrees: {degrees}°")
        command = {
            'command': 'turn',
            'angle_degrees': degrees
        }
        print(f"[DEBUG] Publishing turn command: {command}")
        self.client.publish(f"{self.robot_id}/manual_control", json.dumps(command))
        nowtime= time.time()
        while (not self.visualizer.robot_data[self.robot_id]["navigating"]) and (not time.time()-nowtime > 1):
            print("waiting for turn to start")
            time.sleep(0.1)
        while self.visualizer.robot_data[self.robot_id]["navigating"]:
            time.sleep(1)
        print(f"[DEBUG] Turn completed successfully")
        return True
    def add_objects(self) -> bool:
        if not self.client:
            print("MQTT client not available")
            return False
        print("[DEBUG] Starting add_objects")
        self.visualizer.robot_imprint_states[self.robot_id]["command_pending"] = True
        self.visualizer.robot_imprint_states[self.robot_id]["imprint_for_add_objects"] = True
        self.visualizer.robot_imprint_states[self.robot_id]["command_time"] = time.time()
        command = {
            'command': 'imprint_scene',
            'timestamp': datetime.now().isoformat()
        }
        if self.visualizer.debug:
            print(f"[DEBUG] Publishing imprint command: {command}")
        self.client.publish(f"{self.robot_id}/manual_control", json.dumps(command))
        while self.visualizer.robot_imprint_states[self.robot_id]["command_pending"]:
            time.sleep(0.1)
        if self.visualizer.debug:
            print("[DEBUG] Imprint completed successfully")
        return True
    def handle_navigation_status(self, status_data: dict):
        if self.visualizer:
            self.visualizer.handle_navigation_status(status_data, self.robot_id)
    def check_objects_in_box(self, right_width: float = 10.0, left_width: float = 10.0,
                           front_depth: float = 15.0, back_depth: float = 16.0) -> dict:
        if not self.visualizer:
            print("Visualizer not available")
            return {'front': [], 'back': []}
        robot_x = self.visualizer.robot_data[self.robot_id]["x"]
        robot_y = self.visualizer.robot_data[self.robot_id]["y"]
        robot_heading = self.visualizer.robot_data[self.robot_id]["heading"]
        heading_rad = math.radians(robot_heading)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        box_polygon = [
            (front_depth, -right_width),
            (front_depth, left_width),
            (-back_depth, left_width),
            (-back_depth, -right_width)
        ]
        box_poly = Polygon(box_polygon)
        front_objects = []
        back_objects = []
        tracked_objects = self.visualizer.tracked_objects
        def to_robot_frame(x, y):
            dx = x - robot_x
            dy = y - robot_y
            rel_x = dx * cos_h + dy * sin_h
            rel_y = -dx * sin_h + dy * cos_h
            return (rel_x, rel_y)
        for box in tracked_objects['boxes']:
            rel = to_robot_frame(box.center[0], box.center[1])
            if box_poly.contains(Point(rel)):
                dist = math.hypot(rel[0], rel[1])
                front_objects.append({'object': box, 'distance': dist})
        for cube in tracked_objects['cubes']:
            if cube.polygon:
                rel_points = [to_robot_frame(p[0], p[1]) for p in cube.polygon]
                cube_poly = Polygon(rel_points)
                if box_poly.intersects(cube_poly):
                    dists = [math.hypot(rx, ry) for rx, ry in rel_points]
                    min_dist = min(dists)
                    front_objects.append({'object': cube, 'distance': min_dist})
            else:
                rel = to_robot_frame(cube.center[0], cube.center[1])
                if box_poly.contains(Point(rel)):
                    dist = math.hypot(rel[0], rel[1])
                    front_objects.append({'object': cube, 'distance': dist})
        for tape in tracked_objects['tapes']:
            if tape.polygon:
                rel_points = [to_robot_frame(p[0], p[1]) for p in tape.polygon]
                tape_line = LineString(rel_points)
                if box_poly.intersects(tape_line):
                    dists = [math.hypot(rx, ry) for rx, ry in rel_points]
                    min_dist = min(dists)
                    front_objects.append({'object': tape, 'distance': min_dist})
            else:
                rel = to_robot_frame(tape.center[0], tape.center[1])
                if box_poly.contains(Point(rel)):
                    dist = math.hypot(rel[0], rel[1])
                    front_objects.append({'object': tape, 'distance': dist})
        for crater in tracked_objects.get('craters', []):
            if crater.polygon:
                rel_points = [to_robot_frame(p[0], p[1]) for p in crater.polygon]
                crater_poly = Polygon(rel_points)
                if box_poly.intersects(crater_poly):
                    dists = [math.hypot(rx, ry) for rx, ry in rel_points]
                    min_dist = min(dists)
                    front_objects.append({'object': crater, 'distance': min_dist})
            else:
                rel = to_robot_frame(crater.center[0], crater.center[1])
                if box_poly.contains(Point(rel)):
                    dist = math.hypot(rel[0], rel[1])
                    front_objects.append({'object': crater, 'distance': dist})
        print(f"[DEBUG] Found {len(front_objects)} objects in front (rectangle intersection)")
        return {'front': front_objects, 'back': back_objects}
    def _point_in_polygon(self, point: tuple, polygon: list) -> bool:
        x, y = point
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside
    @staticmethod
    def get_current_robot_id_from_config(config_path="robots_config.json"):
        with open(config_path, "r") as f:
            robots = json.load(f)
        for robot in robots:
            if robot.get("is_current", False):
                return robot["robot_id"]
        return None
    def get_nearest_tape(self):
        objects = self.check_objects_in_box(front_depth=100.0, right_width=100.0, left_width=100.0, back_depth=100.0)
        min_dist = None
        min_heading = None
        min_world = None
        for obj in objects['front']:
            if hasattr(obj['object'], 'polygon'):
                rel_points = []
                for p in obj['object'].polygon:
                    robot_x = self.visualizer.robot_data[self.robot_id]["x"]
                    robot_y = self.visualizer.robot_data[self.robot_id]["y"]
                    robot_heading = self.visualizer.robot_data[self.robot_id]["heading"]
                    heading_rad = math.radians(robot_heading)
                    cos_h = math.cos(heading_rad)
                    sin_h = math.sin(heading_rad)
                    dx = p[0] - robot_x
                    dy = p[1] - robot_y
                    rel_x = dx * cos_h + dy * sin_h
                    rel_y = -dx * sin_h + dy * cos_h
                    dist = math.hypot(rel_x, rel_y)
                    heading_deg = math.degrees(math.atan2(rel_y, rel_x))
                    if min_dist is None or dist < min_dist:
                        min_dist = dist
                        min_heading = heading_deg
                        min_world = p
        if min_dist is not None:
            return (min_dist, min_heading, min_world[0], min_world[1])
        return None
    def select_tape_target(self):
        robot_id = self.robot_id
        objects = self.check_objects_in_box(front_depth=100.0, right_width=100.0, left_width=100.0, back_depth=100.0)
        tape_points = []
        for obj in objects['front']:
            if hasattr(obj['object'], 'polygon') and (
                (hasattr(obj['object'], 'class_name') and obj['object'].class_name.lower() == 'tape') or
                (hasattr(obj['object'], 'type') and obj['object'].type.lower() == 'tape')
            ):
                tape_points.extend(obj['object'].polygon)
        if not tape_points:
            return None
        if robot_id == "robot2":
            target = max(tape_points, key=lambda p: (p[0], -p[1]))
        else:
            target = min(tape_points, key=lambda p: (p[0], -p[1]))
        return target
    def tape_in_view(self, front_depth=15.0, right_width=TAPE_SIDE_DEFAULT_WIDTH, left_width=TAPE_SIDE_DEFAULT_WIDTH, back_depth=0.0):
        if not self.visualizer:
            return False
        robot_x = self.visualizer.robot_data[self.robot_id]["x"]
        robot_y = self.visualizer.robot_data[self.robot_id]["y"]
        robot_heading = self.visualizer.robot_data[self.robot_id]["heading"]
        heading_rad = math.radians(robot_heading)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        front_box = [
            (front_depth, -right_width),
            (front_depth, left_width),
            (-back_depth, left_width),
            (-back_depth, -right_width)
        ]
        front_poly = Polygon(front_box)
        def to_robot_frame(x, y):
            dx = x - robot_x
            dy = y - robot_y
            rel_x = dx * cos_h + dy * sin_h
            rel_y = -dx * sin_h + dy * cos_h
            return (rel_x, rel_y)
        for tape in self.visualizer.tracked_objects['tapes']:
            if tape.polygon:
                rel_points = [to_robot_frame(p[0], p[1]) for p in tape.polygon]
                tape_line = LineString(rel_points)
                if front_poly.intersects(tape_line):
                    return True
            else:
                rel = to_robot_frame(tape.center[0], tape.center[1])
                if front_poly.contains(Point(rel)):
                    return True
        return False
    def get_nearest_tape_object(self):
        objects = self.check_objects_in_box(front_depth=100.0, right_width=100.0, left_width=100.0, back_depth=100.0)
        nearest_tape = None
        min_dist = float('inf')
        for obj in objects['front']:
            if hasattr(obj['object'], 'polygon'):
                if obj['distance'] < min_dist:
                    min_dist = obj['distance']
                    nearest_tape = obj['object']
        return nearest_tape
    def move_to_target_with_checks(self, target_x, target_y, step_cm=15.0, max_steps=20, use_avoidance=False):
        if True:
            print("using avoidance!")
            nearest_tape = self.get_nearest_tape_object()
            self.move_with_object_avoidance(target_x, target_y, nearest_tape, max_steps, stop_on_tape=True)
            return
        for step in range(max_steps):
            self._wait_for_other_robot_to_clear()
            if self.tape_in_view():
                print("Tape detected in view! Starting tape approach and alignment...")
                alignment_success = self.perform_tape_approach_and_alignment()
                if alignment_success:
                    print("Tape alignment successful! Starting normal tape border following...")
                    robot_id = self.robot_id
                    if robot_id == "robot1":
                        direction = "cw"
                    else:
                        direction = "ccw"
                    print(f"Robot {robot_id} following tape border in {direction.upper()} direction")
                    self.drive_around_tape_border(direction)
                    if self.should_end_tape_following_loop():
                        print("Tape following complete - autonomous navigation ending")
                        return
                    else:
                        print("Tape following phase complete - continuing navigation")
                else:
                    print("Tape alignment failed - continuing with original navigation")
            robot_x = self.visualizer.robot_data[self.robot_id]["x"]
            robot_y = self.visualizer.robot_data[self.robot_id]["y"]
            robot_heading = self.visualizer.robot_data[self.robot_id]["heading"]
            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.hypot(dx, dy)
            if distance < 2.0:
                print(f"Arrived at target ({target_x:.1f}, {target_y:.1f})")
                break
            target_angle = math.degrees(math.atan2(dy, dx))
            delta_angle = (target_angle - robot_heading + 180) % 360 - 180
            print(f"Step {step+1}: Turning {delta_angle:.1f}° toward target, then adding objects.")
            self.turn_degrees(delta_angle)
            self.add_objects()
            robot_check = self.check_other_robot_in_box()
            if robot_check['yolo'] or robot_check['robot_data']:
                print(f"[WARNING] Other robot detected nearby! YOLO: {robot_check['yolo']}, robot_data: {robot_check['robot_data']}")
            move_dist = min(step_cm, distance)
            print(f"Moving {move_dist:.1f}cm toward target.")
            self.move_distance(move_dist)
            self.add_objects()
        print("Movement to target complete or max steps reached.")
    def start_autonomous_navigation(self, use_avoidance=True, mode="straight_line"):
        print("Starting autonomous navigation sequence...")
        robot_id = self.robot_id
        print(f"Current robot: {robot_id}")
        if robot_id is None:
            print("Could not determine current robot from config.")
            return
        if mode == "straight_line":
            print("Using straight-line obstacle avoidance mode")
            self.straight_line_avoidance_mode()
            return
        elif mode == "straight_line_simple":
            print("Using simple straight-line mode (no initial scanning)")
            self.add_objects()
            self.straight_line_avoidance_mode()
            return
        print("Step 1: Add objects (imprint)")
        self.add_objects()
        nearest = self.get_nearest_tape()
        if nearest:
            print(f"Nearest tape: distance={nearest[0]:.1f}cm, heading={nearest[1]:.1f}°, world=({nearest[2]:.1f}, {nearest[3]:.1f})")
        else:
            print("No tape found.")
        if robot_id == "robot1":
            turn_seq = [40, 40, -120, -40]
        else:
            turn_seq = [-40, -40, 120, 40]
            self._wait_for_other_robot_to_clear(min_dist=10.0, check_dist=8)
        total_turn = 0
        for idx, turn in enumerate(turn_seq):
            print(f"Step {idx+2}: Turn {turn} degrees and add objects")
            self.turn_degrees(turn)
            total_turn += turn
            self.add_objects()
            nearest = self.get_nearest_tape()
            if nearest:
                print(f"Nearest tape: distance={nearest[0]:.1f}cm, heading={nearest[1]:.1f}°, world=({nearest[2]:.1f}, {nearest[3]:.1f})")
            else:
                print("No tape found.")
        print("Returning to original heading by turning back...")
        if abs(total_turn) > 1.0:
            print(f"Turning {-total_turn:.1f} degrees to return to original heading.")
            self.turn_degrees(-total_turn)
        else:
            print("Already at original heading.")
        print("\n--- Move to tape target ---")
        target = self.select_tape_target()
        if target:
            print(f"Target tape point: ({target[0]:.1f}, {target[1]:.1f})")
            if use_avoidance:
                print("Using object avoidance system")
                nearest_tape = self.get_nearest_tape_object()
                self.move_with_object_avoidance(target[0], target[1], nearest_tape, stop_on_tape=True)
            else:
                self.move_to_target_with_checks(target[0], target[1])
        else:
            print("No tape target found.")
            print("Driving straight with avoidance until any tape is detected...")
            robot_x = self.visualizer.robot_data[self.robot_id]["x"]
            robot_y = self.visualizer.robot_data[self.robot_id]["y"]
            robot_heading = self.visualizer.robot_data[self.robot_id]["heading"]
            heading_rad = math.radians(robot_heading)
            target_x = robot_x + 200.0 * math.cos(heading_rad)
            target_y = robot_y + 200.0 * math.sin(heading_rad)
            self.move_with_object_avoidance(target_x, target_y, None, stop_on_tape=True)
        print("\n--- Checking for tape border following ---")
        if self.tape_in_view():
            print("Tape detected in view! Starting tape approach and alignment...")
            alignment_success = self.perform_tape_approach_and_alignment()
            if alignment_success:
                print("Tape alignment successful! Starting normal tape border following...")
                robot_id = self.robot_id
                if robot_id == "robot1":
                    direction = "cw"
                else:
                    direction = "ccw"
                print(f"Robot {robot_id} following tape border in {direction.upper()} direction")
                self.drive_around_tape_border(direction)
            else:
                print("Tape alignment failed - ending autonomous navigation")
        else:
            print("No tape in view - autonomous navigation complete")
    def perform_tape_approach_and_alignment(self, max_steps=50):
        print("[TAPE_APPROACH] Starting tape approach and alignment mode")
        for step in range(max_steps):
            print(f"[TAPE_APPROACH] Step {step}: Adding objects")
            self.add_objects()
            approach_boxes = self.check_tape_in_approach_rectangles()
            has_rect1 = bool(approach_boxes['rect1'])
            has_rect2 = bool(approach_boxes['rect2'])
            has_rect3 = bool(approach_boxes['rect3'])
            print(f"[TAPE_APPROACH] Rectangle detection - Rect1:{has_rect1}, Rect2:{has_rect2}, Rect3:{has_rect3}")
            robot_id = self.robot_id
            print(robot_id)
            turn_angle = 90.0 if robot_id == "robot1" else -90.0
            if has_rect2 and not has_rect1 and not has_rect3:
                print("[TAPE_APPROACH] Target alignment achieved - only middle rectangle sees tape")
                print(f"[TAPE_APPROACH] Performing {turn_angle}-degree alignment turn")
                self.turn_degrees(turn_angle)
                self.add_objects()
                print("[TAPE_APPROACH] Tape approach and alignment complete!")
                return True
            elif has_rect1 and not has_rect2 and not has_rect3:
                print("[TAPE_APPROACH] Only closest rectangle sees tape - moving back to try to get tape into middle rectangle")
                self.move_distance(-2.0)
            elif has_rect1 and not has_rect2 and not has_rect3:
                print("[TAPE_APPROACH] Alternative alignment achieved - only closest rectangle sees tape")
                print(f"[TAPE_APPROACH] Performing {turn_angle}-degree alignment turn")
                self.turn_degrees(turn_angle)
                self.add_objects()
                print("[TAPE_APPROACH] Tape approach and alignment complete!")
                return True
            elif not has_rect1 and not has_rect2 and not has_rect3:
                print(f"[TAPE_APPROACH] No rectangles see tape - moving forward {TAPE_APPROACH_FORWARD_STEP}cm")
                self.move_distance(TAPE_APPROACH_FORWARD_STEP)
            else:
                print("[TAPE_APPROACH] Multiple rectangles see tape - performing corrective maneuvers")
                if has_rect3:
                    print("[TAPE_APPROACH] Farthest rectangle sees tape - moving forward to get closer")
                    self.move_distance(TAPE_APPROACH_FORWARD_STEP / 2)
                elif has_rect1 and has_rect2:
                    print("[TAPE_APPROACH] Both closest and middle rectangles see tape - fine adjustment needed")
                    self.move_distance(-1.0)
                elif has_rect1 and not has_rect2:
                    print("[TAPE_APPROACH] Only closest rectangle sees tape - acceptable but could be better")
                    print(f"[TAPE_APPROACH] Performing {turn_angle}-degree alignment turn")
                    self.turn_degrees(turn_angle)
                    self.add_objects()
                    print("[TAPE_APPROACH] Tape approach and alignment complete!")
                    return True
                elif has_rect2 and has_rect3:
                    print("[TAPE_APPROACH] Middle and farthest rectangles see tape - moving back slightly")
                    self.move_distance(-1.0)
                else:
                    print("[TAPE_APPROACH] Unexpected detection pattern - trying forward movement")
                    self.move_distance(TAPE_APPROACH_FORWARD_STEP / 2)
            time.sleep(0.1)
        print("[TAPE_APPROACH] Warning: Could not achieve tape alignment within max steps")
        return False
    def check_other_robot_in_box(self, left=10.0, right=10.0, front=15.0, back=15.0):
        robot_id = self.robot_id
        if robot_id != "robot2":
            print(f"[ROBOT_DETECTION] Skipping other robot detection for {robot_id}")
            return {"yolo": False, "robot_data": False, "yolo_details": [], "robot_data_details": None, "min_yolo_distance": None, "robot_data_distance": None}
        robot_x = self.visualizer.robot_data[self.robot_id]["x"]
        robot_y = self.visualizer.robot_data[self.robot_id]["y"]
        robot_heading = self.visualizer.robot_data[self.robot_id]["heading"]
        heading_rad = math.radians(robot_heading)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        def to_robot_frame(x, y):
            dx = x - robot_x
            dy = y - robot_y
            rel_x = dx * cos_h + dy * sin_h
            rel_y = -dx * sin_h + dy * cos_h
            return (rel_x, rel_y)
        robot_box_polygon = [
            (front, -right),
            (front, left),
            (-back, left),
            (-back, -right)
        ]
        robot_box_poly = Polygon(robot_box_polygon)
        all_robots = list(self.visualizer.robot_data.keys())
        other_robot_id = None
        for rid in all_robots:
            if rid != self.robot_id:
                other_robot_id = rid
                break
        if not other_robot_id:
            return {"yolo": False, "robot_data": False, "yolo_details": [], "robot_data_details": None}
        found_yolo = False
        yolo_details = []
        min_yolo_dist = None
        latest_robot_detections = self.visualizer.robot_data[self.robot_id].get("raw_detected_robots", [])
        for det in latest_robot_detections:
            if 'world_polygon' in det and len(det['world_polygon']) >= 3:
                rel_points = [to_robot_frame(p[0], p[1]) for p in det['world_polygon']]
                robot_poly = Polygon(rel_points)
                if robot_box_poly.intersects(robot_poly):
                    found_yolo = True
                    dists = [math.hypot(rx, ry) for rx, ry in rel_points]
                    this_min = min(dists)
                    if min_yolo_dist is None or this_min < min_yolo_dist:
                        min_yolo_dist = this_min
                    yolo_details.append({"type": "polygon", "inside": True, "polygon": rel_points, "distance": this_min})
                else:
                    yolo_details.append({"type": "polygon", "inside": False, "polygon": rel_points})
            else:
                wx, wy = None, None
                if 'world_x' in det and 'world_y' in det:
                    wx, wy = det['world_x'], det['world_y']
                elif hasattr(det, 'world_x') and hasattr(det, 'world_y'):
                    wx, wy = det.world_x, det.world_y
                if wx is None or wy is None:
                    continue
                rel_x, rel_y = to_robot_frame(wx, wy)
                dist = math.hypot(rel_x, rel_y)
                if robot_box_poly.contains(Point(rel_x, rel_y)):
                    found_yolo = True
                    if min_yolo_dist is None or dist < min_yolo_dist:
                        min_yolo_dist = dist
                    yolo_details.append({"type": "center", "inside": True, "point": (rel_x, rel_y), "distance": dist})
                else:
                    yolo_details.append({"type": "center", "inside": False, "point": (rel_x, rel_y)})
        other_robot_pose = self.visualizer.robot_data[other_robot_id]
        rel_x, rel_y = to_robot_frame(other_robot_pose["x"], other_robot_pose["y"])
        dist_robot_data = math.hypot(rel_x, rel_y)
        inside_robot_data = robot_box_poly.contains(Point(rel_x, rel_y))
        robot_data_details = {"inside": inside_robot_data, "point": (rel_x, rel_y), "abs": (other_robot_pose["x"], other_robot_pose["y"]), "distance": dist_robot_data}
        return {"yolo": found_yolo, "robot_data": inside_robot_data, "yolo_details": yolo_details, "robot_data_details": robot_data_details, "min_yolo_distance": min_yolo_dist, "robot_data_distance": dist_robot_data}
    def check_objects_in_avoidance_boxes(self, target_object=None):
        if not self.visualizer:
            print("Visualizer not available")
            return {'front': [], 'right': [], 'left': []}
        robot_x = self.visualizer.robot_data[self.robot_id]["x"]
        robot_y = self.visualizer.robot_data[self.robot_id]["y"]
        robot_heading = self.visualizer.robot_data[self.robot_id]["heading"]
        print(f"[DEBUG] Robot at ({robot_x:.1f}, {robot_y:.1f}, {robot_heading:.1f}°)")
        heading_rad = math.radians(robot_heading)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
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
        def check_object_in_boxes(obj):
            results = {'front': False, 'right': False, 'left': False}
            if target_object and obj == target_object:
                return results
            if hasattr(obj, 'world_polygon') and obj.world_polygon:
                rel_points = [to_robot_frame(p[0], p[1]) for p in obj.world_polygon]
                obj_poly = Polygon(rel_points)
                if front_poly.intersects(obj_poly):
                    results['front'] = True
                if right_poly.intersects(obj_poly):
                    results['right'] = True
                if left_poly.intersects(obj_poly):
                    results['left'] = True
            else:
                if hasattr(obj, 'center'):
                    rel = to_robot_frame(obj.center[0], obj.center[1])
                elif hasattr(obj, 'world_x') and hasattr(obj, 'world_y'):
                    rel = to_robot_frame(obj.world_x, obj.world_y)
                else:
                    return results
                point = Point(rel)
                if front_poly.contains(point):
                    results['front'] = True
                if right_poly.contains(point):
                    results['right'] = True
                if left_poly.contains(point):
                    results['left'] = True
            return results
        raw_boxes = self.visualizer.robot_data[self.robot_id]["raw_detected_boxes"]
        total_objects = len(raw_boxes)
        print(f"[DEBUG] Checking {total_objects} raw boxes for obstacle detection")
        for obj in raw_boxes:
            if hasattr(obj, 'class_name') and obj.class_name.lower() == 'robot':
                scale = 3.0
                if hasattr(obj, 'world_polygon') and obj.world_polygon:
                    rel_points = [to_robot_frame(p[0], p[1]) for p in obj.world_polygon]
                    centroid = Polygon(rel_points).centroid
                    rel_points_scaled = [((p[0] - centroid.x) * scale + centroid.x, (p[1] - centroid.y) * scale + centroid.y) for p in rel_points]
                    obj_poly = Polygon(rel_points_scaled)
                    if front_poly.intersects(obj_poly):
                        front_objects.append(obj)
                        print(f"[DEBUG] RAW ROBOT in FRONT box (3x): {obj.class_name} at ({obj.world_x:.1f}, {obj.world_y:.1f})")
                    if right_poly.intersects(obj_poly):
                        right_objects.append(obj)
                    if left_poly.intersects(obj_poly):
                        left_objects.append(obj)
                else:
                    rel = to_robot_frame(obj.world_x, obj.world_y)
                    point = Point(rel)
                    if front_poly.buffer(2*scale).contains(point):
                        front_objects.append(obj)
                        print(f"[DEBUG] RAW ROBOT in FRONT box (3x): {obj.class_name} at ({obj.world_x:.1f}, {obj.world_y:.1f})")
                    if right_poly.buffer(2*scale).contains(point):
                        right_objects.append(obj)
                    if left_poly.buffer(2*scale).contains(point):
                        left_objects.append(obj)
            elif hasattr(obj, 'class_name') and obj.class_name.lower() == 'box':
                box_results = check_object_in_boxes(obj)
                if box_results['front']:
                    front_objects.append(obj)
                    print(f"[DEBUG] RAW BOX in FRONT box: {obj.class_name} at ({obj.world_x:.1f}, {obj.world_y:.1f})")
                if box_results['right']:
                    right_objects.append(obj)
                if box_results['left']:
                    left_objects.append(obj)
            else:
                box_results = check_object_in_boxes(obj)
                if box_results['front']:
                    front_objects.append(obj)
                if box_results['right']:
                    right_objects.append(obj)
                if box_results['left']:
                    left_objects.append(obj)
        tracked_objects = self.visualizer.tracked_objects
        for crater in tracked_objects.get('craters', []):
            if hasattr(crater, 'polygon') and crater.polygon:
                rel_points = [to_robot_frame(p[0], p[1]) for p in crater.polygon]
                crater_poly = Polygon(rel_points)
                if front_poly.intersects(crater_poly):
                    front_objects.append(crater)
                if right_poly.intersects(crater_poly):
                    right_objects.append(crater)
                if left_poly.intersects(crater_poly):
                    left_objects.append(crater)
            elif hasattr(crater, 'center'):
                rel = to_robot_frame(crater.center[0], crater.center[1])
                point = Point(rel)
                if front_poly.contains(point):
                    front_objects.append(crater)
                if right_poly.contains(point):
                    right_objects.append(crater)
                if left_poly.contains(point):
                    left_objects.append(crater)
        for box in tracked_objects.get('boxes', []):
            if hasattr(box, 'center'):
                rel = to_robot_frame(box.center[0], box.center[1])
                circle = Point(rel).buffer(20.0)
                if front_poly.intersects(circle):
                    front_objects.append(box)
                    print(f"[DEBUG] TRACKED BOX (20cm circle) in FRONT box: {getattr(box, 'class_name', 'box')} at {box.center}")
                if right_poly.intersects(circle):
                    right_objects.append(box)
                if left_poly.intersects(circle):
                    left_objects.append(box)
        print(f"[DEBUG] Detection results: {len(front_objects)} front, {len(right_objects)} right, {len(left_objects)} left")
        return {'front': front_objects, 'right': right_objects, 'left': left_objects}
    def determine_avoidance_direction(self, blocking_objects, target_object=None):
        current_objects = self.check_objects_in_avoidance_boxes(target_object)
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
    def move_with_object_avoidance(self, target_x, target_y, target_object=None, max_steps=50, stop_on_tape=True):
        print(f"[AVOIDANCE] Starting movement to ({target_x:.1f}, {target_y:.1f})")
        robot_id = self.robot_id
        following_right = (robot_id == "robot2")
        side_name = "right" if following_right else "left"
        print(f"[AVOIDANCE] Following {side_name} side for {robot_id}")
        for step in range(max_steps):
            self._wait_for_other_robot_to_clear()
            self.update_position_history()
            if self.is_robot_stuck():
                print("[AVOIDANCE] Robot is stuck! Performing escape maneuver")
                self.perform_stuck_escape_maneuver()
                continue
            robot_x = self.visualizer.robot_data[self.robot_id]["x"]
            robot_y = self.visualizer.robot_data[self.robot_id]["y"]
            distance_to_target = math.hypot(target_x - robot_x, target_y - robot_y)
            if distance_to_target < 5.0:
                print(f"[AVOIDANCE] Reached target at step {step}")
                break
            if stop_on_tape and self.tape_in_view():
                print("[AVOIDANCE] Tape detected in view! Stopping movement to target.")
                break
            print(f"[AVOIDANCE] Adding objects before checking boxes (step {step})")
            self.add_objects()
            objects_in_boxes = self.check_objects_in_avoidance_boxes(target_object)
            in_front = bool(objects_in_boxes['front'])
            in_side = target_object in objects_in_boxes[side_name] if target_object else False
            print(f"[AVOIDANCE] Step {step}: Front={len(objects_in_boxes['front'])}, Side({side_name})={len(objects_in_boxes[side_name])}, Target in side={in_side}")
            if in_front:
                print(f"[AVOIDANCE] Objects detected in front: {len(objects_in_boxes['front'])} - starting avoidance")
                self._perform_avoidance_maneuver(objects_in_boxes, target_object, side_name)
            elif in_side:
                print(f"[AVOIDANCE] Target object in {side_name} side box - going straight")
                self.move_distance(AVOIDANCE_STEP_CM)
                self.avoidance_movement_count += 1
            else:
                print(f"[AVOIDANCE] Target object lost from {side_name} side box - attempting heading correction")
                self._perform_heading_correction(target_x, target_y, target_object, side_name, max_correction_turns=6)
            time.sleep(0.1)
        print(f"[AVOIDANCE] Movement complete after {step} steps")
    def _perform_avoidance_maneuver(self, objects_in_boxes, target_object, side_name):
        print(f"[AVOIDANCE] Moving back 2cm before turning")
        self.move_distance(-2.0)
        avoidance_dir = self.determine_avoidance_direction(objects_in_boxes['front'], target_object)
        if avoidance_dir is None:
            print("[AVOIDANCE] No objects in front after moving back, continuing normally")
            return
        print(f"[AVOIDANCE] Chosen avoidance direction: {avoidance_dir}")
        total_turn_angle = 0
        max_turns_in_direction = 4
        turn_angle = 30.0 if avoidance_dir == 'right' else -30.0
        objects_after_turn = None
        for turn_attempt in range(max_turns_in_direction):
            print(f"[AVOIDANCE] Turn attempt {turn_attempt + 1}: turning {turn_angle:.1f}° {avoidance_dir}")
            self.turn_degrees(turn_angle)
            total_turn_angle += abs(turn_angle)
            self.add_objects()
            objects_after_turn = self.check_objects_in_avoidance_boxes(target_object)
            if not objects_after_turn['front']:
                print(f"[AVOIDANCE] Path clear after {total_turn_angle:.1f}° turn")
                self.last_avoidance_direction = avoidance_dir
                print(f"[AVOIDANCE] Remembered successful direction: {avoidance_dir}")
                break
            else:
                print(f"[AVOIDANCE] Still blocked after {total_turn_angle:.1f}° turn, trying more")
        if objects_after_turn and objects_after_turn['front']:
            print(f"[AVOIDANCE] Still blocked after {total_turn_angle:.1f}° in {avoidance_dir} direction, trying opposite")
            opposite_dir = 'left' if avoidance_dir == 'right' else 'right'
            opposite_turn_angle = -60.0 if avoidance_dir == 'right' else 60.0
            print(f"[AVOIDANCE] Switching to opposite direction: {opposite_dir}")
            self.turn_degrees(opposite_turn_angle)
            self.add_objects()
            objects_after_opposite = self.check_objects_in_avoidance_boxes(target_object)
            if not objects_after_opposite['front']:
                print(f"[AVOIDANCE] Path clear after switching to {opposite_dir}")
                self.last_avoidance_direction = opposite_dir
                print(f"[AVOIDANCE] Remembered successful direction: {opposite_dir}")
            else:
                print(f"[AVOIDANCE] Still blocked after switching direction, moving back more")
                self.move_distance(-7.0)
        print(f"[AVOIDANCE] Moving {AVOIDANCE_STEP_CM}cm forward in new direction (no heading correction)")
        self.move_distance(AVOIDANCE_STEP_CM)
        self.avoidance_movement_count += 1
        self.add_objects()
    def _perform_heading_correction(self, target_x, target_y, target_object, side_name, max_correction_turns=6):
        robot_x = self.visualizer.robot_data[self.robot_id]["x"]
        robot_y = self.visualizer.robot_data[self.robot_id]["y"]
        dx = target_x - robot_x
        dy = target_y - robot_y
        required_heading = math.degrees(math.atan2(dy, dx))
        current_heading = self.visualizer.robot_data[self.robot_id]["heading"]
        initial_delta = (required_heading - current_heading + 180) % 360 - 180
        print(f"[HEADING] Current: {current_heading:.1f}°, Required: {required_heading:.1f}°, Delta: {initial_delta:.1f}°")
        if abs(initial_delta) < 5:
            print(f"[HEADING] Already close to required heading ({abs(initial_delta):.1f}°), going straight")
            self.move_distance(AVOIDANCE_STEP_CM)
            self.avoidance_movement_count += 1
            return
        for turn_attempt in range(max_correction_turns):
            current_heading = self.visualizer.robot_data[self.robot_id]["heading"]
            delta = (required_heading - current_heading + 180) % 360 - 180
            if abs(delta) < 5:
                print(f"[HEADING] Reached required heading after {turn_attempt} turns")
                break
            turn_angle = 15.0 if delta > 0 else -15.0
            print(f"[HEADING] Turn attempt {turn_attempt + 1}: turning {turn_angle:.1f}° toward required heading")
            self.turn_degrees(turn_angle)
            print(f"[HEADING] Adding objects after turn to update detection")
            self.add_objects()
            objects_after_turn = self.check_objects_in_avoidance_boxes(target_object)
            in_front_after_turn = bool(objects_after_turn['front'])
            in_side_after_turn = target_object in objects_after_turn[side_name] if target_object else False
            print(f"[HEADING] After turn: Front={len(objects_after_turn['front'])}, Side({side_name})={len(objects_after_turn[side_name])}, Target in side={in_side_after_turn}")
            if in_front_after_turn:
                print(f"[HEADING] Object now in front after turn, will trigger avoidance next step")
                break
            elif in_side_after_turn:
                print(f"[HEADING] Target object back in {side_name} side box after turn!")
                break
        final_objects = self.check_objects_in_avoidance_boxes(target_object)
        final_in_side = target_object in final_objects[side_name] if target_object else False
        if final_in_side:
            print(f"[HEADING] Target object in {side_name} side box after correction - going straight")
        elif final_objects['front']:
            print(f"[HEADING] Objects in front after correction - avoidance will trigger next step")
        else:
            print(f"[HEADING] Target object still not in {side_name} side box, but going straight toward target")
        self.move_distance(AVOIDANCE_STEP_CM)
        self.avoidance_movement_count += 1
    def check_tape_in_detection_boxes(self):
        if not self.visualizer:
            print("Visualizer not available")
            return {'front': [], 'left': [], 'center': [], 'right': [], 'edge_left': [], 'edge_center': [], 'edge_right': []}
        robot_x = self.visualizer.robot_data[self.robot_id]["x"]
        robot_y = self.visualizer.robot_data[self.robot_id]["y"]
        robot_heading = self.visualizer.robot_data[self.robot_id]["heading"]
        print(f"[TAPE] Robot at ({robot_x:.1f}, {robot_y:.1f}, {robot_heading:.1f}°)")
        heading_rad = math.radians(robot_heading)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        front_box = [
            (TAPE_FRONT_DETECTION_DEPTH, -TAPE_SIDE_DEFAULT_WIDTH),
            (TAPE_FRONT_DETECTION_DEPTH, TAPE_SIDE_DEFAULT_WIDTH),
            (0.0, TAPE_SIDE_DEFAULT_WIDTH),
            (0.0, -TAPE_SIDE_DEFAULT_WIDTH)
        ]
        left_box = [
            (TAPE_SIDE_DEFAULT_WIDTH, -TAPE_SIDE_DEFAULT_WIDTH),
            (TAPE_SIDE_DEFAULT_WIDTH, -TAPE_SIDE_DEFAULT_WIDTH + TAPE_SIDE_BOX_WIDTH),
            (0.0, -TAPE_SIDE_DEFAULT_WIDTH + TAPE_SIDE_BOX_WIDTH),
            (0.0, -TAPE_SIDE_DEFAULT_WIDTH)
        ]
        center_box = [
            (TAPE_SIDE_DEFAULT_WIDTH, -TAPE_SIDE_BOX_WIDTH/2),
            (TAPE_SIDE_DEFAULT_WIDTH, TAPE_SIDE_BOX_WIDTH/2),
            (0.0, TAPE_SIDE_BOX_WIDTH/2),
            (0.0, -TAPE_SIDE_BOX_WIDTH/2)
        ]
        right_box = [
            (TAPE_SIDE_DEFAULT_WIDTH, TAPE_SIDE_DEFAULT_WIDTH - TAPE_SIDE_BOX_WIDTH),
            (TAPE_SIDE_DEFAULT_WIDTH, TAPE_SIDE_DEFAULT_WIDTH),
            (0.0, TAPE_SIDE_DEFAULT_WIDTH),
            (0.0, TAPE_SIDE_DEFAULT_WIDTH - TAPE_SIDE_BOX_WIDTH)
        ]
        edge_start = TAPE_FRONT_DETECTION_DEPTH
        edge_end = edge_start + 0.5
        edge_left_box = [
            (edge_end, -TAPE_SIDE_BOX_WIDTH),
            (edge_end, 0.0),
            (edge_start, 0.0),
            (edge_start, -TAPE_SIDE_BOX_WIDTH)
        ]
        edge_center_box = [
            (edge_end, -TAPE_SIDE_BOX_WIDTH/2),
            (edge_end, TAPE_SIDE_BOX_WIDTH/2),
            (edge_start, TAPE_SIDE_BOX_WIDTH/2),
            (edge_start, -TAPE_SIDE_BOX_WIDTH/2)
        ]
        edge_right_box = [
            (edge_end, 0.0),
            (edge_end, TAPE_SIDE_BOX_WIDTH),
            (edge_start, TAPE_SIDE_BOX_WIDTH),
            (edge_start, 0.0)
        ]
        front_poly = Polygon(front_box)
        left_poly = Polygon(left_box)
        center_poly = Polygon(center_box)
        right_poly = Polygon(right_box)
        edge_left_poly = Polygon(edge_left_box)
        edge_center_poly = Polygon(edge_center_box)
        edge_right_poly = Polygon(edge_right_box)
        front_tapes = []
        left_tapes = []
        center_tapes = []
        right_tapes = []
        edge_left_tapes = []
        edge_center_tapes = []
        edge_right_tapes = []
        def to_robot_frame(x, y):
            dx = x - robot_x
            dy = y - robot_y
            rel_x = dx * cos_h + dy * sin_h
            rel_y = -dx * sin_h + dy * cos_h
            return (rel_x, rel_y)
        tracked_objects = self.visualizer.tracked_objects
        for tape in tracked_objects['tapes']:
            if tape.polygon:
                rel_points = [to_robot_frame(p[0], p[1]) for p in tape.polygon]
                tape_line = LineString(rel_points)
                if front_poly.intersects(tape_line):
                    front_tapes.append(tape)
                if left_poly.intersects(tape_line):
                    left_tapes.append(tape)
                if center_poly.intersects(tape_line):
                    center_tapes.append(tape)
                if right_poly.intersects(tape_line):
                    right_tapes.append(tape)
                if edge_left_poly.intersects(tape_line):
                    edge_left_tapes.append(tape)
                if edge_center_poly.intersects(tape_line):
                    edge_center_tapes.append(tape)
                if edge_right_poly.intersects(tape_line):
                    edge_right_tapes.append(tape)
            else:
                rel = to_robot_frame(tape.center[0], tape.center[1])
                point = Point(rel)
                if front_poly.contains(point):
                    front_tapes.append(tape)
                if left_poly.contains(point):
                    left_tapes.append(tape)
                if center_poly.contains(point):
                    center_tapes.append(tape)
                if right_poly.contains(point):
                    right_tapes.append(tape)
                if edge_left_poly.contains(point):
                    edge_left_tapes.append(tape)
                if edge_center_poly.contains(point):
                    edge_center_tapes.append(tape)
                if edge_right_poly.contains(point):
                    edge_right_tapes.append(tape)
        print(f"[TAPE] Detection results: {len(front_tapes)} front, {len(left_tapes)} left, {len(center_tapes)} center, {len(right_tapes)} right")
        print(f"[TAPE] Edge detection: {len(edge_left_tapes)} edge_left, {len(edge_center_tapes)} edge_center, {len(edge_right_tapes)} edge_right")
        return {
            'front': front_tapes,
            'left': left_tapes,
            'center': center_tapes,
            'right': right_tapes,
            'edge_left': edge_left_tapes,
            'edge_center': edge_center_tapes,
            'edge_right': edge_right_tapes
        }
    def check_tape_in_approach_rectangles(self):
        if not self.visualizer:
            print("Visualizer not available")
            return {'rect1': [], 'rect2': [], 'rect3': []}
        robot_x = self.visualizer.robot_data[self.robot_id]["x"]
        robot_y = self.visualizer.robot_data[self.robot_id]["y"]
        robot_heading = self.visualizer.robot_data[self.robot_id]["heading"]
        print(f"[TAPE_APPROACH] Robot at ({robot_x:.1f}, {robot_y:.1f}, {robot_heading:.1f}°)")
        heading_rad = math.radians(robot_heading)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        rect1_box = [
            (TAPE_APPROACH_RECT_HEIGHT1, -TAPE_APPROACH_RECT1_WIDTH/2),
            (TAPE_APPROACH_RECT_HEIGHT1, TAPE_APPROACH_RECT1_WIDTH/2),
            (0.0, TAPE_APPROACH_RECT1_WIDTH/2),
            (0.0, -TAPE_APPROACH_RECT1_WIDTH/2)
        ]
        rect2_box = [
            (2 * TAPE_APPROACH_RECT_HEIGHT, -TAPE_APPROACH_RECT2_WIDTH/2),
            (2 * TAPE_APPROACH_RECT_HEIGHT, TAPE_APPROACH_RECT2_WIDTH/2),
            (TAPE_APPROACH_RECT_HEIGHT, TAPE_APPROACH_RECT2_WIDTH/2),
            (TAPE_APPROACH_RECT_HEIGHT, -TAPE_APPROACH_RECT2_WIDTH/2)
        ]
        rect3_box = [
            (3 * TAPE_APPROACH_RECT_HEIGHT, -TAPE_APPROACH_RECT3_WIDTH/2),
            (3 * TAPE_APPROACH_RECT_HEIGHT, TAPE_APPROACH_RECT3_WIDTH/2),
            (2 * TAPE_APPROACH_RECT_HEIGHT, TAPE_APPROACH_RECT3_WIDTH/2),
            (2 * TAPE_APPROACH_RECT_HEIGHT, -TAPE_APPROACH_RECT3_WIDTH/2)
        ]
        rect1_poly = Polygon(rect1_box)
        rect2_poly = Polygon(rect2_box)
        rect3_poly = Polygon(rect3_box)
        rect1_tapes = []
        rect2_tapes = []
        rect3_tapes = []
        def to_robot_frame(x, y):
            dx = x - robot_x
            dy = y - robot_y
            rel_x = dx * cos_h + dy * sin_h
            rel_y = -dx * sin_h + dy * cos_h
            return (rel_x, rel_y)
        tracked_objects = self.visualizer.tracked_objects
        for tape in tracked_objects['tapes']:
            if tape.polygon:
                rel_points = [to_robot_frame(p[0], p[1]) for p in tape.polygon]
                tape_line = LineString(rel_points)
                if rect1_poly.intersects(tape_line):
                    rect1_tapes.append(tape)
                    print(f"[TAPE_APPROACH] Tape detected in rect1 (closest)")
                if rect2_poly.intersects(tape_line):
                    rect2_tapes.append(tape)
                    print(f"[TAPE_APPROACH] Tape detected in rect2 (middle)")
                if rect3_poly.intersects(tape_line):
                    rect3_tapes.append(tape)
                    print(f"[TAPE_APPROACH] Tape detected in rect3 (farthest)")
            else:
                rel = to_robot_frame(tape.center[0], tape.center[1])
                point = Point(rel)
                if rect1_poly.contains(point):
                    rect1_tapes.append(tape)
                    print(f"[TAPE_APPROACH] Tape detected in rect1 (closest)")
                if rect2_poly.contains(point):
                    rect2_tapes.append(tape)
                    print(f"[TAPE_APPROACH] Tape detected in rect2 (middle)")
                if rect3_poly.contains(point):
                    rect3_tapes.append(tape)
                    print(f"[TAPE_APPROACH] Tape detected in rect3 (farthest)")
        print(f"[TAPE_APPROACH] Detection results: {len(rect1_tapes)} rect1, {len(rect2_tapes)} rect2, {len(rect3_tapes)} rect3")
        return {
            'rect1': rect1_tapes,
            'rect2': rect2_tapes,
            'rect3': rect3_tapes
        }
    def approach_rectangles_see_tape(self):
        approach_boxes = self.check_tape_in_approach_rectangles()
        has_any = bool(approach_boxes['rect1'] or approach_boxes['rect2'] or approach_boxes['rect3'])
        if has_any:
            print("[TAPE_APPROACH] Approach rectangles detect tape - ready for alignment mode")
        return has_any
    def should_end_tape_following_loop(self):
        return False
    def drive_around_tape_border(self, direction="cw", max_steps=100):
        print(f"[TAPE_BORDER] Starting tape border following - direction: {direction}")
        if direction.lower() == "cw":
            primary_side = "left"
            secondary_side = "right"
            turn_direction_multiplier = 1
        else:
            primary_side = "right"
            secondary_side = "left"
            turn_direction_multiplier = -1
        print(f"[TAPE_BORDER] Following {direction.upper()} - monitoring {primary_side} side")
        print("[TAPE_BORDER] Phase 1: Getting into position...")
        self._get_into_tape_following_position(direction, turn_direction_multiplier)
        print("[TAPE_BORDER] Phase 2: Starting main tape following loop...")
        for step in range(max_steps):
            if self.should_end_tape_following_loop():
                print("[TAPE_BORDER] Loop termination condition met - ending")
                break
            print(f"[TAPE_BORDER] Step {step}: Adding objects")
            self.add_objects()
            tape_boxes = self.check_tape_in_detection_boxes()
            obstacle_boxes = self.check_objects_in_avoidance_boxes()
            if obstacle_boxes['front']:
                print(f"[TAPE_BORDER] Obstacles detected in front - starting avoidance")
                self._perform_avoidance_maneuver(obstacle_boxes, None, primary_side)
                continue
            action = self._determine_tape_following_action(tape_boxes, direction, turn_direction_multiplier)
            if action['type'] == 'move_forward':
                print(f"[TAPE_BORDER] Moving forward {action['distance']}cm")
                self.move_distance(action['distance'])
            elif action['type'] == 'turn':
                print(f"[TAPE_BORDER] Turning {action['angle']}° ({action['reason']})")
                self.turn_degrees(action['angle'])
            elif action['type'] == 'search_front':
                print(f"[TAPE_BORDER] Tape in front - searching for clear path")
                self._handle_tape_in_front(direction, turn_direction_multiplier)
            time.sleep(0.1)
        print(f"[TAPE_BORDER] Tape border following complete after {step} steps")
    def _get_into_tape_following_position(self, direction, turn_direction_multiplier):
        print("[TAPE_BORDER] Getting into tape following position...")
        for attempt in range(20):
            self.add_objects()
            tape_boxes = self.check_tape_in_detection_boxes()
            position_status = self._is_good_tape_following_position(tape_boxes)
            if position_status == True:
                print("[TAPE_BORDER] Reached good tape following position")
                return
            elif position_status == "edge_case":
                print("[TAPE_BORDER] Reached 90° edge case position - handling edge case")
                action = self._determine_tape_following_action(tape_boxes, direction, turn_direction_multiplier)
                if action['type'] == 'turn':
                    print(f"[TAPE_BORDER] Executing edge case turn: {action['angle']}° ({action['reason']})")
                    self.turn_degrees(action['angle'])
                return
            if tape_boxes['front']:
                print("[TAPE_BORDER] Tape in front during positioning - handling")
                self._handle_tape_in_front(direction, turn_direction_multiplier)
            else:
                print("[TAPE_BORDER] Moving forward to find proper tape position")
                self.move_distance(AVOIDANCE_STEP_CM)
        print("[TAPE_BORDER] Warning: Could not reach ideal tape following position")
    def _is_good_tape_following_position(self, tape_boxes):
        has_front = bool(tape_boxes['front'])
        has_left = bool(tape_boxes['left'])
        has_center = bool(tape_boxes['center'])
        has_right = bool(tape_boxes['right'])
        has_edge_left = bool(tape_boxes.get('edge_left', []))
        has_edge_center = bool(tape_boxes.get('edge_center', []))
        has_edge_right = bool(tape_boxes.get('edge_right', []))
        effective_center = has_center or has_edge_center or has_edge_left or has_edge_right
        effective_right = has_right or has_edge_right
        effective_left = has_left or has_edge_left
        edge_count = sum([has_edge_left, has_edge_center, has_edge_right])
        other_detection = has_front or has_left or has_center or has_right
        if has_front:
            return False
        good_patterns = [
            (False, False, True, False),
            (False, False, True, True),
            (False, False, False, True),
            (False, True, True, False),
            (False, True, False, False),
        ]
        current_pattern = (has_front, effective_left, effective_center, effective_right)
        is_good = current_pattern in good_patterns
        print(f"[TAPE_BORDER] Position check - Front:{has_front}, Left:{has_left}, Center:{has_center}, Right:{has_right} -> {'GOOD' if is_good else 'BAD'}")
        print(f"[TAPE_BORDER] Effective detection - Left:{effective_left}, Center:{effective_center}, Right:{effective_right}")
        if edge_count > 0:
            print(f"[TAPE_BORDER] Edge detection - EdgeLeft:{has_edge_left}, EdgeCenter:{has_edge_center}, EdgeRight:{has_edge_right}")
        return is_good
    def _determine_tape_following_action(self, tape_boxes, direction, turn_direction_multiplier):
        has_front = bool(tape_boxes['front'])
        has_left = bool(tape_boxes['left'])
        has_center = bool(tape_boxes['center'])
        has_right = bool(tape_boxes['right'])
        has_edge_left = bool(tape_boxes.get('edge_left', []))
        has_edge_center = bool(tape_boxes.get('edge_center', []))
        has_edge_right = bool(tape_boxes.get('edge_right', []))
        effective_center = has_center or has_edge_center or has_edge_left or has_edge_right
        effective_right = has_right or has_edge_right
        effective_left = has_left or has_edge_left
        print(f"[TAPE_BORDER] Tape pattern - Front:{has_front}, Left:{has_left}, Center:{has_center}, Right:{has_right}")
        if has_edge_left or has_edge_center or has_edge_right:
            print(f"[TAPE_BORDER] Edge detection - EdgeLeft:{has_edge_left}, EdgeCenter:{has_edge_center}, EdgeRight:{has_edge_right}")
            print(f"[TAPE_BORDER] Effective detection - Left:{effective_left}, Center:{effective_center}, Right:{effective_right}")
        edge_count = sum([has_edge_left, has_edge_center, has_edge_right])
        other_detection = has_front or has_left or has_center or has_right
        if edge_count == 1 and not other_detection:
            print("[TAPE_BORDER] 90° edge case detected - turning 90 degrees")
            if has_edge_left:
                angle = -90.0
                reason = "90deg_edge_left"
            elif has_edge_right:
                angle = 90.0
                reason = "90deg_edge_right"
            else:
                angle = 90.0 * turn_direction_multiplier
                reason = "90deg_edge_center"
            return {'type': 'turn', 'angle': angle, 'reason': reason}
        if has_front and (effective_left or effective_right):
            side_detected = "left" if effective_left else "right"
            print(f"[TAPE_BORDER] Both front and {side_detected} boxes detect tape - turning CW until only side detection")
            print(f"[TAPE_BORDER] Direction multiplier: {turn_direction_multiplier} ({'CW' if turn_direction_multiplier > 0 else 'CCW'})")
            angle = 15.0 * turn_direction_multiplier
            return {'type': 'turn', 'angle': angle, 'reason': 'front_and_side_detected'}
        if has_front:
            return {'type': 'search_front', 'reason': 'tape_in_front'}
        if effective_center and not effective_left and not effective_right:
            reason = 'only_center'
            if has_edge_center and not has_center:
                reason = 'only_edge_center'
            return {'type': 'move_forward', 'distance': AVOIDANCE_STEP_CM, 'reason': reason}
        elif effective_right and not effective_left and not effective_center:
            angle = TAPE_TURN_ANGLE_FULL * turn_direction_multiplier
            reason = 'only_right'
            if has_edge_right and not has_right:
                reason = 'only_edge_right'
            return {'type': 'move_forward_then_turn', 'distance': AVOIDANCE_STEP_CM, 'angle': angle, 'reason': reason}
        elif effective_left and not effective_right and not effective_center:
            angle = -TAPE_TURN_ANGLE_FULL * turn_direction_multiplier
            reason = 'only_left'
            if has_edge_left and not has_left:
                reason = 'only_edge_left'
            return {'type': 'move_forward_then_turn', 'distance': AVOIDANCE_STEP_CM, 'angle': angle, 'reason': reason}
        elif effective_center and effective_right and not effective_left:
            angle = TAPE_TURN_ANGLE_HALF * turn_direction_multiplier
            reason = 'center_and_right'
            if has_edge_right:
                reason = 'center_and_edge_right' if has_center else 'edge_right_as_center_right'
            return {'type': 'move_forward_then_turn', 'distance': AVOIDANCE_STEP_CM, 'angle': angle, 'reason': reason}
        elif effective_center and effective_left and not effective_right:
            angle = -TAPE_TURN_ANGLE_HALF * turn_direction_multiplier
            reason = 'center_and_left'
            if has_edge_left:
                reason = 'center_and_edge_left' if has_center else 'edge_left_as_center_left'
            return {'type': 'move_forward_then_turn', 'distance': AVOIDANCE_STEP_CM, 'angle': angle, 'reason': reason}
        else:
            print(f"[TAPE_BORDER] Warning: Unexpected tape pattern, going straight")
            return {'type': 'move_forward', 'distance': AVOIDANCE_STEP_CM, 'reason': 'unexpected_pattern'}
    def _handle_tape_in_front(self, direction, turn_direction_multiplier):
        print("[TAPE_BORDER] Handling tape in front")
        max_search_turns = 20
        initial_heading = self.visualizer.robot_data[self.robot_id]["heading"]
        for turn_attempt in range(max_search_turns):
            turn_angle = TAPE_SEARCH_TURN_ANGLE * turn_direction_multiplier
            print(f"[TAPE_BORDER] Search turn {turn_attempt + 1}: turning {turn_angle}°")
            self.turn_degrees(turn_angle)
            self.add_objects()
            tape_boxes = self.check_tape_in_detection_boxes()
            if not tape_boxes['front']:
                print("[TAPE_BORDER] Front clear after search turn")
                position_status = self._is_good_tape_following_position(tape_boxes)
                if position_status == True:
                    print("[TAPE_BORDER] Found good position after search turn")
                    return
                elif position_status == "edge_case":
                    print("[TAPE_BORDER] Found edge case position after search turn")
                    return
                else:
                    print("[TAPE_BORDER] No side detection - trying to turn back")
                    break
        if not tape_boxes['front']:
            for turn_back_attempt in range(max_search_turns):
                turn_angle = -TAPE_SEARCH_TURN_ANGLE * turn_direction_multiplier
                print(f"[TAPE_BORDER] Turn back {turn_back_attempt + 1}: turning {turn_angle}°")
                self.turn_degrees(turn_angle)
                self.add_objects()
                tape_boxes = self.check_tape_in_detection_boxes()
                position_status = self._is_good_tape_following_position(tape_boxes)
                if tape_boxes['front'] or position_status == True or position_status == "edge_case":
                    print("[TAPE_BORDER] Found tape after turning back")
                    return
        print("[TAPE_BORDER] Warning: Could not resolve tape in front situation")
    def straight_line_avoidance_mode(self, max_cycles=10, straight_distance=200.0):
        print(f"[STRAIGHT_MODE] Starting straight-line obstacle avoidance mode")
        print(f"[STRAIGHT_MODE] Will run up to {max_cycles} cycles of straight->tape->turn 120°")
        if not self.stuck_detection_enabled:
            self.enable_stuck_detection(True, radius=20.0, time_threshold=60.0)
        for cycle in range(max_cycles):
            self._wait_for_other_robot_to_clear()
            print(f"\n[STRAIGHT_MODE] === Cycle {cycle + 1}/{max_cycles} ===")
            print("[STRAIGHT_MODE] Phase 1: Going straight with obstacle avoidance until tape detected")
            tape_hit = self._go_straight_until_tape(straight_distance)
            if not tape_hit:
                print("[STRAIGHT_MODE] Warning: No tape detected in this cycle, continuing anyway")
            print("[STRAIGHT_MODE] Phase 2: Turning 120 degrees")
            self.turn_degrees(120.0)
            print("[STRAIGHT_MODE] Adding objects after turn")
            self.add_objects()
            time.sleep(0.5)
        print(f"[STRAIGHT_MODE] Straight-line obstacle avoidance mode complete after {max_cycles} cycles")
    def _go_straight_until_tape(self, max_distance=200.0, step_size=AVOIDANCE_STEP_CM):
        print(f"[STRAIGHT_MODE] Going straight up to {max_distance}cm until tape detected")
        initial_robot_x = self.visualizer.robot_data[self.robot_id]["x"]
        initial_robot_y = self.visualizer.robot_data[self.robot_id]["y"]
        initial_heading = self.visualizer.robot_data[self.robot_id]["heading"]
        heading_rad = math.radians(initial_heading)
        target_x = initial_robot_x + max_distance * math.cos(heading_rad)
        target_y = initial_robot_y + max_distance * math.sin(heading_rad)
        print(f"[STRAIGHT_MODE] Initial position: ({initial_robot_x:.1f}, {initial_robot_y:.1f})")
        print(f"[STRAIGHT_MODE] Initial heading: {initial_heading:.1f}°")
        print(f"[STRAIGHT_MODE] Target point: ({target_x:.1f}, {target_y:.1f})")
        distance_traveled = 0.0
        max_steps = int(max_distance / step_size) + 10
        for step in range(max_steps):
            self.update_position_history()
            if self.is_robot_stuck():
                print("[STRAIGHT_MODE] Robot is stuck! Performing escape maneuver")
                self.perform_stuck_escape_maneuver()
                current_robot_x = self.visualizer.robot_data[self.robot_id]["x"]
                current_robot_y = self.visualizer.robot_data[self.robot_id]["y"]
                current_heading = self.visualizer.robot_data[self.robot_id]["heading"]
                heading_rad = math.radians(current_heading)
                target_x = current_robot_x + max_distance * math.cos(heading_rad)
                target_y = current_robot_y + max_distance * math.sin(heading_rad)
                print(f"[STRAIGHT_MODE] New target after escape: ({target_x:.1f}, {target_y:.1f})")
                continue
            if self.tape_in_view():
                print(f"[STRAIGHT_MODE] Tape detected after {distance_traveled:.1f}cm!")
                return True
            current_robot_x = self.visualizer.robot_data[self.robot_id]["x"]
            current_robot_y = self.visualizer.robot_data[self.robot_id]["y"]
            distance_from_start = math.hypot(
                current_robot_x - initial_robot_x,
                current_robot_y - initial_robot_y
            )
            if distance_from_start >= max_distance:
                print(f"[STRAIGHT_MODE] Reached max distance {max_distance}cm without finding tape")
                return False
            print(f"[STRAIGHT_MODE] Step {step}: Adding objects before movement")
            self.add_objects()
            obstacle_boxes = self.check_objects_in_avoidance_boxes()
            if obstacle_boxes['front']:
                print(f"[STRAIGHT_MODE] Obstacles detected in front, performing avoidance")
                self._perform_avoidance_maneuver(obstacle_boxes, None, "center")
                current_robot_x = self.visualizer.robot_data[self.robot_id]["x"]
                current_robot_y = self.visualizer.robot_data[self.robot_id]["y"]
                current_heading = self.visualizer.robot_data[self.robot_id]["heading"]
                heading_rad = math.radians(current_heading)
                remaining_distance = max_distance - distance_traveled
                target_x = current_robot_x + remaining_distance * math.cos(heading_rad)
                target_y = current_robot_y + remaining_distance * math.sin(heading_rad)
                print(f"[STRAIGHT_MODE] Updated target after avoidance: ({target_x:.1f}, {target_y:.1f})")
            else:
                print(f"[STRAIGHT_MODE] Clear path, moving {step_size}cm straight")
                self.move_distance(step_size)
                distance_traveled += step_size
                self.avoidance_movement_count += 1
                self.add_objects()
            time.sleep(0.1)
        print(f"[STRAIGHT_MODE] Reached max steps without finding tape")
        return False
    def start_straight_line_mode(self, max_cycles=10, straight_distance=200.0):
        print("Starting straight-line obstacle avoidance mode directly...")
        self.enable_stuck_detection(True, radius=20.0, time_threshold=60.0)
        print("Initial object detection...")
        self.add_objects()
        self.straight_line_avoidance_mode(max_cycles, straight_distance)
    def update_position_history(self):
        if not self.stuck_detection_enabled or not self.visualizer:
            return
        current_time = time.time()
        robot_x = self.visualizer.robot_data[self.robot_id]["x"]
        robot_y = self.visualizer.robot_data[self.robot_id]["y"]
        self.stuck_positions.append((robot_x, robot_y, current_time))
        cutoff_time = current_time - (self.stuck_time_threshold + 10.0)
        self.stuck_positions = [(x, y, t) for x, y, t in self.stuck_positions if t > cutoff_time]
        if len(self.stuck_positions) > 200:
            self.stuck_positions = self.stuck_positions[-100:]
    def is_robot_stuck(self):
        if not self.stuck_detection_enabled or not self.visualizer:
            return False
        current_time = time.time()
        if current_time - self.last_stuck_check < 5.0:
            return False
        self.last_stuck_check = current_time
        if len(self.stuck_positions) < 10:
            return False
        robot_x = self.visualizer.robot_data[self.robot_id]["x"]
        robot_y = self.visualizer.robot_data[self.robot_id]["y"]
        stuck_start_time = None
        for pos_x, pos_y, timestamp in self.stuck_positions:
            distance = math.hypot(robot_x - pos_x, robot_y - pos_y)
            if distance <= self.stuck_check_radius:
                if stuck_start_time is None:
                    stuck_start_time = timestamp
                if current_time - stuck_start_time >= self.stuck_time_threshold:
                    print(f"[STUCK_DETECTION] Robot stuck! Been within {self.stuck_check_radius}cm radius for {current_time - stuck_start_time:.1f}s")
                    return True
            else:
                stuck_start_time = None
        return False
    def perform_stuck_escape_maneuver(self):
        print("[STUCK_ESCAPE] Performing stuck escape maneuver")
        print("[STUCK_ESCAPE] Turning 180 degrees to escape stuck position")
        self.turn_degrees(180.0)
        print("[STUCK_ESCAPE] Adding objects after escape turn")
        self.add_objects()
        self.stuck_positions.clear()
        print("[STUCK_ESCAPE] Cleared position history")
        print("[STUCK_ESCAPE] Moving forward to get away from stuck area")
        self.move_distance(AVOIDANCE_STEP_CM * 2)
        self.add_objects()
        print("[STUCK_ESCAPE] Stuck escape maneuver complete")
    def enable_stuck_detection(self, enable=True, radius=20.0, time_threshold=60.0):
        self.stuck_detection_enabled = enable
        self.stuck_check_radius = radius
        self.stuck_time_threshold = time_threshold
        if enable:
            print(f"[STUCK_DETECTION] Enabled - radius: {radius}cm, time: {time_threshold}s")
            self.stuck_positions.clear()
        else:
            print("[STUCK_DETECTION] Disabled")
    def reset_avoidance_memory(self):
        print("[AVOIDANCE] Resetting avoidance direction memory")
        self.last_avoidance_direction = None
        self.avoidance_direction_preference = None
    def set_avoidance_preference(self, direction):
        if direction in ['right', 'left']:
            self.avoidance_direction_preference = direction
            print(f"[AVOIDANCE] Set avoidance preference to: {direction}")
        else:
            print(f"[AVOIDANCE] Invalid direction: {direction}. Use 'right' or 'left'")
    def _wait_for_other_robot_to_clear(self, min_dist=35.0, check_dist=20.0, check_interval=0.2):
        robot_id = self.robot_id
        if robot_id != "robot2" or not self.visualizer:
            return
        robot2_x = self.visualizer.robot_data["robot2"]["x"]
        robot2_y = self.visualizer.robot_data["robot2"]["y"]
        robot1_x = self.visualizer.robot_data["robot1"]["x"]
        robot1_y = self.visualizer.robot_data["robot1"]["y"]
        dist = math.hypot(robot2_x - robot1_x, robot2_y - robot1_y)
        if dist < check_dist:
            print(f"[ROBOT2] Robot1 detected within {check_dist}cm ({dist:.1f}cm). Pausing until at least {min_dist}cm away...")
            while True:
                robot2_x = self.visualizer.robot_data["robot2"]["x"]
                robot2_y = self.visualizer.robot_data["robot2"]["y"]
                robot1_x = self.visualizer.robot_data["robot1"]["x"]
                robot1_y = self.visualizer.robot_data["robot1"]["y"]
                dist = math.hypot(robot2_x - robot1_x, robot2_y - robot1_y)
                if dist >= min_dist:
                    print(f"[ROBOT2] Robot1 is now {dist:.1f}cm away. Resuming movement.")
                    break
                time.sleep(check_interval)