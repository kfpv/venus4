import threading
import time
import math
import sys
import os
import signal
import select
import termios
import tty
import json
from datetime import datetime
from shapely.geometry import LineString
import polyline
from adb import DetectionProcessor
from robot_control import RobotController, RELEASE_THRESHOLD
from detection_classes import Box, Robot
USE_ESP_MQTT = True
if USE_ESP_MQTT:
    from simple_mqtt_bridge import MQTTClient as BridgeMQTTClient
    mqtt = BridgeMQTTClient
    MQTT_AVAILABLE = True
else:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
ROBOT_CONFIG_FILE = "robots_config.json"
try:
    from robot_id_config import ROBOT_ID
    print(f"Loaded ROBOT_ID from config: {ROBOT_ID}")
except ImportError:
    ROBOT_ID = ""
    print("Warning: robot_id_config.py not found, using single robot mode")
robot_ctl = None
detection_proc = None
mqtt_client = None
mqtt_publish_thread = None
mqtt_publish_active = False
robot_config = None
CUBE_DATA_DELAY_AFTER_MOVEMENT = 1.0
last_movement_time = 0.0
last_published_pose = {'x': None, 'y': None, 'heading': None}
def transform_point_to_world(robot_x, robot_y, robot_heading_deg, obj_camera_x, obj_camera_y):
    heading_rad = math.radians(robot_heading_deg)
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)
    world_x = robot_x + (obj_camera_y * cos_h - obj_camera_x * sin_h)
    world_y = robot_y + (obj_camera_y * sin_h + obj_camera_x * cos_h)
    return world_x, world_y
def get_camera_bounds(robot_x, robot_y, robot_heading, detection_proc):
    if not detection_proc or not detection_proc.is_ready():
        print("Detection processor not ready for camera bounds")
        return None
    camera_view = detection_proc.get_camera_view_polygon()
    if camera_view is None:
        print("No camera view polygon available")
        return None
    world_points = []
    for point in camera_view:
        if len(point) >= 2:
            world_x, world_y = transform_point_to_world(robot_x, robot_y, robot_heading, point[0], point[1])
            world_points.append((world_x, world_y))
    return world_points if len(world_points) >= 3 else None
def setup_mqtt(broker="100.69.37.102", port=1883):
    global mqtt_client
    if not MQTT_AVAILABLE:
        print("MQTT not available, skipping MQTT setup")
        return False
    try:
        mqtt_client = mqtt()
        mqtt_client.on_message = on_mqtt_message
        mqtt_client.connect(broker, port, 60)
        mqtt_client.loop_start()
        topics = robot_config["mqtt_topics"]
        mqtt_client.subscribe(topics["navigation"])
        mqtt_client.subscribe(topics["manual_control"])
        mqtt_client.subscribe(topics["imprint_command"])
        print(f"Connected to MQTT broker at {broker}:{port}")
        print(f"Subscribed to topics for robot: {robot_config['robot_id']}")
        return True
    except Exception as e:
        print(f"Failed to connect to MQTT broker: {e}")
        mqtt_client = None
        return False
def on_mqtt_message(client, userdata, msg):
    global robot_ctl
    try:
        print(f"DEBUG: Received MQTT message on topic: {msg.topic}")
        print(f"DEBUG: Message payload: {msg.payload.decode()}")
        topics = robot_config["mqtt_topics"]
        print(f"DEBUG: Expected topics - navigation: {topics['navigation']}, manual_control: {topics['manual_control']}")
        if msg.topic == topics["navigation"]:
            payload = json.loads(msg.payload.decode())
            if payload.get('command') == 'navigate_to':
                target_x = payload.get('target_x')
                target_y = payload.get('target_y')
                if target_x is not None and target_y is not None and robot_ctl:
                    print(f"\nReceived navigation command: go to ({target_x:.1f}, {target_y:.1f}) cm")
                    nav_thread = threading.Thread(
                        target=navigate_to_position,
                        args=(target_x, target_y),
                        daemon=True
                    )
                    nav_thread.start()
                else:
                    print("Invalid navigation command or robot not available")
        elif msg.topic == topics["manual_control"]:
            payload = json.loads(msg.payload.decode())
            command = payload.get('command')
            if command == 'turn':
                angle = payload.get('angle_degrees')
                if angle is not None and robot_ctl:
                    print(f"\nReceived turn command: {angle}°")
                    turn_thread = threading.Thread(
                        target=execute_turn_command,
                        args=(angle,),
                        daemon=True
                    )
                    turn_thread.start()
            elif command == 'turn_to_heading':
                target_heading = payload.get('target_heading_degrees')
                if target_heading is not None and robot_ctl:
                    print(f"\nReceived turn to heading command: {target_heading}°")
                    turn_to_heading_thread = threading.Thread(
                        target=execute_turn_to_heading_command,
                        args=(target_heading,),
                        daemon=True
                    )
                    turn_to_heading_thread.start()
            elif command == 'move':
                distance = payload.get('distance_cm')
                if distance is not None and robot_ctl:
                    print(f"\nReceived move command: {distance}cm")
                    move_thread = threading.Thread(
                        target=execute_move_command,
                        args=(distance,),
                        daemon=True
                    )
                    move_thread.start()
            elif command == 'stop':
                print("\nReceived stop command")
                if robot_ctl:
                    robot_ctl.stop(force_reset=True)
            elif command == 'go_home':
                print("\nReceived go home command")
                if robot_ctl:
                    home_thread = threading.Thread(
                        target=robot_ctl.go_to_origin,
                        daemon=True
                    )
                    home_thread.start()
            elif command == 'test_left_wheel_rotation':
                print("\nReceived left wheel rotation test command")
                if robot_ctl:
                    test_thread = threading.Thread(
                        target=robot_ctl.test_single_wheel_rotation_left,
                        daemon=True
                    )
                    test_thread.start()
            elif command == 'test_right_wheel_rotation':
                print("\nReceived right wheel rotation test command")
                if robot_ctl:
                    test_thread = threading.Thread(
                        target=robot_ctl.test_single_wheel_rotation_right,
                        daemon=True
                    )
                    test_thread.start()
            elif command == 'update_correction_factors':
                turn_ccw_factor = payload.get('turn_ccw_correction_factor')
                turn_cw_factor = payload.get('turn_cw_correction_factor')
                distance_factor = payload.get('distance_correction_factor')
                left_understep_percent = payload.get('left_understep_percent')
                if ((turn_ccw_factor is not None or turn_cw_factor is not None) and
                    distance_factor is not None and robot_ctl):
                    print(f"\nReceived correction factor update:")
                    if turn_ccw_factor is not None:
                        print(f"  Turn CCW={turn_ccw_factor:.4f}")
                    if turn_cw_factor is not None:
                        print(f"  Turn CW={turn_cw_factor:.4f}")
                    print(f"  Distance={distance_factor:.4f}")
                    if left_understep_percent is not None:
                        print(f"  Left wheel understepping={left_understep_percent:.1f}%")
                    update_thread = threading.Thread(
                        target=execute_correction_factor_update,
                        args=(turn_ccw_factor, turn_cw_factor, distance_factor, left_understep_percent),
                        daemon=True
                    )
                    update_thread.start()
            elif command == 'reset_to_origin':
                print("\nReceived reset to origin command")
                if robot_ctl:
                    reset_thread = threading.Thread(
                        target=execute_reset_to_origin,
                        daemon=True
                    )
                    reset_thread.start()
            elif command == 'set_speed_multiplier':
                speed_multiplier = payload.get('speed_multiplier')
                if speed_multiplier is not None and robot_ctl:
                    print(f"\nReceived speed multiplier update: {speed_multiplier:.2f}")
                    speed_thread = threading.Thread(
                        target=execute_speed_multiplier_update,
                        args=(speed_multiplier,),
                        daemon=True
                    )
                    speed_thread.start()
            elif command == 'update_physical_parameters':
                wheel_radius = payload.get('wheel_radius_cm')
                wheel_base = payload.get('wheel_base_cm')
                if wheel_radius is not None and wheel_base is not None and robot_ctl:
                    print(f"\nReceived physical parameters update: Radius={wheel_radius:.2f}cm, Base={wheel_base:.2f}cm")
                    physical_thread = threading.Thread(
                        target=execute_physical_parameters_update,
                        args=(wheel_radius, wheel_base),
                        daemon=True
                    )
                    physical_thread.start()
            elif command == 'imprint_scene':
                print("\nReceived imprint scene command from map visualizer")
                if detection_proc:
                    imprint_thread = threading.Thread(
                        target=perform_imprinting_from_mqtt,
                        daemon=True
                    )
                    imprint_thread.start()
                else:
                    print("Detection processor not available for imprintting")
            elif command == 'autonav2':
                print("\nReceived autonav2 command: starting autonomous_independant SIMPLE DEMO mode")
                try:
                    import autonomous_independant
                    auto_thread = threading.Thread(
                        target=autonomous_independant.run_simple_demo_mode,
                        args=(robot_ctl, detection_proc, mqtt_client, robot_config),
                        daemon=True
                    )
                    auto_thread.start()
                except Exception as e:
                    print(f"Failed to start autonomous_independant simple demo mode: {e}")
            elif command == 'autonav3':
                print("\nReceived autonav3 command: starting autonomous_independant FULL RUN mode")
                try:
                    import autonomous_independant
                    auto_thread = threading.Thread(
                        target=autonomous_independant.run_autonomous_mode,
                        args=(robot_ctl, detection_proc, mqtt_client, robot_config),
                        daemon=True
                    )
                    auto_thread.start()
                except Exception as e:
                    print(f"Failed to start autonomous_independant FULL RUN mode: {e}")
        elif msg.topic == topics["imprint_command"]:
            payload = json.loads(msg.payload.decode())
            if payload.get('command') == 'imprint_scene':
                print("\nReceived imprint scene command")
                if detection_proc:
                    imprint_thread = threading.Thread(
                        target=perform_imprinting_from_mqtt,
                        daemon=True
                    )
                    imprint_thread.start()
                else:
                    print("Detection processor not available for imprinting")
    except Exception as e:
        print(f"Error processing MQTT message: {e}")
def execute_turn_command(angle_degrees):
    global robot_ctl, last_movement_time, mqtt_client
    try:
        if not robot_ctl:
            print("Robot controller not available")
            return
        if robot_ctl.moving:
            robot_ctl.stop(force_reset=True)
            time.sleep(0.5)
        print(f"Turning by {angle_degrees:.1f}° ({'CCW' if angle_degrees < 0 else 'CW'})")
        if mqtt_client:
            start_data = {
                'command': 'navigation_start',
                'movement_type': 'turn',
                'angle': angle_degrees,
            }
            topics = robot_config["mqtt_topics"]
            mqtt_client.publish(f"{topics['navigation_status']}", json.dumps(start_data, separators=(',', ':')))
        robot_ctl.turn_angle(angle_degrees)
        last_movement_time = time.time()
        while robot_ctl.moving:
            time.sleep(0.1)
        print(f"Turn complete. Final heading: {robot_ctl.heading:.1f}°")
        if mqtt_client:
            complete_data = {
                'command': 'navigation_complete',
                'movement_type': 'turn',
                'final_heading': robot_ctl.heading,
            }
            topics = robot_config["mqtt_topics"]
            mqtt_client.publish(f"{topics['navigation_status']}", json.dumps(complete_data, separators=(',', ':')))
    except Exception as e:
        print(f"Error during turn command: {e}")
        if robot_ctl and robot_ctl.moving:
            robot_ctl.stop(force_reset=True)
        if mqtt_client:
            error_data = {
                'command': 'navigation_error',
                'movement_type': 'turn',
                'error_message': str(e),
            }
            topics = robot_config["mqtt_topics"]
            mqtt_client.publish(f"{topics['navigation_status']}", json.dumps(error_data, separators=(',', ':')))
def execute_turn_to_heading_command(target_heading_degrees):
    global robot_ctl, last_movement_time
    try:
        if not robot_ctl:
            print("Robot controller not available")
            return
        if robot_ctl.moving:
            robot_ctl.stop(force_reset=True)
            time.sleep(0.5)
        target_heading = target_heading_degrees % 360
        current_heading = robot_ctl.heading
        turn_angle = target_heading - current_heading
        if turn_angle > 180:
            turn_angle -= 360
        elif turn_angle < -180:
            turn_angle += 360
        print(f"Current heading: {current_heading:.1f}°")
        print(f"Target heading: {target_heading:.1f}°")
        print(f"Turn angle needed: {turn_angle:.1f}° ({'CCW' if turn_angle < 0 else 'CW'})")
        robot_ctl._turn_to_heading(target_heading)
        last_movement_time = time.time()
        print(f"Turn to heading complete. Final heading: {robot_ctl.heading:.1f}°")
    except Exception as e:
        print(f"Error during turn to heading command: {e}")
        if robot_ctl and robot_ctl.moving:
            robot_ctl.stop(force_reset=True)
def execute_move_command(distance_cm):
    global robot_ctl, last_movement_time, mqtt_client
    try:
        if not robot_ctl:
            print("Robot controller not available")
            return
        if robot_ctl.moving:
            robot_ctl.stop(force_reset=True)
            time.sleep(0.5)
        print(f"Moving {distance_cm:.1f}cm ({'Backward' if distance_cm < 0 else 'Forward'})")
        if mqtt_client:
            start_data = {
                'command': 'navigation_start',
                'movement_type': 'move',
                'distance': distance_cm,
            }
            topics = robot_config["mqtt_topics"]
            mqtt_client.publish(f"{topics['navigation_status']}", json.dumps(start_data, separators=(',', ':')))
        robot_ctl.move_distance(distance_cm)
        last_movement_time = time.time()
        while robot_ctl.moving:
            time.sleep(0.1)
        final_x, final_y = robot_ctl.x, robot_ctl.y
        print(f"Move complete. Final position: ({final_x:.1f}, {final_y:.1f}) cm")
        if mqtt_client:
            complete_data = {
                'command': 'navigation_complete',
                'movement_type': 'move',
                'final_x': final_x,
                'final_y': final_y,
            }
            topics = robot_config["mqtt_topics"]
            mqtt_client.publish(f"{topics['navigation_status']}", json.dumps(complete_data, separators=(',', ':')))
    except Exception as e:
        print(f"Error during move command: {e}")
        if robot_ctl and robot_ctl.moving:
            robot_ctl.stop(force_reset=True)
        if mqtt_client:
            error_data = {
                'command': 'navigation_error',
                'movement_type': 'move',
                'error_message': str(e),
            }
            topics = robot_config["mqtt_topics"]
            mqtt_client.publish(f"{topics['navigation_status']}", json.dumps(error_data, separators=(',', ':')))
def navigate_to_position(target_x, target_y):
    global robot_ctl, last_movement_time, mqtt_client
    try:
        if not robot_ctl:
            print("Robot controller not available")
            return
        if robot_ctl.moving:
            robot_ctl.stop(force_reset=True)
            time.sleep(0.5)
        current_x, current_y = robot_ctl.x, robot_ctl.y
        current_heading = robot_ctl.heading
        dx = target_x - current_x
        dy = target_y - current_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        if distance_to_target < 0.5:
            print(f"Already at target position ({target_x:.1f}, {target_y:.1f})")
            if mqtt_client:
                arrival_data = {
                    'command': 'navigation_complete',
                    'target_x': target_x,
                    'target_y': target_y,
                    'final_x': current_x,
                    'final_y': current_y,
                    'final_heading': current_heading,
                    'distance_error': distance_to_target,
                }
                topics = robot_config["mqtt_topics"]
                mqtt_client.publish(f"{topics['navigation_status']}", json.dumps(arrival_data, separators=(',', ':')))
            return
        angle_to_target = math.degrees(math.atan2(dy, dx)) % 360
        print(f"Navigating from ({current_x:.1f}, {current_y:.1f}) to ({target_x:.1f}, {target_y:.1f})")
        print(f"Distance: {distance_to_target:.1f}cm, Target angle: {angle_to_target:.1f}°")
        turn_angle = angle_to_target - current_heading
        if turn_angle > 180:
            turn_angle -= 360
        elif turn_angle < -180:
            turn_angle += 360
        if abs(turn_angle) > 1:
            print(f"Turning {turn_angle:.1f}° to face target")
            robot_ctl.turn_angle(turn_angle)
            while robot_ctl.moving:
                time.sleep(0.1)
        print(f"Moving {distance_to_target:.1f}cm to target")
        robot_ctl.move_distance(distance_to_target)
        last_movement_time = time.time()
        while robot_ctl.moving:
            time.sleep(0.1)
        final_x, final_y = robot_ctl.x, robot_ctl.y
        final_heading = robot_ctl.heading
        final_dx = target_x - final_x
        final_dy = target_y - final_y
        final_distance_error = math.sqrt(final_dx**2 + final_dy**2)
        print(f"Navigation complete. Final position: ({final_x:.1f}, {final_y:.1f}) cm")
        print(f"Distance error: {final_distance_error:.1f} cm")
        if mqtt_client:
            arrival_data = {
                'command': 'navigation_complete',
                'target_x': target_x,
                'target_y': target_y,
                'final_x': final_x,
                'final_y': final_y,
                'final_heading': final_heading,
                'distance_error': final_distance_error,
            }
            topics = robot_config["mqtt_topics"]
            mqtt_client.publish(f"{topics['navigation_status']}", json.dumps(arrival_data, separators=(',', ':')))
            print(f"Sent arrival notification: target=({target_x:.1f}, {target_y:.1f}), final=({final_x:.1f}, {final_y:.1f}), error={final_distance_error:.1f}cm")
    except Exception as e:
        print(f"Error during navigation: {e}")
        if robot_ctl and robot_ctl.moving:
            robot_ctl.stop(force_reset=True)
        if mqtt_client:
            error_data = {
                'command': 'navigation_error',
                'target_x': target_x,
                'target_y': target_y,
                'error_message': str(e),
            }
            topics = robot_config["mqtt_topics"]
            mqtt_client.publish(f"{topics['navigation_status']}", json.dumps(error_data, separators=(',', ':')))
def publish_robot_data(force=False):
    global robot_ctl, detection_proc, mqtt_client, last_movement_time, last_published_pose
    if not mqtt_client or not robot_ctl or not detection_proc:
        return
    try:
        robot = Robot(
            x=robot_ctl.x,
            y=robot_ctl.y,
            heading=robot_ctl.heading,
            moving=robot_ctl.moving,
            current_direction=robot_ctl.current_direction
        )
        pose_changed = False
        threshold = 1e-2
        if last_published_pose['x'] is None or force:
            pose_changed = True
        else:
            if (abs(robot_ctl.x - last_published_pose['x']) > threshold or
                abs(robot_ctl.y - last_published_pose['y']) > threshold or
                abs(robot_ctl.heading - last_published_pose['heading']) > threshold):
                pose_changed = True
        if not pose_changed:
            return
        last_published_pose['x'] = robot_ctl.x
        last_published_pose['y'] = robot_ctl.y
        last_published_pose['heading'] = robot_ctl.heading
        camera_bounds = get_camera_bounds(robot.x, robot.y, robot.heading, detection_proc)
        pose_data = robot.to_dict()
        pose_data['robot_id'] = robot_config['robot_id']
        if 'heading' in pose_data:
            pose_data['h'] = pose_data.pop('heading')
        topics = robot_config["mqtt_topics"]
        mqtt_client.publish(topics["pose"], json.dumps(pose_data, separators=(',', ':')))
        if not USE_ESP_MQTT:
            detections = detection_proc.get_latest_detections()
            if detections and detection_proc.is_data_fresh(max_age_seconds=2.0):
                current_time = time.time()
                send_cube_data = (current_time - last_movement_time) >= CUBE_DATA_DELAY_AFTER_MOVEMENT
                cubes_data = []
                if send_cube_data:
                    cubes_camera = detections.get('cubes_realworld_cm', [])
                    for cube_cam in cubes_camera:
                        try:
                            box = Box.from_detection_data(cube_cam, robot.x, robot.y, robot.heading)
                            cube_dict = box.to_dict()
                            if 'world_polygon' in cube_dict and cube_dict['world_polygon']:
                                poly = encode_polygon_polyline(cube_dict['world_polygon'])
                                if poly:
                                    cube_dict['polyline'] = poly
                                cube_dict.pop('world_polygon', None)
                            cubes_data.append(cube_dict)
                        except Exception as e:
                            print(f"Error processing cube data: {e}")
                            continue
                tapes_data = []
                tapes_camera = detections.get('tape_polygons_realworld_cm', [])
                for tape_cam in tapes_camera:
                    try:
                        if 'points_realworld_cm' in tape_cam:
                            world_points = []
                            for point in tape_cam['points_realworld_cm']:
                                if len(point) >= 2:
                                    world_x, world_y = transform_point_to_world(
                                        robot.x, robot.y, robot.heading, point[0], point[1]
                                    )
                                    world_points.append([world_x, world_y])
                            world_points = simplify_and_round_polygon(world_points, max_points=16)
                            tape_data = {
                                'polyline': encode_polygon_polyline(world_points),
                                'points_count': len(tape_cam['points_realworld_cm']),
                                'area_sq_cm': tape_cam.get('area_sq_cm', 0),
                                'color': tape_cam.get('color', 'tape')
                            }
                            tapes_data.append(tape_data)
                    except Exception as e:
                        print(f"Error processing tape data: {e}")
                        continue
                if cubes_data or tapes_data:
                    detection_data = {
                        'robot_id': robot_config['robot_id'],
                        'robot_pose': pose_data,
                        'cubes': cubes_data,
                        'tapes': tapes_data,
                    }
                    mqtt_client.publish(topics["detections"], json.dumps(detection_data, separators=(',', ':')))
    except Exception as e:
        print(f"Error in publish_robot_data: {e}")
def display_detection_info():
    global robot_ctl, detection_proc
    if not robot_ctl or not detection_proc:
        sys.stdout.write("Components not initialized.\r\n")
        sys.stdout.flush()
        return
    rx, ry, r_heading = robot_ctl.x, robot_ctl.y, robot_ctl.heading
    timestamp_str = datetime.now().strftime("%H:%M:%S")
    output_buffer = []
    output_buffer.append(f"\r\n--- [{timestamp_str}] Robot and Detection Info ---")
    output_buffer.append(f"Robot Pose: X={rx:.2f}cm, Y={ry:.2f}cm, Heading={r_heading:.2f}°")
    detections = detection_proc.get_latest_detections()
    if detections and detection_proc.is_data_fresh(max_age_seconds=2.0):
        cubes_camera = detections.get('cubes_realworld_cm', [])
        tapes_camera = detections.get('tape_polygons_realworld_cm', [])
        output_buffer.append("Detected Cubes:")
        if not cubes_camera:
            output_buffer.append("  No cubes detected.")
        for i, cube_cam in enumerate(cubes_camera):
            cam_x, cam_y = cube_cam['center_realworld_cm']
            world_x, world_y = transform_point_to_world(rx, ry, r_heading, cam_x, cam_y)
            size = cube_cam.get('size_classification', 'unknown')
            area = cube_cam.get('area_sq_cm', 0)
            cube_class = cube_cam.get('class', 'unknown')
            confidence = cube_cam.get('confidence', 0)
            output_buffer.append(f"  Cube {i+1} ({cube_class}, {size}, {area:.1f}cm², Conf={confidence:.2f})")
            output_buffer.append(f"    Relative Coords (cam_x, cam_y): ({cam_x:.1f}, {cam_y:.1f}) cm")
            output_buffer.append(f"    World Coords    (world_x, world_y): ({world_x:.1f}, {world_y:.1f}) cm")
        output_buffer.append("Detected Tape Polygons (World Coordinates):")
        if not tapes_camera:
            output_buffer.append("  No tape polygons detected.")
        for i, tape_cam in enumerate(tapes_camera):
            if tape_cam.get('points_realworld_cm') and len(tape_cam['points_realworld_cm']) > 0:
                world_points = []
                for point_cam_x, point_cam_y in tape_cam['points_realworld_cm']:
                    world_x, world_y = transform_point_to_world(rx, ry, r_heading, point_cam_x, point_cam_y)
                    world_points.append((world_x, world_y))
                color = tape_cam.get('color', 'unknown')
                area = tape_cam.get('area_sq_cm', 0)
                output_buffer.append(f"  Tape {i+1} ({color}, {area:.1f}cm²): {len(world_points)} points")
                for j, (wx, wy) in enumerate(world_points[:3]):
                    output_buffer.append(f"    Point {j+1}: ({wx:.1f}, {wy:.1f}) cm")
                if len(world_points) > 3:
                    output_buffer.append(f"    ... and {len(world_points)-3} more points")
            else:
                output_buffer.append(f"  Tape {i+1}: No points data.")
    else:
        output_buffer.append("  No fresh detection data available or processor not ready.")
    output_buffer.append("-------------------------------------------")
    for line in output_buffer:
        sys.stdout.write(line + "\r\n")
    sys.stdout.flush()
def get_key_stdin():
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None
def mqtt_publish_loop():
    global mqtt_publish_active
    while mqtt_publish_active:
        try:
            publish_robot_data()
            time.sleep(0.5)
        except Exception as e:
            print(f"Error in MQTT publish loop: {e}")
            time.sleep(1.0)
def start_mqtt_publishing():
    global mqtt_publish_thread, mqtt_publish_active
    if not mqtt_client:
        return False
    mqtt_publish_active = True
    mqtt_publish_thread = threading.Thread(target=mqtt_publish_loop, daemon=True)
    mqtt_publish_thread.start()
    print("Started continuous MQTT publishing (2Hz)")
    return True
def stop_mqtt_publishing():
    global mqtt_publish_thread, mqtt_publish_active
    mqtt_publish_active = False
    if mqtt_publish_thread and mqtt_publish_thread.is_alive():
        mqtt_publish_thread.join(timeout=2.0)
        print("Stopped MQTT publishing thread")
def custom_key_listener():
    global robot_ctl, detection_proc, last_movement_time
    print("\n--- Integrated Robot Control & Detection ---")
    print("Controls:")
    print("  W - Move forward | S - Move backward")
    print("  A - Turn left    | D - Turn right")
    print("  Space - Stop robot")
    print("  H - Show robot pose and detection info")
    print("  O - Robot go to origin (0,0)")
    print("  L - Test left wheel 360° rotation")
    print("  R - Reset position")
    print("  I - Imprint current scene (discard 0.7s, then capture)")
    print("  + - Increase speed | - - Decrease speed")
    print("  M - Send map data via MQTT")
    print("  Use map visualization for:")
    print("    • Click to navigate to position")
    print("    • Manual turn/move with precise values")
    print("  Q - Quit")
    print("-------------------------------------------\n")
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key_data = {}
        while True:
            key = get_key_stdin()
            current_time = time.time()
            if key is not None:
                key_lower = key.lower()
                action_taken = True
                if key_lower == 'w':
                    direction = 'forward'
                elif key_lower == 's':
                    direction = 'backward'
                elif key_lower == 'a':
                    direction = 'left'
                elif key_lower == 'd':
                    direction = 'right'
                else:
                    direction = None
                if direction:
                    if direction not in key_data or not key_data[direction]['pressed']:
                        key_data[direction] = {'pressed': True, 'last_seen': current_time}
                        robot_ctl.move(direction)
                        last_movement_time = current_time
                        print(f"Moving: {direction}")
                    else:
                        key_data[direction]['last_seen'] = current_time
                        last_movement_time = current_time
                elif key == ' ':
                    robot_ctl.stop(force_reset=True)
                    print("Robot stopped.")
                    for d_key in ['forward', 'backward', 'left', 'right']:
                        if d_key in key_data: key_data[d_key]['pressed'] = False
                elif key_lower == 'h':
                    display_detection_info()
                elif key_lower == 'o':
                    print("Commanding robot to origin...")
                    robot_ctl.go_to_origin()
                elif key_lower == 'l':
                    print("Starting left wheel 360° rotation test...")
                    robot_ctl.test_single_wheel_rotation_left()
                elif key_lower == 'r':
                    print("resetting position...")
                    robot_ctl.reset_position()
                elif key_lower == 'i':
                    print("Starting imprinting process...")
                    perform_imprinting()
                elif key_lower == 'm':
                    print("Sending map data via MQTT...")
                    if mqtt_client:
                        topics = robot_config["mqtt_topics"]
                        mqtt_client.publish(f"{topics['pose']}_map_request", json.dumps({"request": "send_map"}, separators=(',', ':')))
                    else:
                        print("MQTT not available")
                elif key_lower == 'q':
                    print("Quitting...")
                    break
                elif key == '+':
                    robot_ctl.set_speed(faster=True)
                elif key == '-':
                    robot_ctl.set_speed(faster=False)
                elif key_lower == 'u':
                    print("Starting autonomous independent mode...")
                    try:
                        import autonomous_independant
                        threading.Thread(target=autonomous_independant.run_autonomous_mode, args=(robot_ctl, detection_proc, mqtt_client, robot_config), daemon=True).start()
                    except Exception as e:
                        print(f"Failed to start autonomous mode: {e}")
                elif command == 'autonav2':
                    print("\nReceived autonav2 command: starting autonomous_independant SIMPLE DEMO mode")
                    try:
                        import autonomous_independant
                        auto_thread = threading.Thread(
                            target=autonomous_independant.run_simple_demo_mode,
                            args=(robot_ctl, detection_proc, mqtt_client, robot_config),
                            daemon=True
                        )
                        auto_thread.start()
                    except Exception as e:
                        print(f"Failed to start autonomous_independant simple demo mode: {e}")
                elif command == 'autonav3':
                    print("\nReceived autonav3 command: starting autonomous_independant FULL RUN mode")
                    try:
                        import autonomous_independant
                        auto_thread = threading.Thread(
                            target=autonomous_independant.run_autonomous_mode,
                            args=(robot_ctl, detection_proc, mqtt_client, robot_config),
                            daemon=True
                        )
                        auto_thread.start()
                    except Exception as e:
                        print(f"Failed to start autonomous_independant FULL RUN mode: {e}")
                else:
                    action_taken = False
                if action_taken:
                    if direction:
                        for d_key, d_info in key_data.items():
                            if d_key != direction and d_info['pressed']:
                                d_info['pressed'] = False
            for k_dir, k_info in list(key_data.items()):
                if k_info.get('pressed', False) and (current_time - k_info.get('last_seen', 0) > RELEASE_THRESHOLD):
                    k_info['pressed'] = False
                    print(f"Key {k_dir} released (timeout)")
                    if robot_ctl.current_direction == k_dir:
                        robot_ctl.stop(force_reset=True)
                        print(f"Robot stopped due to {k_dir} key release.")
            time.sleep(0.02)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nRestored terminal settings.")
        stop_mqtt_publishing()
        if robot_ctl:
            print("Shutting down robot controller...")
            robot_ctl.shutdown()
        if detection_proc:
            print("Stopping detection processor...")
            detection_proc.stop_processing()
        if mqtt_client:
            print("Disconnecting MQTT client...")
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        print("Cleanup complete. Exiting program.")
        os.kill(os.getpid(), signal.SIGINT)
def perform_imprinting():
    global robot_ctl, detection_proc, mqtt_client
    if not detection_proc or not detection_proc.is_ready():
        print("Detection processor not ready for imprinting")
        return
    try:
        imprint_event = detection_proc.start_imprinting(duration_seconds=0.7)
        print("Imprinting in progress - discarding old data...")
        if imprint_event.wait(timeout=5.0):
            imprint_result = detection_proc.get_imprinting_result()
            if imprint_result:
                detections = imprint_result.get("detections", {})
                cubes = detections.get('cubes_realworld_cm', [])
                tapes = detections.get('tape_polygons_realworld_cm', [])
                print(f"\nImprinting complete! Captured scene:")
                print(f"  Cubes detected: {len(cubes)}")
                print(f"  Tape polygons detected: {len(tapes)}")
                if robot_ctl:
                    rx, ry, r_heading = robot_ctl.x, robot_ctl.y, robot_ctl.heading
                    for i, cube in enumerate(cubes):
                        cam_x, cam_y = cube['center_realworld_cm']
                        world_x, world_y = transform_point_to_world(rx, ry, r_heading, cam_x, cam_y)
                        cube_class = cube.get('class', 'unknown')
                        size = cube.get('size_classification', 'unknown')
                        conf = cube.get('confidence', 0)
                        print(f"    Cube {i+1}: {cube_class} ({size}) at world ({world_x:.1f}, {world_y:.1f}) cm, conf={conf:.2f}")
                    for i, tape in enumerate(tapes):
                        points = tape.get('points_realworld_cm', [])
                        area = tape.get('area_sq_cm', 0)
                        print(f"    Tape {i+1}: {len(points)} points, area={area:.1f}cm²")
                if mqtt_client and robot_ctl:
                    print(detections)
                    publish_imprinted_data(detections, robot_ctl.x, robot_ctl.y, robot_ctl.heading, mqtt_client, robot_config)
            else:
                print("Imprinting completed but no data captured")
        else:
            print("Imprinting timed out")
    except Exception as e:
        print(f"Error during imprinting: {e}")
def publish_imprinted_data(detections, robot_x, robot_y, robot_heading, mqtt_client, robot_config, send_latest_pose=False):
    try:
        cubes_data = []
        for cube_cam in detections.get('cubes_realworld_cm', []):
            cam_x, cam_y = cube_cam['center_realworld_cm']
            world_x, world_y = transform_point_to_world(robot_x, robot_y, robot_heading, cam_x, cam_y)
            cube_class = cube_cam.get('class', 'unknown')
            cube_color = cube_class.split()[0] if ' ' in cube_class else cube_class
            world_polygon = None
            if 'bbox_polygon_realworld_cm' in cube_cam:
                world_polygon = []
                for point_cam_x, point_cam_y in cube_cam['bbox_polygon_realworld_cm']:
                    world_point_x, world_point_y = transform_point_to_world(robot_x, robot_y, robot_heading, point_cam_x, point_cam_y)
                    world_polygon.append([world_point_x, world_point_y])
            world_polygon = simplify_and_round_polygon(world_polygon, max_points=4)
            cube_dict = {
                'world_x': world_x,
                'world_y': world_y,
                'cam_x': cam_x,
                'cam_y': cam_y,
                'class': cube_class,
                'size_classification': cube_cam.get('size_classification', 'unknown'),
                'area_sq_cm': cube_cam.get('area_sq_cm', 0),
                'confidence': cube_cam.get('confidence', 0),
                'color': cube_color,
                'polyline': encode_polygon_polyline(world_polygon),
            }
            cubes_data.append(cube_dict)
        tapes_data = []
        for tape_cam in detections.get('tape_polygons_realworld_cm', []):
            if tape_cam.get('points_realworld_cm') and len(tape_cam['points_realworld_cm']) > 0:
                world_points = []
                for point_cam_x, point_cam_y in tape_cam['points_realworld_cm']:
                    world_x, world_y = transform_point_to_world(robot_x, robot_y, robot_heading, point_cam_x, point_cam_y)
                    world_points.append([world_x, world_y])
                world_points = simplify_and_round_polygon(world_points, max_points=16)
                tape_data = {
                    'polyline': encode_polygon_polyline(world_points),
                    'points_count': len(tape_cam['points_realworld_cm']),
                    'area_sq_cm': tape_cam.get('area_sq_cm', 0),
                    'color': tape_cam.get('color', 'unknown')
                }
                tapes_data.append(tape_data)
        imprint_data = {
            'robot_id': robot_config['robot_id'],
            'robot_pose': {
                'x': robot_x,
                'y': robot_y,
                'h': robot_heading
            },
            'cubes': cubes_data,
            'tapes': tapes_data,
            'boxes': [],
            'imprinted': True
        }
        if send_latest_pose:
            imprint_data['latest_pose'] = {
                'x': robot_x,
                'y': robot_y,
                'h': robot_heading
            }
        topics = robot_config["mqtt_topics"]
        mqtt_client.publish(topics["imprinted_data"], json.dumps(imprint_data, separators=(',', ':')))
        print("Published imprinted data via MQTT")
    except Exception as e:
        print(f"Error publishing imprinted data: {e}")
def perform_imprinting_from_mqtt():
    global robot_ctl, detection_proc, mqtt_client
    if not detection_proc or not detection_proc.is_ready():
        print("Detection processor not ready for imprinting")
        return
    try:
        imprint_event = detection_proc.start_imprinting(duration_seconds=0.7)
        print("MQTT Imprinting in progress - discarding old data...")
        if imprint_event.wait(timeout=5.0):
            imprint_result = detection_proc.get_imprinting_result()
            if imprint_result and robot_ctl and mqtt_client:
                detections = imprint_result.get("detections", {})
                cubes = detections.get('cubes_realworld_cm', [])
                tapes = detections.get('tape_polygons_realworld_cm', [])
                print(f"MQTT Imprinting complete! Captured scene:")
                print(f"  Cubes detected: {len(cubes)}")
                print(f"  Tape polygons detected: {len(tapes)}")
                if USE_ESP_MQTT:
                    publish_imprinted_data(detections, robot_ctl.x, robot_ctl.y, robot_ctl.heading, mqtt_client, robot_config)
            else:
                print("MQTT Imprinting completed but no data captured or components unavailable")
        else:
            print("MQTT Imprinting timed out")
    except Exception as e:
        print(f"Error during MQTT imprinting: {e}")
def execute_correction_factor_update(turn_ccw_factor, turn_cw_factor, distance_factor, left_understep_percent=None):
    global robot_ctl
    try:
        if not robot_ctl:
            print("Robot controller not available")
            return
        print(f"Updating correction factors:")
        if turn_ccw_factor is not None:
            print(f"  Turn CCW: {turn_ccw_factor:.4f}")
        if turn_cw_factor is not None:
            print(f"  Turn CW: {turn_cw_factor:.4f}")
        print(f"  Distance: {distance_factor:.4f}")
        if left_understep_percent is not None:
            print(f"  Left wheel understepping: {left_understep_percent:.1f}%")
        robot_ctl.update_correction_factors(
            turn_factor_ccw=turn_ccw_factor,
            turn_factor_cw=turn_cw_factor,
            distance_factor=distance_factor,
            left_understep_percent=left_understep_percent
        )
        print(f"Correction factors updated successfully")
    except Exception as e:
        print(f"Error during correction factor update: {e}")
def execute_physical_parameters_update(wheel_radius, wheel_base):
    global robot_ctl
    try:
        if not robot_ctl:
            print("Robot controller not available")
            return
        print(f"Updating physical parameters: Wheel radius={wheel_radius:.2f}cm, Wheel base={wheel_base:.2f}cm")
        robot_ctl.WHEEL_RADIUS_CM = wheel_radius
        robot_ctl.WHEEL_BASE_CM = wheel_base
        robot_ctl.CM_PER_STEP = 2 * math.pi * robot_ctl.WHEEL_RADIUS_CM / robot_ctl.STEPS_PER_REV
        robot_ctl.MIN_TURN_ANGLE_RAD = (2 * robot_ctl.CM_PER_STEP) / robot_ctl.WHEEL_BASE_CM * robot_ctl.TURN_CORRECTION_FACTOR
        robot_ctl.MIN_TURN_ANGLE_DEG = math.degrees(robot_ctl.MIN_TURN_ANGLE_RAD)
        print(f"Physical parameters updated successfully")
    except Exception as e:
        print(f"Error during physical parameters update: {e}")
def execute_speed_multiplier_update(speed_multiplier):
    global robot_ctl
    try:
        if not robot_ctl:
            print("Robot controller not available")
            return
        print(f"Updating speed multiplier: {speed_multiplier:.2f}")
        robot_ctl.set_speed_multiplier(speed_multiplier)
        print(f"Speed multiplier updated successfully")
    except Exception as e:
        print(f"Error during speed multiplier update: {e}")
def execute_reset_to_origin():
    global robot_ctl
    try:
        if not robot_ctl:
            print("Robot controller not available")
            return
        print("Resetting robot position to origin (0, 0, 0°)")
        robot_ctl.reset_position()
        publish_robot_data(force=True)
        print("Robot position reset to origin successfully")
    except Exception as e:
        print(f"Error during reset to origin: {e}")
def load_robot_configuration():
    global robot_config
    if not ROBOT_ID:
        robot_config = {
            "robot_id": "robot",
            "phone_type": "default",
            "adb_specific": "",
            "robot_parameters": {
                "wheel_radius_cm": 3.2,
                "wheel_base_cm": 13.0,
                "turn_ccw_correction_factor": 1.0,
                "turn_cw_correction_factor": 1.0,
                "distance_correction_factor": 1.0,
                "left_understep_percent": 0.0,
                "speed_multiplier": 1.7
            },
            "mqtt_topics": {
                "pose": "robot/pose",
                "detections": "robot/detections",
                "imprinted_data": "robot/imprinted_data",
                "navigation": "robot/navigation",
                "manual_control": "robot/manual_control",
                "imprint_command": "robot/imprint_command"
            }
        }
        print("Using single robot mode with default configuration")
        return True
    try:
        with open(ROBOT_CONFIG_FILE, "r") as f:
            configs = json.load(f)
        target_robot = None
        for config in configs:
            if config.get("robot_id") == ROBOT_ID:
                target_robot = config
                break
        if not target_robot:
            print(f"Error: Robot ID '{ROBOT_ID}' not found in {ROBOT_CONFIG_FILE}")
            return False
        robot_config = target_robot
        print(f"Loaded configuration for robot: {ROBOT_ID}")
        print(f"Phone type: {robot_config['phone_type']}")
        print(f"ADB specific: {robot_config.get('adb_specific', 'default')}")
        return True
    except FileNotFoundError:
        print(f"Error: {ROBOT_CONFIG_FILE} not found. Using default configuration.")
        robot_config = {
            "robot_id": ROBOT_ID or "robot",
            "phone_type": "default",
            "adb_specific": "",
            "robot_parameters": {
                "wheel_radius_cm": 3.2,
                "wheel_base_cm": 13.0,
                "turn_ccw_correction_factor": 1.0,
                "turn_cw_correction_factor": 1.0,
                "distance_correction_factor": 1.0,
                "left_understep_percent": 0.0,
                "speed_multiplier": 1.7
            },
            "mqtt_topics": {
                "pose": f"{ROBOT_ID}/p" if ROBOT_ID else "robot/pose",
                "detections": f"{ROBOT_ID}/d" if ROBOT_ID else "robot/detections",
                "imprinted_data": f"{ROBOT_ID}/i" if ROBOT_ID else "robot/imprinted_data",
                "navigation": f"{ROBOT_ID}/n" if ROBOT_ID else "robot/navigation",
                "manual_control": f"{ROBOT_ID}/m" if ROBOT_ID else "robot/manual_control",
                "imprint_command": f"{ROBOT_ID}/ip" if ROBOT_ID else "robot/imprint_command"
            }
        }
        return True
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in {ROBOT_CONFIG_FILE}: {e}")
        return False
def simplify_and_round_polygon(points, max_points=12, decimals=3):
    if not points or len(points) <= max_points:
        return [[round(x, decimals), round(y, decimals)] for x, y in points]
    line = LineString(points)
    tolerance = 0.5
    simplified = line.simplify(tolerance, preserve_topology=False)
    simplified_points = list(simplified.coords)
    if len(simplified_points) > max_points:
        step = max(1, len(simplified_points) // max_points)
        simplified_points = simplified_points[::step]
        if simplified_points[0] != simplified_points[-1]:
            simplified_points.append(simplified_points[0])
    return [[round(x, decimals), round(y, decimals)] for x, y in simplified_points]
def encode_polygon_polyline(points):
    if not points or len(points) < 4:
        return None
    if points[0] != points[-1]:
        points = points + [points[0]]
    from shapely.geometry import LineString
    line = LineString(points)
    simplified = line.simplify(2.0, preserve_topology=False)
    coords = [[round(x, 2), round(y, 2)] for x, y in simplified.coords]
    coords_swapped = [[y, x] for x, y in coords]
    if len(coords_swapped) < 4:
        return None
    if coords_swapped[0] != coords_swapped[-1]:
        coords_swapped.append(coords_swapped[0])
    return polyline.encode(coords_swapped, precision=2)
def main():
    global robot_ctl, detection_proc
    def signal_handler(sig, frame):
        print('\nCtrl+C or SIGINT received, initiating shutdown...')
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    print("=== Integrated Robot Control & Detection ===")
    if not load_robot_configuration():
        print("Failed to load robot configuration")
        return
    print(f"Robot ID: {robot_config['robot_id']}")
    if ROBOT_ID:
        print(f"Multi-robot mode - Robot: {ROBOT_ID}")
    else:
        print("Single robot mode")
    print("Initializing Robot Controller...")
    try:
        params = robot_config["robot_parameters"]
        default_pos = robot_config.get("default_position", {"x": 0.0, "y": 0.0, "heading": 0.0})
        robot_ctl = RobotController(
            wheel_radius_cm=params.get("wheel_radius_cm", 3.25),
            wheel_base_cm=params.get("wheel_base_cm", 16.0),
            turn_ccw_correction_factor=params.get("turn_ccw_correction_factor", 1.0),
            turn_cw_correction_factor=params.get("turn_cw_correction_factor", 1.0),
            distance_correction_factor=params.get("distance_correction_factor", 1.0),
            left_understep_percent=params.get("left_understep_percent", 0.0),
            speed_multiplier=params.get("speed_multiplier", 1.7)
        )
        robot_ctl.x = default_pos["x"]
        robot_ctl.y = default_pos["y"]
        robot_ctl.heading = default_pos["heading"]
        robot_ctl.start()
        print(f"✓ Robot Controller initialized for {robot_config['robot_id']}")
        print(f"  Default position: ({robot_ctl.x:.1f}, {robot_ctl.y:.1f}) cm, heading: {robot_ctl.heading:.1f}°")
        publish_robot_data(force=True)
    except Exception as e:
        print(f"Failed to initialize Robot Controller: {e}")
        return
    print("Initializing Detection Processor...")
    try:
        detection_proc = DetectionProcessor(
            phone_type=robot_config["phone_type"],
            adb_specific=robot_config.get("adb_specific", ""),
            robot_id=robot_config["robot_id"]
        )
        detection_proc.start_processing()
        print(f"✓ Detection Processor initialized for {robot_config['robot_id']}")
    except Exception as e:
        print(f"Failed to initialize Detection Processor: {e}")
        if robot_ctl: robot_ctl.shutdown()
        return
    print("Setting up MQTT connection...")
    mqtt_success = setup_mqtt()
    if not mqtt_success:
        print("Continuing without MQTT...")
    else:
        start_mqtt_publishing()
    print("Waiting for Detection Processor initialization...")
    if not detection_proc.wait_for_initialization(timeout=15):
        print("Warning: Detection Processor did not initialize within 15 seconds.")
    else:
        print("Detection Processor initialized and ready.")
        if mqtt_client:
            ready_topic = f"{robot_config['robot_id']}/ready"
            mqtt_client.publish(ready_topic, "1")
            print(f"Published '1' to {ready_topic} (robot ready)")
    load_robot_configuration()
    try:
        custom_key_listener()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt caught in main. Program will exit.")
    except Exception as e:
        print(f"An unexpected error occurred in key listener: {e}")
    finally:
        print("Main function's final cleanup...")
        stop_mqtt_publishing()
        if robot_ctl and getattr(robot_ctl, 'stepper', None) and getattr(robot_ctl.stepper, '_initialised', False):
             if not getattr(robot_ctl, '_shutdown_called', False):
                robot_ctl.shutdown()
        if detection_proc and detection_proc.is_processing:
             detection_proc.stop_processing()
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        print("Main function exiting.")
if __name__ == "__main__":
    if os.geteuid() != 0:
        print("Warning: This script might need root privileges for hardware access (Stepper motors).")
    main()