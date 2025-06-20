import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button, TextBox, CheckButtons
import matplotlib.animation as animation
import numpy as np
import json
import time
import threading
from datetime import datetime
USE_LAPTOP_BRIDGE = True
import sys
import os
if USE_LAPTOP_BRIDGE:
    from paho_bridge_client import MQTTClient as BridgeMQTTClient
    mqtt_client_class = BridgeMQTTClient
else:
    import paho.mqtt.client as mqtt
    mqtt_client_class = lambda: mqtt.Client(protocol=mqtt.MQTTv311)
from typing import Dict, List, Optional
import math
from detection_classes import Box, Robot
import cv2
from camera_view_filter import CameraViewFilter
from object_tracker import ObjectTracker
from autonomous_navigation import AutonomousNavigation
from shapely.geometry import Polygon, Point
from camera_calibration import PHONE_CONFIGS, INCHES_TO_CM
import polyline
class SimpleMapVisualizer:
    def __init__(self, mqtt_broker="100.69.37.102", mqtt_port=1883):
        self.current_robot = "robot1"
        self.available_robots = ["robot1", "robot2"]
        self.show_both_robots = True
        self.avoid_obstacles = False
        self.button_height = 0.04
        self.button_width = 0.2
        self.left_margin = 0.01
        self.top_start = 0.95
        self.gap = 0.06
        self.checkbox_height = 0.03
        self.robot_configs = self.load_robot_configurations()
        self.robot_data = {
            "robot1": {
                "x": 0, "y": 0, "heading": 0,
                "path": [],
                "detected_boxes": [],
                "detected_tapes": [],
                "raw_detected_boxes": [],
                "raw_detected_tapes": [],
                "raw_detected_robots": [],
                "last_update_time": time.time(),
                "camera_filter": CameraViewFilter(),
                "navigating": False,
                "navigation_target": None
            },
            "robot2": {
                "x": 22, "y": 0, "heading": 180,
                "path": [],
                "detected_boxes": [],
                "detected_tapes": [],
                "raw_detected_boxes": [],
                "raw_detected_tapes": [],
                "raw_detected_robots": [],
                "last_update_time": time.time(),
                "camera_filter": CameraViewFilter(),
                "navigating": False,
                "navigation_target": None
            }
        }
        for robot_id in self.available_robots:
            if robot_id in self.robot_configs and "default_position" in self.robot_configs[robot_id]:
                default_pos = self.robot_configs[robot_id]["default_position"]
                self.robot_data[robot_id]["x"] = default_pos["x"]
                self.robot_data[robot_id]["y"] = default_pos["y"]
                self.robot_data[robot_id]["heading"] = default_pos["heading"]
        self.client = mqtt_client_class()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(mqtt_broker, mqtt_port)
        self.client.loop_start()
        self.camera_filter = CameraViewFilter()
        self.confidence_thresholds = {
            'black cube': 0.80,
            'blue cube': 0.80,
            'green cube': 0.80,
            'red cube': 0.80,
            'white cube': 0.80,
            'box': 0.0,
            'robot': 0.0
        }
        self.robot_x = self.robot_data[self.current_robot]["x"]
        self.robot_y = self.robot_data[self.current_robot]["y"]
        self.robot_heading = self.robot_data[self.current_robot]["heading"]
        self.robot_path = self.robot_data[self.current_robot]["path"]
        self.detected_boxes = self.robot_data[self.current_robot]["detected_boxes"]
        self.detected_tapes = self.robot_data[self.current_robot]["detected_tapes"]
        self.raw_detected_boxes = self.robot_data[self.current_robot]["raw_detected_boxes"]
        self.raw_detected_tapes = self.robot_data[self.current_robot]["raw_detected_tapes"]
        self.last_update_time = self.robot_data[self.current_robot]["last_update_time"]
        self.cube_imprints = []
        self.tape_imprints = []
        self.IMPRINT_TIMEOUT = 10.0
        self.robot_imprint_states = {
            "robot1": {
                "command_pending": False,
                "command_time": None,
                "imprint_for_add_objects": False
            },
            "robot2": {
                "command_pending": False,
                "command_time": None,
                "imprint_for_add_objects": False
            }
        }
        self.data_lock = threading.Lock()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_title('Robot Map - BOTH ROBOTS')
        self.min_x = -100
        self.max_x = 100
        self.min_y = -100
        self.max_y = 100
        self.object_tracker = ObjectTracker(overlap_threshold=0.01)
        self.tracked_objects = {
            'tapes': [],
            'craters': [],
            'cubes': [],
            'boxes': []
        }
        self.autonomous_nav = AutonomousNavigation(self.client, self.current_robot, self)
        self.filter_mode = 1
        self.setup_buttons()
        self.set_filter_mode(self.filter_mode)
        self.setup_manual_controls()
        self.update_robot_button_colors()
        self.update_filter_button_colors()
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.debug_add_objects = False
        self.debug=False
        self.camera_bounds = {}
        for robot_id in self.available_robots:
            phone_type = get_robot_phone_type(robot_id)
            bounds = calculate_camera_bounds(phone_type)
            print("for phone type: ", phone_type, "camera bounds: ", bounds)
            self.robot_data[robot_id]["camera_filter"].set_camera_bounds(bounds)
        self.ready_toasts = []
    def setup_buttons(self):
        send_imprint_button_ax = plt.axes([self.left_margin, self.top_start, self.button_width, self.button_height])
        self.send_imprint_button = Button(send_imprint_button_ax, 'Send Imprint Cmd',
                                        color='lightyellow', hovercolor='yellow')
        self.send_imprint_button.on_clicked(self.send_imprint_command)
        if self.show_both_robots:
            add_objects_r1_button_ax = plt.axes([self.left_margin, self.top_start-self.gap, self.button_width/2-0.01, self.button_height])
            self.add_objects_r1_button = Button(add_objects_r1_button_ax, 'Add R1 Objects',
                                           color='lightgreen', hovercolor='green')
            self.add_objects_r1_button.on_clicked(lambda x: self.add_current_objects_for_robot(x, "robot1"))
            add_objects_r2_button_ax = plt.axes([self.left_margin + self.button_width/2, self.top_start-self.gap, self.button_width/2, self.button_height])
            self.add_objects_r2_button = Button(add_objects_r2_button_ax, 'Add R2 Objects',
                                           color='lightgreen', hovercolor='green')
            self.add_objects_r2_button.on_clicked(lambda x: self.add_current_objects_for_robot(x, "robot2"))
        else:
            add_objects_button_ax = plt.axes([self.left_margin, self.top_start-self.gap, self.button_width, self.button_height])
            self.add_objects_button = Button(add_objects_button_ax, 'Add Current Objects',
                                           color='lightgreen', hovercolor='green')
            self.add_objects_button.on_clicked(self.add_current_objects)
        clear_button_ax = plt.axes([self.left_margin, self.top_start-2*self.gap, self.button_width, self.button_height])
        self.clear_button = Button(clear_button_ax, 'Clear Imprints',
                                 color='lightcoral', hovercolor='red')
        self.clear_button.on_clicked(self.clear_imprints)
        clear_objects_button_ax = plt.axes([self.left_margin, self.top_start-3*self.gap, self.button_width, self.button_height])
        self.clear_objects_button = Button(clear_objects_button_ax, 'Clear Objects',
                                         color='lightblue', hovercolor='blue')
        self.clear_objects_button.on_clicked(self.clear_objects)
        robot1_button_ax = plt.axes([self.left_margin, self.top_start-4*self.gap, self.button_width/2-0.01, self.button_height])
        self.robot1_button = Button(robot1_button_ax, 'Robot1',
                                   color='lightpink', hovercolor='pink')
        self.robot1_button.on_clicked(lambda x: self.switch_robot("robot1"))
        robot2_button_ax = plt.axes([self.left_margin + self.button_width/2, self.top_start-4*self.gap, self.button_width/2, self.button_height])
        self.robot2_button = Button(robot2_button_ax, 'Robot2',
                                   color='lightcyan', hovercolor='cyan')
        self.robot2_button.on_clicked(lambda x: self.switch_robot("robot2"))
        checkbox_ax = plt.axes([self.left_margin, self.top_start-5*self.gap, self.button_width, self.checkbox_height])
        self.show_raw_detections = [False]
        self.raw_checkbox = CheckButtons(checkbox_ax, ['Show Raw Detections'], [False])
        self.raw_checkbox.on_clicked(self.toggle_show_raw_detections)
        both_robots_checkbox_ax = plt.axes([self.left_margin, self.top_start-6*self.gap, self.button_width, self.checkbox_height])
        self.show_both_robots_checkbox = CheckButtons(both_robots_checkbox_ax, ['Show Both Robots'], [True])
        self.show_both_robots_checkbox.on_clicked(self.toggle_show_both_robots)
        avoid_obstacles_checkbox_ax = plt.axes([self.left_margin, self.top_start-7*self.gap, self.button_width, self.checkbox_height])
        self.avoid_obstacles_checkbox = CheckButtons(avoid_obstacles_checkbox_ax, ['Avoid Obstacles'], [False])
        self.avoid_obstacles_checkbox.on_clicked(self.toggle_avoid_obstacles)
        stop_nav_button_ax = plt.axes([self.left_margin, self.top_start-8*self.gap, self.button_width, self.button_height])
        self.stop_nav_button = Button(stop_nav_button_ax, 'Stop Navigation',
                                    color='lightcoral', hovercolor='red')
        self.stop_nav_button.on_clicked(self.stop_navigation)
        auto_nav_button_ax = plt.axes([self.left_margin, self.top_start-9*self.gap, self.button_width, self.button_height])
        self.auto_nav_button = Button(auto_nav_button_ax, 'Start Auto Nav',
                                    color='lightgreen', hovercolor='green')
        self.auto_nav_button.on_clicked(self.start_autonomous_navigation)
        both_auto_nav_button_ax = plt.axes([self.left_margin, self.top_start-10*self.gap, self.button_width, self.button_height])
        self.both_auto_nav_button = Button(both_auto_nav_button_ax, 'Start BOTH Auto Nav',
                                    color='lightgreen', hovercolor='green')
        self.both_auto_nav_button.on_clicked(self.start_both_autonomous_navigation)
        autonav2_button_ax = plt.axes([self.left_margin, self.top_start-11*self.gap, self.button_width, self.button_height])
        self.autonav2_button = Button(autonav2_button_ax, 'Start AutoNav2',
                                    color='lightyellow', hovercolor='yellow')
        self.autonav2_button.on_clicked(self.send_autonav2_command)
        autonav3_button_ax = plt.axes([self.left_margin, self.top_start-11.5*self.gap, self.button_width, self.button_height])
        self.autonav3_button = Button(autonav3_button_ax, 'Start AutoNav3',
                                    color='orange', hovercolor='red')
        self.autonav3_button.on_clicked(self.send_autonav3_command)
        debug_boxes_button_ax = plt.axes([self.left_margin, self.top_start-12*self.gap, self.button_width, self.button_height])
        self.debug_boxes_button = Button(debug_boxes_button_ax, 'Debug Detection Boxes',
                                       color='lightyellow', hovercolor='yellow')
        self.debug_boxes_button.on_clicked(self.debug_detection_boxes)
        debug_general_box_button_ax = plt.axes([self.left_margin, self.top_start-13*self.gap, self.button_width, self.button_height])
        self.debug_general_box_button = Button(debug_general_box_button_ax, 'Debug General Box',
                                              color='lightyellow', hovercolor='yellow')
        self.debug_general_box_button.on_clicked(self.debug_general_box)
        debug_tape_approach_button_ax = plt.axes([self.left_margin, self.top_start-14*self.gap, self.button_width, self.button_height])
        self.debug_tape_approach_button = Button(debug_tape_approach_button_ax, 'Debug Tape Approach Rects',
                                       color='lightyellow', hovercolor='yellow')
        self.debug_tape_approach_button.on_clicked(self.debug_tape_approach_rects)
        minimal_mode_checkbox_ax = plt.axes([self.left_margin, self.top_start-15*self.gap, self.button_width, self.checkbox_height])
        self.show_minimal_mode = [False]
        self.minimal_mode_checkbox = CheckButtons(minimal_mode_checkbox_ax, ['Minimal Mode'], [False])
        self.minimal_mode_checkbox.on_clicked(self.toggle_minimal_mode)
        independant_mode_checkbox_ax = plt.axes([self.left_margin, self.top_start-16*self.gap, self.button_width, self.checkbox_height])
        self.independant_mode = [True]
        self.independant_mode_checkbox = CheckButtons(independant_mode_checkbox_ax, ['Independant Mode'], [True])
        self.independant_mode_checkbox.on_clicked(self.toggle_independant_mode)
        filter_button_width = self.button_width / 3 - 0.005
        no_filter_button_ax = plt.axes([self.left_margin, self.top_start-15.5*self.gap, filter_button_width, self.button_height])
        self.no_filter_button = Button(no_filter_button_ax, 'No Filter',
                                     color='lightgreen', hovercolor='green')
        self.no_filter_button.on_clicked(lambda x: self.set_filter_mode(0))
        simple_filter_button_ax = plt.axes([self.left_margin + filter_button_width + 0.005, self.top_start-15.5*self.gap, filter_button_width, self.button_height])
        self.simple_filter_button = Button(simple_filter_button_ax, 'Simple Filter',
                                         color='lightyellow', hovercolor='yellow')
        self.simple_filter_button.on_clicked(lambda x: self.set_filter_mode(1))
        full_filter_button_ax = plt.axes([self.left_margin + 2*(filter_button_width + 0.005), self.top_start-15.5*self.gap, filter_button_width, self.button_height])
        self.full_filter_button = Button(full_filter_button_ax, 'Full Filter',
                                       color='lightcoral', hovercolor='red')
        self.full_filter_button.on_clicked(lambda x: self.set_filter_mode(2))
    def setup_manual_controls(self):
        left_margin = 0.02
        input_width = 0.08
        button_width = 0.12
        control_height = 0.035
        angle_y = 0.44
        distance_y = 0.40
        angle_label_ax = plt.axes([left_margin, angle_y, 0.06, control_height])
        angle_label_ax.text(0.5, 0.5, 'Turn (°):', ha='center', va='center', fontsize=9)
        angle_label_ax.set_xticks([])
        angle_label_ax.set_yticks([])
        angle_input_ax = plt.axes([left_margin + 0.07, angle_y, input_width, control_height])
        self.turn_degrees = TextBox(angle_input_ax, '', initial='90.0')
        turn_button_ax = plt.axes([left_margin + 0.16, angle_y, button_width, control_height])
        self.turn_button = Button(turn_button_ax, 'Turn', color='lightyellow', hovercolor='yellow')
        self.turn_button.on_clicked(self.execute_turn)
        distance_label_ax = plt.axes([left_margin, distance_y, 0.06, control_height])
        distance_label_ax.text(0.5, 0.5, 'Move (cm):', ha='center', va='center', fontsize=9)
        distance_label_ax.set_xticks([])
        distance_label_ax.set_yticks([])
        distance_input_ax = plt.axes([left_margin + 0.07, distance_y, input_width, control_height])
        self.move_distance = TextBox(distance_input_ax, '', initial='10.0')
        move_button_ax = plt.axes([left_margin + 0.16, distance_y, button_width, control_height])
        self.move_button = Button(move_button_ax, 'Move', color='lightcyan', hovercolor='cyan')
        self.move_button.on_clicked(self.execute_move)
    def send_imprint_command(self, event):
        print("Sending imprint scene command to robot...")
        if self.client:
            filtered_cubes = []
            filtered_boxes = []
            for cube in self.detected_boxes:
                cube_dict = cube.to_dict()
                class_name = cube.class_name.lower()
                if class_name == 'box':
                    filtered_box = self.camera_filter.filter_box(cube_dict)
                    if filtered_box:
                        filtered_boxes.append(filtered_box)
                else:
                    filtered_cube = self.camera_filter.filter_cube(cube_dict)
                    if filtered_cube:
                        filtered_cubes.append(filtered_cube)
            filtered_tapes = []
            for tape in self.detected_tapes:
                if 'world_polygon' in tape and len(tape['world_polygon']) >= 3:
                    classification = self.classify_tape_polygon(tape['world_polygon'])
                    if classification == 'crater':
                        filtered = self.camera_filter.filter_crater(tape)
                    else:
                        filtered = self.camera_filter.filter_tape(tape)
                    if filtered:
                        filtered_tapes.append(filtered)
            for box in self.detected_boxes:
                if hasattr(box, 'class_name') and box.class_name.lower() == 'box':
                    continue
                box_dict = box.to_dict()
                filtered_box = self.camera_filter.filter_box(box_dict)
                if filtered_box:
                    filtered_boxes.append(filtered_box)
            imprint_command = {
                'command': 'imprint_scene',
                'cubes': filtered_cubes,
                'tapes': filtered_tapes,
                'boxes': filtered_boxes
            }
            self.client.publish(f"{self.current_robot}/m", json.dumps(imprint_command))
            print("Imprint scene command sent to robot - waiting for response...")
            self.robot_imprint_states[self.current_robot]["command_pending"] = True
            self.robot_imprint_states[self.current_robot]["command_time"] = time.time()
        else:
            print("MQTT not available - cannot send imprint command")
    def clear_imprints(self, event):
        with self.data_lock:
            cube_count = len(self.cube_imprints)
            tape_count = len(self.tape_imprints)
            self.cube_imprints.clear()
            self.tape_imprints.clear()
            print(f"Cleared {cube_count} cube imprints and {tape_count} tape imprints")
    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code: {rc}")
        self.subscribe_to_robot_topics()
    def subscribe_to_robot_topics(self):
        if self.client:
            for robot in self.available_robots:
                self.client.unsubscribe(f"{robot}/p")
                self.client.unsubscribe(f"{robot}/d")
                self.client.unsubscribe(f"{robot}/i")
                self.client.unsubscribe(f"{robot}/s")
            if self.show_both_robots:
                for robot in self.available_robots:
                    self.client.subscribe(f"{robot}/p")
                    self.client.subscribe(f"{robot}/d")
                    self.client.subscribe(f"{robot}/i")
                    self.client.subscribe(f"{robot}/s")
                    self.client.subscribe(f"{robot}/ready")
                print(f"Subscribed to topics for both robots")
            else:
                self.client.subscribe(f"{self.current_robot}/p")
                self.client.subscribe(f"{self.current_robot}/d")
                self.client.subscribe(f"{self.current_robot}/i")
                self.client.subscribe(f"{self.current_robot}/n")
                print(f"Subscribed to topics for {self.current_robot}")
    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            print("b")
            topic_parts = msg.topic.split('/')
            print("topic_parts", topic_parts)
            if len(topic_parts) >= 2:
                robot_id = topic_parts[0]
                topic_type = topic_parts[1]
                if robot_id == self.current_robot or (self.show_both_robots and robot_id in self.available_robots):
                    if topic_type == "p":
                        self.handle_pose_update(payload, robot_id)
                    elif topic_type == "d":
                        self.handle_detection_update(payload, robot_id)
                    elif topic_type == "i":
                        self.handle_imprinted_data(payload, robot_id)
                    elif topic_type == "s":
                        self.handle_navigation_status(payload, robot_id)
                    elif topic_type == "ready":
                        self.show_ready_toast(robot_id)
        except Exception as e:
            print(f"Error processing MQTT message: {e}")
            print(f"Message payload: {msg.payload.decode()}")
    def switch_robot(self, robot_id):
        if robot_id in self.available_robots:
            if robot_id == self.current_robot:
                print(f"Robot {robot_id} is already selected")
                return False
            old_robot = self.current_robot
            self.current_robot = robot_id
            with self.data_lock:
                self.update_current_robot_references()
                self.camera_filter.camera_bounds = None
                self.robot_data[robot_id]["camera_filter"].camera_bounds = None
            if not self.show_both_robots:
                self.subscribe_to_robot_topics()
            if self.show_both_robots:
                self.ax.set_title('Robot Map - BOTH ROBOTS')
            else:
                self.ax.set_title(f'Robot Map - {self.current_robot.upper()}')
            self.update_robot_button_colors()
            print(f"Switched from {old_robot} to {self.current_robot}")
            return True
        return False
    def update_robot_button_colors(self):
        if self.current_robot == "robot1":
            self.robot1_button.color = 'red'
            self.robot1_button.hovercolor = 'darkred'
            self.robot2_button.color = 'lightgray'
            self.robot2_button.hovercolor = 'gray'
        else:
            self.robot1_button.color = 'lightgray'
            self.robot1_button.hovercolor = 'gray'
            self.robot2_button.color = 'blue'
            self.robot2_button.hovercolor = 'darkblue'
        self.fig.canvas.draw_idle()
    def handle_pose_update(self, pose_data, robot_id=None):
        print("A")
        if robot_id is None:
            robot_id = self.current_robot
        with self.data_lock:
            self.robot_data[robot_id]["x"] = pose_data['x']
            self.robot_data[robot_id]["y"] = pose_data['y']
            self.robot_data[robot_id]["heading"] = pose_data['h']
            self.robot_data[robot_id]["last_update_time"] = time.time()
            self.robot_data[robot_id]["path"].append((pose_data['x'], pose_data['y']))
            if len(self.robot_data[robot_id]["path"]) > 1000:
                self.robot_data[robot_id]["path"] = self.robot_data[robot_id]["path"][-1000:]
            if robot_id == self.current_robot:
                self.robot_x = pose_data['x']
                self.robot_y = pose_data['y']
                self.robot_heading = pose_data['h']
                self.robot_path = self.robot_data[robot_id]["path"]
                self.last_update_time = time.time()
            phone_type = get_robot_phone_type(robot_id)
            camera_polygon = get_camera_view_polygon_robot_frame(phone_type)
            if camera_polygon:
                world_camera_bounds = transform_camera_polygon_to_world(
                    self.robot_data[robot_id]["x"],
                    self.robot_data[robot_id]["y"],
                    self.robot_data[robot_id]["heading"],
                    camera_polygon
                )
                self.robot_data[robot_id]["camera_filter"].set_camera_bounds(world_camera_bounds)
            self.update_map_bounds(pose_data['x'], pose_data['y'])
    def handle_detection_update(self, detection_data, robot_id=None):
        if robot_id is None:
            robot_id = self.current_robot
        with self.data_lock:
            robot_x = self.robot_data[robot_id]["x"]
            robot_y = self.robot_data[robot_id]["y"]
            robot_heading = self.robot_data[robot_id]["heading"]
            self.robot_data[robot_id]["raw_detected_boxes"] = []
            self.robot_data[robot_id]["raw_detected_robots"] = []
            self.robot_data[robot_id]["raw_detected_tapes"] = []
            self.robot_data[robot_id]["detected_boxes"] = []
            self.robot_data[robot_id]["detected_tapes"] = []
            for cube_data in detection_data.get('cubes', []):
                try:
                    if cube_data.get('class', 'unknown').lower() == 'robot':
                        self.robot_data[robot_id]["raw_detected_robots"].append(cube_data)
                        continue
                    if 'polyline' in cube_data:
                        cube_data['world_polygon'] = decode_polyline_to_points(cube_data['polyline'])
                    self.robot_data[robot_id]["raw_detected_boxes"].append(Box.from_detection_data(cube_data, robot_x, robot_y, robot_heading))
                except Exception as e:
                    print(f"Error processing raw cube data for {robot_id}: {e}")
            for tape_data in detection_data.get('tapes', []):
                try:
                    if 'polyline' in tape_data:
                        tape_data['world_polygon'] = decode_polyline_to_points(tape_data['polyline'])
                    if 'world_polygon' in tape_data and len(tape_data['world_polygon']) >= 3:
                        self.robot_data[robot_id]["raw_detected_tapes"].append(tape_data)
                except Exception as e:
                    print(f"Error processing raw tape/crater data for {robot_id}: {e}")
            if self.show_both_robots or robot_id == self.current_robot:
                robot_camera_filter = self.robot_data[robot_id]["camera_filter"]
                for cube_data in detection_data.get('cubes', []):
                    try:
                        class_name = cube_data.get('class', 'unknown').lower()
                        if class_name == 'robot':
                            continue
                        confidence = cube_data.get('confidence', 0.0)
                        threshold = self.confidence_thresholds.get(class_name, 0.8)
                        if confidence >= threshold:
                            if 'polyline' in cube_data:
                                cube_data['world_polygon'] = decode_polyline_to_points(cube_data['polyline'])
                            if class_name == 'box':
                                filtered_box = robot_camera_filter.filter_box(cube_data)
                                if filtered_box:
                                    self.robot_data[robot_id]["detected_boxes"].append(Box.from_detection_data(filtered_box, robot_x, robot_y, robot_heading))
                                    if 'world_polygon' in filtered_box:
                                        for point in filtered_box['world_polygon']:
                                            self.update_map_bounds(point[0], point[1])
                            else:
                                box = Box.from_detection_data(cube_data, robot_x, robot_y, robot_heading)
                                filtered_cube = robot_camera_filter.filter_cube(box.to_dict())
                                if filtered_cube:
                                    self.robot_data[robot_id]["detected_boxes"].append(Box.from_detection_data(filtered_cube, robot_x, robot_y, robot_heading))
                                    self.update_map_bounds(box.world_x, box.world_y)
                                    if box.world_polygon:
                                        for point in box.world_polygon:
                                            self.update_map_bounds(point[0], point[1])
                    except Exception as e:
                        print(f"Error processing cube data for {robot_id}: {e}")
                for tape_data in detection_data.get('tapes', []):
                    try:
                        if 'polyline' in tape_data:
                            tape_data['world_polygon'] = decode_polyline_to_points(tape_data['polyline'])
                        if 'world_polygon' in tape_data and len(tape_data['world_polygon']) >= 3:
                            classification = self.classify_tape_polygon(tape_data['world_polygon'])
                            if classification == 'crater':
                                filtered = robot_camera_filter.filter_crater(tape_data)
                            else:
                                filtered = robot_camera_filter.filter_tape(tape_data)
                            if filtered:
                                self.robot_data[robot_id]["detected_tapes"].append(filtered)
                                for point in filtered['world_polygon']:
                                    self.update_map_bounds(point[0], point[1])
                    except Exception as e:
                        print(f"Error processing tape/crater data for {robot_id}: {e}")
                for box_data in detection_data.get('boxes', []):
                    try:
                        if 'polyline' in box_data:
                            box_data['world_polygon'] = decode_polyline_to_points(box_data['polyline'])
                        if 'world_polygon' in box_data and len(box_data['world_polygon']) >= 3:
                            filtered_box = robot_camera_filter.filter_box(box_data)
                            if filtered_box:
                                self.robot_data[robot_id]["detected_boxes"].append(Box.from_detection_data(filtered_box, robot_x, robot_y, robot_heading))
                                for point in filtered_box['world_polygon']:
                                    self.update_map_bounds(point[0], point[1])
                    except Exception as e:
                        print(f"Error processing box data for {robot_id}: {e}")
            if robot_id == self.current_robot:
                self.raw_detected_boxes = self.robot_data[robot_id]["raw_detected_boxes"]
                self.raw_detected_tapes = self.robot_data[robot_id]["raw_detected_tapes"]
                self.detected_boxes = self.robot_data[robot_id]["detected_boxes"]
                self.detected_tapes = self.robot_data[robot_id]["detected_tapes"]
    def handle_imprinted_data(self, imprinted_data, robot_id=None):
        if robot_id is None:
            robot_id = self.current_robot
        latest_pose = imprinted_data.get('latest_pose', None)
        if latest_pose is not None:
            print(f"[IMPRINT] Latest pose received: x={latest_pose.get('x')}, y={latest_pose.get('y')}, heading={latest_pose.get('h')}")
            self.handle_pose_update(latest_pose, robot_id)
        if self.independant_mode[0]:
            print(imprinted_data)
            cubes_data = imprinted_data.get('cubes', [])
            tapes_data = imprinted_data.get('tapes', [])
            boxes_data = imprinted_data.get('boxes', [])
            robot_pose = imprinted_data.get('robot_pose', {})
            if self.filter_mode == 0:
                filtered_tapes = []
                filtered_craters = []
                for tape in tapes_data:
                    if 'polyline' in tape:
                        tape['world_polygon'] = decode_polyline_to_points(tape['polyline'])
                        print("tape['world_polygon']", tape['world_polygon'])
                    if 'world_polygon' in tape and len(tape['world_polygon']) >= 3:
                        classification = self.classify_tape_polygon(tape['world_polygon'])
                        if classification == 'crater':
                            filtered_craters.append(tape)
                        else:
                            filtered_tapes.append(tape)
                filtered_cubes = []
                filtered_boxes = []
                for cube in cubes_data:
                    if 'polyline' in cube:
                        cube['world_polygon'] = decode_polyline_to_points(cube['polyline'])
                    class_name = cube.get('class', 'unknown').lower()
                    if class_name == 'box':
                        filtered_boxes.append(cube)
                    else:
                        filtered_cubes.append(cube)
                for box in boxes_data:
                    if 'polyline' in box:
                        box['world_polygon'] = decode_polyline_to_points(box['polyline'])
                    filtered_boxes.append(box)
            else:
                robot_camera_filter = self.robot_data[robot_id]["camera_filter"]
                filtered_tapes = []
                filtered_craters = []
                for tape in tapes_data:
                    if 'polyline' in tape:
                        tape['world_polygon'] = decode_polyline_to_points(tape['polyline'])
                        print("tape['world_polygon']", tape['world_polygon'])
                    if 'world_polygon' in tape and len(tape['world_polygon']) >= 3:
                        classification = self.classify_tape_polygon(tape['world_polygon'])
                        if classification == 'crater':
                            print("crater")
                            filtered = robot_camera_filter.filter_crater(tape)
                            if filtered:
                                filtered_craters.append(filtered)
                        else:
                            print("tape")
                            filtered = robot_camera_filter.filter_tape(tape)
                            if filtered:
                                filtered_tapes.append(filtered)
                filtered_cubes = []
                filtered_boxes = []
                for cube in cubes_data:
                    if 'polyline' in cube:
                        cube['world_polygon'] = decode_polyline_to_points(cube['polyline'])
                    class_name = cube.get('class', 'unknown').lower()
                    if class_name == 'box':
                        filtered = robot_camera_filter.filter_box(cube)
                        if filtered:
                            filtered_boxes.append(filtered)
                    else:
                        filtered = robot_camera_filter.filter_cube(cube)
                        if filtered:
                            filtered_cubes.append(filtered)
                for box in boxes_data:
                    if 'polyline' in box:
                        box['world_polygon'] = decode_polyline_to_points(box['polyline'])
                    filtered = robot_camera_filter.filter_box(box)
                    if filtered:
                        filtered_boxes.append(filtered)
            self.object_tracker.add_tapes(filtered_tapes)
            self.object_tracker.add_craters(filtered_craters)
            self.object_tracker.add_cubes(filtered_cubes)
            self.object_tracker.add_boxes(filtered_boxes)
            self.tracked_objects = self.object_tracker.get_all_tracked_objects()
            print(f"[INDEP] Added {len(filtered_tapes)} tapes, {len(filtered_craters)} craters, {len(filtered_cubes)} cubes, {len(filtered_boxes)} boxes to imprints (independant mode)")
            print(f"Robot was at position: ({robot_pose.get('x', 0):.1f}, {robot_pose.get('y', 0):.1f}, {robot_pose.get('heading', 0):.1f}°)")
            return
        if self.robot_imprint_states[robot_id]["command_pending"]:
            current_time = time.time()
            elapsed_time = current_time - self.robot_imprint_states[robot_id]["command_time"]
            if elapsed_time > self.IMPRINT_TIMEOUT:
                print(f"Warning: Imprint command for {robot_id} timed out after {elapsed_time:.1f}s")
                self.robot_imprint_states[robot_id]["command_pending"] = False
        was_waiting = self.robot_imprint_states[robot_id]["command_pending"]
        self.robot_imprint_states[robot_id]["command_pending"] = False
        response_time = time.time() - self.robot_imprint_states[robot_id]["command_time"] if self.robot_imprint_states[robot_id]["command_time"] else 0
        if was_waiting:
            pass
        else:
            print(f"Received unexpected imprinted data from {robot_id}")
        cubes_data = imprinted_data.get('cubes', [])
        tapes_data = imprinted_data.get('tapes', [])
        boxes_data = imprinted_data.get('boxes', [])
        robot_pose = imprinted_data.get('robot_pose', {})
        if not cubes_data and not tapes_data and not boxes_data:
            print("No objects in imprinted data - nothing to process")
            return
        robot_camera_filter = self.robot_data[robot_id]["camera_filter"]
        filtered_tapes = []
        filtered_craters = []
        for tape in tapes_data:
            if 'polyline' in tape:
                tape['world_polygon'] = decode_polyline_to_points(tape['polyline'])
            if 'world_polygon' in tape and len(tape['world_polygon']) >= 3:
                classification = self.classify_tape_polygon(tape['world_polygon'])
                if classification == 'crater':
                    filtered = robot_camera_filter.filter_crater(tape)
                    if filtered:
                        filtered_craters.append(filtered)
                else:
                    filtered = robot_camera_filter.filter_tape(tape)
                    if filtered:
                        filtered_tapes.append(filtered)
        filtered_cubes = []
        filtered_boxes = []
        for cube in cubes_data:
            if 'polyline' in cube:
                cube['world_polygon'] = decode_polyline_to_points(cube['polyline'])
            class_name = cube.get('class', 'unknown').lower()
            if class_name == 'box':
                filtered = robot_camera_filter.filter_box(cube)
                if filtered:
                    filtered_boxes.append(filtered)
            else:
                filtered = robot_camera_filter.filter_cube(cube)
                if filtered:
                    filtered_cubes.append(filtered)
        for box in boxes_data:
            if 'polyline' in box:
                box['world_polygon'] = decode_polyline_to_points(box['polyline'])
            filtered = robot_camera_filter.filter_box(box)
            if filtered:
                filtered_boxes.append(filtered)
        if self.robot_imprint_states[robot_id]["imprint_for_add_objects"]:
            self.object_tracker.add_tapes(filtered_tapes)
            self.object_tracker.add_craters(filtered_craters)
            self.object_tracker.add_cubes(filtered_cubes)
            self.object_tracker.add_boxes(filtered_boxes)
            self.tracked_objects = self.object_tracker.get_all_tracked_objects()
        else:
            self.cube_imprints.extend(filtered_cubes)
            self.cube_imprints.extend(filtered_boxes)
            self.tape_imprints.extend(filtered_tapes)
            self.tape_imprints.extend(filtered_craters)
            print(f"Added {len(filtered_tapes)} tapes, {len(filtered_craters)} craters, {len(filtered_cubes)} cubes, {len(filtered_boxes)} boxes to imprints")
        print(f"Robot was at position: ({robot_pose.get('x', 0):.1f}, {robot_pose.get('y', 0):.1f}, {robot_pose.get('heading', 0):.1f}°)")
        self.robot_imprint_states[robot_id]["imprint_for_add_objects"] = False
        self.debug_add_objects = False
    def update_map_bounds(self, x, y):
        margin = 50
        if not hasattr(self, 'min_x'):
            self.min_x = x - margin
            self.max_x = x + margin
            self.min_y = y - margin
            self.max_y = y + margin
        else:
            self.min_x = min(self.min_x, x - margin)
            self.max_x = max(self.max_x, x + margin)
            self.min_y = min(self.min_y, y - margin)
            self.max_y = max(self.max_y, y + margin)
    def on_click(self, event):
        if event.inaxes != self.ax:
            return
        target_x = event.xdata
        target_y = event.ydata
        if not self.avoid_obstacles:
            if self.client:
                self.robot_data[self.current_robot]["navigating"] = True
                self.robot_data[self.current_robot]["navigation_target"] = (target_x, target_y)
                command = {
                    'command': 'navigate_to',
                    'target_x': float(target_x),
                    'target_y': float(target_y)
                }
                if self.debug:
                    print(f"[DEBUG] Publishing direct navigation command: {command}")
                self.client.publish(f"{self.current_robot}/navigation", json.dumps(command))
                print(f"Navigating directly to ({target_x:.1f}, {target_y:.1f}) cm")
            return
        if self.client:
            robot_pos = (self.robot_x, self.robot_y)
            robot_heading = self.robot_heading
            target_pos = (target_x, target_y)
            self.robot_data[self.current_robot]["navigating"] = True
            self.robot_data[self.current_robot]["navigation_target"] = target_pos
            command = {
                'command': 'navigate_to',
                'target_x': float(target_x),
                'target_y': float(target_y),
                'avoid_obstacles': True
            }
            if self.debug:
                print(f"[DEBUG] Publishing navigation command with obstacle avoidance: {command}")
            self.client.publish(f"{self.current_robot}/navigation", json.dumps(command))
            print(f"Navigating to ({target_x:.1f}, {target_y:.1f}) cm with obstacle avoidance")
    def draw_robot(self):
        if self.show_both_robots:
            colors = {'robot1': ('red', 'darkred'), 'robot2': ('blue', 'darkblue')}
            status_texts = []
            for i, robot_id in enumerate(self.available_robots):
                robot_data = self.robot_data[robot_id]
                color, edge_color = colors[robot_id]
                self.ax.plot(robot_data["x"], robot_data["y"], 'o', markersize=12,
                           markerfacecolor=color, markeredgecolor=edge_color,
                           markeredgewidth=2, label=f'{robot_id.upper()}')
                arrow_length = min(15, (self.max_x - self.min_x) * 0.05)
                heading_rad = np.radians(robot_data["heading"])
                head_x = robot_data["x"] + arrow_length * np.cos(heading_rad)
                head_y = robot_data["y"] + arrow_length * np.sin(heading_rad)
                self.ax.annotate('', xy=(head_x, head_y), xytext=(robot_data["x"], robot_data["y"]),
                               arrowprops=dict(arrowstyle='->', color=color, lw=3))
                if robot_data["navigating"]:
                    self.draw_safety_zones(robot_data["x"], robot_data["y"], robot_data["heading"], color)
                if robot_data["navigating"]:
                    if robot_data["navigation_target"]:
                        target_x, target_y = robot_data["navigation_target"]
                        self.ax.plot(target_x, target_y, 'x', markersize=10, color=color, label=f'{robot_id.upper()} Target')
                        self.ax.plot([robot_data["x"], target_x], [robot_data["y"], target_y],
                                   color=color, linestyle='--', alpha=0.5)
                        mid_x = (robot_data["x"] + target_x) / 2
                        mid_y = (robot_data["y"] + target_y) / 2
                        self.ax.text(mid_x, mid_y, "Moving...", color=color,
                                   ha='center', va='center', bbox=dict(facecolor='white', alpha=0.7))
                    else:
                        self.ax.text(robot_data["x"] + 10, robot_data["y"] + 10, "Moving...", color=color,
                                   ha='center', va='center', bbox=dict(facecolor='white', alpha=0.7))
                status_text = f"{robot_id.upper()}: ({robot_data['x']:.1f}, {robot_data['y']:.1f}) cm, {robot_data['heading']:.1f}°"
                if robot_data["navigating"]:
                    status_text += " [Moving]"
                status_texts.append(status_text)
                if robot_data["path"]:
                    path_x, path_y = zip(*robot_data["path"])
                    self.ax.plot(path_x, path_y, color=color, alpha=0.3, linewidth=1)
            combined_status = '\n'.join(status_texts)
            self.ax.text(0.02, 0.98, combined_status, transform=self.ax.transAxes,
                        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        else:
            self.ax.plot(self.robot_x, self.robot_y, 'ro', markersize=12,
                       markerfacecolor='red', markeredgecolor='darkred',
                       markeredgewidth=2, label='Robot')
            arrow_length = min(15, (self.max_x - self.min_x) * 0.05)
            heading_rad = np.radians(self.robot_heading)
            head_x = self.robot_x + arrow_length * np.cos(heading_rad)
            head_y = self.robot_y + arrow_length * np.sin(heading_rad)
            self.ax.annotate('', xy=(head_x, head_y), xytext=(self.robot_x, self.robot_y),
                           arrowprops=dict(arrowstyle='->', color='red', lw=3))
            if self.robot_data[self.current_robot]["navigating"]:
                self.draw_safety_zones(self.robot_x, self.robot_y, self.robot_heading, 'red')
            if self.robot_data[self.current_robot]["navigating"]:
                if self.robot_data[self.current_robot]["navigation_target"]:
                    target_x, target_y = self.robot_data[self.current_robot]["navigation_target"]
                    self.ax.plot(target_x, target_y, 'rx', markersize=10, label='Target')
                    self.ax.plot([self.robot_x, target_x], [self.robot_y, target_y],
                               color='red', linestyle='--', alpha=0.5)
                    mid_x = (self.robot_x + target_x) / 2
                    mid_y = (self.robot_y + target_y) / 2
                    self.ax.text(mid_x, mid_y, "Moving...", color='red',
                               ha='center', va='center', bbox=dict(facecolor='white', alpha=0.7))
                else:
                    self.ax.text(self.robot_x + 10, self.robot_y + 10, "Moving...", color='red',
                               ha='center', va='center', bbox=dict(facecolor='white', alpha=0.7))
            status_text = f"{self.current_robot.upper()}: ({self.robot_x:.1f}, {self.robot_y:.1f}) cm\nHeading: {self.robot_heading:.1f}°"
            if self.robot_data[self.current_robot]["navigating"]:
                status_text += "\nMoving..."
            elif self.robot_heading != 0:
                status_text += f"\nMoving: {self.robot_heading:.1f}°"
            else:
                status_text += "\nStopped"
            self.ax.text(0.02, 0.98, status_text, transform=self.ax.transAxes,
                        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    def draw_safety_zones(self, x: float, y: float, heading: float, color: str):
        front_distance = 24.0
        right_distance = 18.0
        left_distance = 10.0
        back_distance = 5.0
        def draw_rotated_rectangle(heading_deg):
            heading_rad = math.radians(heading_deg)
            cos_h = math.cos(heading_rad)
            sin_h = math.sin(heading_rad)
            corners = [
                (x + front_distance * cos_h - right_distance * sin_h,
                 y + front_distance * sin_h + right_distance * cos_h),
                (x + front_distance * cos_h + left_distance * sin_h,
                 y + front_distance * sin_h - left_distance * cos_h),
                (x - back_distance * cos_h + left_distance * sin_h,
                 y - back_distance * sin_h - left_distance * cos_h),
                (x - back_distance * cos_h - right_distance * sin_h,
                 y - back_distance * sin_h + right_distance * cos_h)
            ]
            polygon = patches.Polygon(corners, closed=True, fill=True,
                                   facecolor=color, alpha=0.1,
                                   edgecolor=color, linestyle='--', linewidth=1)
            self.ax.add_patch(polygon)
        draw_rotated_rectangle(heading)
        draw_rotated_rectangle((heading + 45) % 360)
        draw_rotated_rectangle((heading - 45) % 360)
    def get_color(self, color_name):
        if ' ' in color_name:
            color_name = color_name.split()[0]
        color_map = {
            'red': '#FF0000',
            'green': '#00FF00',
            'blue': '#0000FF',
            'black': '#000000',
            'white': '#FFFFFF',
            'box': '#A0522D',
            'tape': '#800080',
            'crater': '#808080'
        }
        return color_map.get(color_name.lower(), '#808080')
    def draw_boxes(self):
        if not self.show_raw_detections[0]:
            return
        for box in self.detected_boxes:
            try:
                color = self.get_color(box.class_name)
                if box.world_polygon:
                    if self.debug_add_objects:
                        print(f"[DEBUG] draw_boxes: Drawing box polygon with {len(box.world_polygon)} points (class: {box.class_name})")
                    polygon_points = np.array(box.world_polygon)
                    polygon = patches.Polygon(polygon_points, closed=True,
                                           fill=True, facecolor=color,
                                           alpha=0.4, edgecolor='black', linewidth=2)
                    self.ax.add_patch(polygon)
                else:
                    if self.debug_add_objects:
                        print(f"[DEBUG] draw_boxes: Drawing box centerpoint at ({box.world_x}, {box.world_y}) (class: {box.class_name})")
                    size = 5
                    rect = patches.Rectangle(
                        (box.world_x - size/2, box.world_y - size/2),
                        size, size,
                        fill=True, facecolor=color,
                        alpha=0.4, edgecolor='black', linewidth=2
                    )
                    self.ax.add_patch(rect)
            except Exception as e:
                print(f"Error drawing box: {e}")
    def draw_tapes(self):
        for tape in self.detected_tapes:
            try:
                if 'world_polygon' in tape and len(tape['world_polygon']) >= 3:
                    classification = self.classify_tape_polygon(tape['world_polygon'])
                    color = self.get_tape_color(tape, classification)
                    polygon_points = np.array(tape['world_polygon'])
                    polygon = patches.Polygon(polygon_points, closed=True,
                                           fill=True, facecolor=color,
                                           alpha=0.4, edgecolor='black', linewidth=2)
                    self.ax.add_patch(polygon)
                    center = np.mean(polygon_points, axis=0)
            except Exception as e:
                print(f"Error drawing tape: {e}")
    def draw_imprints(self):
        for imprint in self.cube_imprints:
            try:
                color = imprint.get('color', 'gray')
                if imprint.get('world_polygon'):
                    polygon_points = np.array(imprint['world_polygon'])
                    polygon = patches.Polygon(polygon_points, closed=True,
                                           fill=True, facecolor=color,
                                           alpha=0.4, edgecolor='black', linewidth=2)
                    self.ax.add_patch(polygon)
                else:
                    size = 5
                    rect = patches.Rectangle(
                        (imprint['world_x'] - size/2, imprint['world_y'] - size/2),
                        size, size,
                        fill=True, facecolor=color,
                        alpha=0.4, edgecolor='black', linewidth=2
                    )
                    self.ax.add_patch(rect)
            except Exception as e:
                print(f"Error drawing cube imprint: {e}")
        for imprint in self.tape_imprints:
            try:
                if 'world_polygon' in imprint and len(imprint['world_polygon']) >= 3:
                    color = 'purple'
                    polygon_points = np.array(imprint['world_polygon'])
                    polygon = patches.Polygon(polygon_points, closed=True,
                                           fill=True, facecolor=color,
                                           alpha=0.4, edgecolor='black', linewidth=2)
                    self.ax.add_patch(polygon)
            except Exception as e:
                print(f"Error drawing tape imprint: {e}")
    def draw_camera_view(self):
        if self.show_both_robots:
            colors = {'robot1': ('red', 'darkred'), 'robot2': ('blue', 'darkblue')}
            for robot_id in self.available_robots:
                robot_camera_filter = self.robot_data[robot_id]["camera_filter"]
                color, _ = colors[robot_id]
                if robot_camera_filter.camera_bounds:
                    camera_polygon = patches.Polygon(
                        robot_camera_filter.camera_bounds,
                        closed=True,
                        fill=False,
                        edgecolor=color,
                        linestyle='--',
                        alpha=0.5,
                        label=f'{robot_id.upper()} Camera View'
                    )
                    self.ax.add_patch(camera_polygon)
                if hasattr(robot_camera_filter, 'margin_polygon_tape') and robot_camera_filter.margin_polygon_tape:
                    margin_polygon = patches.Polygon(
                        robot_camera_filter.margin_polygon_tape,
                        closed=True,
                        fill=False,
                        edgecolor='purple',
                        linestyle=':',
                        alpha=0.5,
                        label=f'{robot_id.upper()} Tape Margin' if robot_id == 'robot1' else None
                    )
                    self.ax.add_patch(margin_polygon)
                if hasattr(robot_camera_filter, 'margin_polygon_crater_cube') and robot_camera_filter.margin_polygon_crater_cube:
                    margin_polygon = patches.Polygon(
                        robot_camera_filter.margin_polygon_crater_cube,
                        closed=True,
                        fill=False,
                        edgecolor='black',
                        linestyle='-.',
                        alpha=0.5,
                        label=f'{robot_id.upper()} Crater/Cube Margin' if robot_id == 'robot1' else None
                    )
                    self.ax.add_patch(margin_polygon)
                if hasattr(robot_camera_filter, 'margin_polygon_box') and robot_camera_filter.margin_polygon_box:
                    margin_polygon = patches.Polygon(
                        robot_camera_filter.margin_polygon_box,
                        closed=True,
                        fill=False,
                        edgecolor='orange',
                        linestyle='-',
                        alpha=0.5,
                        label=f'{robot_id.upper()} Box Margin' if robot_id == 'robot1' else None
                    )
                    self.ax.add_patch(margin_polygon)
        else:
            if self.camera_filter.camera_bounds:
                camera_polygon = patches.Polygon(
                    self.camera_filter.camera_bounds,
                    closed=True,
                    fill=False,
                    edgecolor='blue',
                    linestyle='--',
                    alpha=0.5,
                    label='Camera View'
                )
                self.ax.add_patch(camera_polygon)
            if hasattr(self.camera_filter, 'margin_polygon_tape') and self.camera_filter.margin_polygon_tape:
                margin_polygon = patches.Polygon(
                    self.camera_filter.margin_polygon_tape,
                    closed=True,
                    fill=False,
                    edgecolor='purple',
                    linestyle=':',
                    alpha=0.7,
                    label='Tape Margin (6cm)'
                )
                self.ax.add_patch(margin_polygon)
            if hasattr(self.camera_filter, 'margin_polygon_crater_cube') and self.camera_filter.margin_polygon_crater_cube:
                margin_polygon = patches.Polygon(
                    self.camera_filter.margin_polygon_crater_cube,
                    closed=True,
                    fill=False,
                    edgecolor='black',
                    linestyle='-.',
                    alpha=0.7,
                    label='Crater/Cube Margin (2cm)'
                )
                self.ax.add_patch(margin_polygon)
            if hasattr(self.camera_filter, 'margin_polygon_box') and self.camera_filter.margin_polygon_box:
                margin_polygon = patches.Polygon(
                    self.camera_filter.margin_polygon_box,
                    closed=True,
                    fill=False,
                    edgecolor='orange',
                    linestyle='-',
                    alpha=0.7,
                    label='Box Margin (2cm, 70% area)'
                )
                self.ax.add_patch(margin_polygon)
    def draw_tracked_objects(self):
        for tape in self.tracked_objects['tapes']:
            try:
                polygon_points = np.array(tape.polygon)
                polygon = patches.Polygon(polygon_points, closed=True,
                                       fill=True, facecolor='purple',
                                       alpha=0.4, edgecolor='black', linewidth=2)
                self.ax.add_patch(polygon)
            except Exception as e:
                print(f"Error drawing tracked tape: {e}")
        for crater in self.tracked_objects['craters']:
            try:
                polygon_points = np.array(crater.polygon)
                polygon = patches.Polygon(polygon_points, closed=True,
                                       fill=True, facecolor='black',
                                       alpha=0.4, edgecolor='black', linewidth=2)
                self.ax.add_patch(polygon)
            except Exception as e:
                print(f"Error drawing tracked crater: {e}")
        for cube in self.tracked_objects['cubes']:
            try:
                color = self.get_color(getattr(cube, 'color', getattr(cube, 'class_name', 'gray')))
                center = cube.center
                size_class = getattr(cube, 'size_classification', 'medium')
                if size_class == 'big':
                    size = 16
                else:
                    size = 4
                self.ax.add_patch(patches.Rectangle((center[0] - size/2, center[1] - size/2), size, size, facecolor=color, edgecolor='black', alpha=0.8, linewidth=2))
                if hasattr(cube, 'avg_area'):
                    area = cube.avg_area
                    label_color = 'black' if color.lower() in ['#ffffff', 'white'] else color
            except Exception as e:
                print(f"Error drawing tracked cube: {e}")
        for box in self.tracked_objects['boxes']:
            try:
                color = self.get_color(getattr(box, 'color', 'box'))
                if self.debug_add_objects:
                    print(f"[DEBUG] draw_tracked_objects: Drawing tracked box centerpoint at {box.center} with color {color}")
                circle = patches.Circle((box.center[0], box.center[1]), 25, facecolor=color, edgecolor='black', alpha=0.3, linewidth=2)
                self.ax.add_patch(circle)
            except Exception as e:
                print(f"Error drawing tracked box: {e}")
    def animate(self, frame):
        with self.data_lock:
            self.ax.clear()
            self.ax.set_xlabel('X (cm)')
            self.ax.set_ylabel('Y (cm)')
            self.ax.grid(True)
            margin = 50
            if hasattr(self, 'min_x') and hasattr(self, 'max_x') and hasattr(self, 'min_y') and hasattr(self, 'max_y'):
                self.ax.set_xlim(self.max_x + margin, self.min_x - margin)
                self.ax.set_ylim(self.min_y - margin, self.max_y + margin)
            if self.show_minimal_mode[0]:
                self.draw_robot()
                self.draw_tracked_objects()
                return
            if self.show_both_robots:
                colors = {'robot1': 'red', 'robot2': 'blue'}
                for robot_id in self.available_robots:
                    robot_path = self.robot_data[robot_id]["path"]
                    if robot_path:
                        path_x, path_y = zip(*robot_path)
                        self.ax.plot(path_x, path_y, color=colors[robot_id], alpha=0.3, linewidth=1)
            else:
                if self.robot_path:
                    path_x, path_y = zip(*self.robot_path)
                    self.ax.plot(path_x, path_y, 'b-', alpha=0.3, linewidth=1)
            self.draw_camera_view()
            self.draw_robot()
            if self.show_raw_detections[0]:
                if self.show_both_robots:
                    for robot_id in self.available_robots:
                        self.draw_robot_raw_detections(robot_id)
                else:
                    self.draw_robot_raw_detections(self.current_robot)
            else:
                self.draw_tapes()
                self.draw_imprints()
                self.draw_tracked_objects()
                if self.show_both_robots:
                    for robot_id in self.available_robots:
                        robot_detected_boxes = self.robot_data[robot_id]["detected_boxes"]
                        robot_detected_tapes = self.robot_data[robot_id]["detected_tapes"]
                        for box in robot_detected_boxes:
                            try:
                                color = self.get_color(box.class_name)
                                if box.world_polygon:
                                    polygon_points = np.array(box.world_polygon)
                                    polygon = patches.Polygon(polygon_points, closed=True, fill=True, facecolor=color, alpha=0.4, edgecolor='black', linewidth=2)
                                    self.ax.add_patch(polygon)
                            except Exception as e:
                                print(f"Error drawing filtered box from {robot_id}: {e}")
                        for tape in robot_detected_tapes:
                            try:
                                if 'world_polygon' in tape and len(tape['world_polygon']) >= 3:
                                    classification = self.classify_tape_polygon(tape['world_polygon'])
                                    color = self.get_tape_color(tape, classification)
                                    polygon_points = np.array(tape['world_polygon'])
                                    polygon = patches.Polygon(polygon_points, closed=True, fill=True, facecolor=color, alpha=0.4, edgecolor='black', linewidth=2)
                                    self.ax.add_patch(polygon)
                            except Exception as e:
                                print(f"Error drawing filtered tape from {robot_id}: {e}")
                else:
                    for box in self.detected_boxes:
                        try:
                            color = self.get_color(box.class_name)
                            if box.world_polygon:
                                polygon_points = np.array(box.world_polygon)
                                polygon = patches.Polygon(polygon_points, closed=True, fill=True, facecolor=color, alpha=0.4, edgecolor='black', linewidth=2)
                                self.ax.add_patch(polygon)
                        except Exception as e:
                            print(f"Error drawing filtered box: {e}")
                    for tape in self.detected_tapes:
                        try:
                            if 'world_polygon' in tape and len(tape['world_polygon']) >= 3:
                                classification = self.classify_tape_polygon(tape['world_polygon'])
                                color = self.get_tape_color(tape, classification)
                                polygon_points = np.array(tape['world_polygon'])
                                polygon = patches.Polygon(polygon_points, closed=True, fill=True, facecolor=color, alpha=0.4, edgecolor='black', linewidth=2)
                                self.ax.add_patch(polygon)
                        except Exception as e:
                            print(f"Error drawing filtered tape: {e}")
            info_text = ""
            if self.robot_imprint_states[self.current_robot]["command_pending"]:
                elapsed = time.time() - self.robot_imprint_states[self.current_robot]["command_time"] if self.robot_imprint_states[self.current_robot]["command_time"] else 0
                info_text = f"ADDING OBJECTS... ({elapsed:.1f}s)"
            if info_text:
                self.ax.text(0.02, 0.02, info_text, transform=self.ax.transAxes, bbox=dict(facecolor='yellow', alpha=0.8))
            self.ax.legend(loc='upper right')
        now = time.time()
        toasts_to_keep = []
        for toast, t0 in self.ready_toasts:
            if now - t0 < 5.0:
                toasts_to_keep.append((toast, t0))
            else:
                toast.set_visible(False)
        self.ready_toasts = toasts_to_keep
    def start_visualization(self):
        ani = animation.FuncAnimation(self.fig, self.animate, interval=100)
        plt.show()
    def execute_turn(self, event):
        try:
            degrees = float(self.turn_degrees.text)
            command = {
                'command': 'turn',
                'angle_degrees': degrees
            }
            self.client.publish(f"{self.current_robot}/m", json.dumps(command))
            print(f"Turning {degrees}°")
        except ValueError:
            print("Invalid turn angle")
    def execute_move(self, event):
        try:
            distance = float(self.move_distance.text)
            command = {
                'command': 'move',
                'distance_cm': distance
            }
            self.client.publish(f"{self.current_robot}/m", json.dumps(command))
            print(f"Moving {distance}cm")
        except ValueError:
            print("Invalid move distance")
    def classify_tape_polygon(self, polygon_points):
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
    def get_tape_color(self, tape_data, classification=None):
        if classification == 'crater':
            return 'black'
        return 'purple'
    def add_current_objects(self, event):
        print("Adding current objects to tracked objects...")
        self.debug_add_objects = True
        if self.client:
            self.robot_imprint_states[self.current_robot]["command_pending"] = True
            self.robot_imprint_states[self.current_robot]["imprint_for_add_objects"] = True
            self.robot_imprint_states[self.current_robot]["command_time"] = time.time()
            imprint_command = {
                'command': 'imprint_scene',
                'timestamp': datetime.now().isoformat()
            }
            print("Imprint command sent to robot - waiting for response...")
    def add_current_objects_for_robot(self, event, robot_id):
        print(f"Adding current objects from {robot_id} to tracked objects...")
        self.debug_add_objects = True
        if self.client:
            self.robot_imprint_states[robot_id]["command_pending"] = True
            self.robot_imprint_states[robot_id]["imprint_for_add_objects"] = True
            self.robot_imprint_states[robot_id]["command_time"] = time.time()
            imprint_command = {
                'command': 'imprint_scene',
                'timestamp': datetime.now().isoformat()
            }
            self.client.publish(f"{robot_id}/m", json.dumps(imprint_command))
            print(f"Imprint command sent to {robot_id} - waiting for response...")
    def toggle_show_raw_detections(self, label):
        self.show_raw_detections[0] = not self.show_raw_detections[0]
        print(f"Show Raw Detections: {self.show_raw_detections[0]}")
    def clear_objects(self, event):
        self.object_tracker.tracked_tapes.clear()
        self.object_tracker.tracked_craters.clear()
        self.object_tracker.tracked_cubes.clear()
        self.object_tracker.tracked_boxes.clear()
        self.tracked_objects = self.object_tracker.get_all_tracked_objects()
        print("Cleared all tracked objects.")
    def load_robot_configurations(self):
        try:
            with open("robots_config.json", "r") as f:
                configs = json.load(f)
            robot_configs = {}
            for config in configs:
                robot_id = config["robot_id"]
                robot_configs[robot_id] = config
            return robot_configs
        except Exception as e:
            print(f"Warning: Could not load robot configurations: {e}")
            return {
                "robot1": {"default_position": {"x": 0.0, "y": 0.0, "heading": 0.0}},
                "robot2": {"default_position": {"x": 22.0, "y": 0.0, "heading": 180.0}}
            }
    def toggle_show_both_robots(self, label):
        self.show_both_robots = not self.show_both_robots
        print(f"Show Both Robots: {self.show_both_robots}")
        self.subscribe_to_robot_topics()
        self.update_current_robot_references()
        if self.show_both_robots:
            self.ax.set_title('Robot Map - BOTH ROBOTS')
        else:
            self.ax.set_title(f'Robot Map - {self.current_robot.upper()}')
        self.recreate_add_objects_buttons()
    def recreate_add_objects_buttons(self):
        if hasattr(self, 'add_objects_button'):
            self.add_objects_button.ax.remove()
            del self.add_objects_button
        if hasattr(self, 'add_objects_r1_button'):
            self.add_objects_r1_button.ax.remove()
            del self.add_objects_r1_button
        if hasattr(self, 'add_objects_r2_button'):
            self.add_objects_r2_button.ax.remove()
            del self.add_objects_r2_button
        button_height = 0.04
        button_width = 0.2
        left_margin = 0.01
        top_start = 0.95
        gap = 0.06
        if self.show_both_robots:
            add_objects_r1_button_ax = plt.axes([left_margin, top_start-gap, button_width/2-0.01, button_height])
            self.add_objects_r1_button = Button(add_objects_r1_button_ax, 'Add R1 Objects',
                                           color='lightgreen', hovercolor='green')
            self.add_objects_r1_button.on_clicked(lambda x: self.add_current_objects_for_robot(x, "robot1"))
            add_objects_r2_button_ax = plt.axes([left_margin + button_width/2, top_start-gap, button_width/2, button_height])
            self.add_objects_r2_button = Button(add_objects_r2_button_ax, 'Add R2 Objects',
                                           color='lightgreen', hovercolor='green')
            self.add_objects_r2_button.on_clicked(lambda x: self.add_current_objects_for_robot(x, "robot2"))
        else:
            add_objects_button_ax = plt.axes([left_margin, top_start-gap, button_width, button_height])
            self.add_objects_button = Button(add_objects_button_ax, 'Add Current Objects',
                                           color='lightgreen', hovercolor='green')
            self.add_objects_button.on_clicked(self.add_current_objects)
        self.fig.canvas.draw()
    def update_current_robot_references(self):
        self.robot_x = self.robot_data[self.current_robot]["x"]
        self.robot_y = self.robot_data[self.current_robot]["y"]
        self.robot_heading = self.robot_data[self.current_robot]["heading"]
        self.robot_path = self.robot_data[self.current_robot]["path"]
        self.detected_boxes = self.robot_data[self.current_robot]["detected_boxes"]
        self.detected_tapes = self.robot_data[self.current_robot]["detected_tapes"]
        self.raw_detected_boxes = self.robot_data[self.current_robot]["raw_detected_boxes"]
        self.raw_detected_tapes = self.robot_data[self.current_robot]["raw_detected_tapes"]
        self.last_update_time = self.robot_data[self.current_robot]["last_update_time"]
    def draw_robot_raw_detections(self, robot_id):
        raw_boxes = self.robot_data[robot_id]["raw_detected_boxes"]
        raw_tapes = self.robot_data[robot_id]["raw_detected_tapes"]
        for box in raw_boxes:
            try:
                color = self.get_color(box.class_name)
                if box.world_polygon:
                    polygon_points = np.array(box.world_polygon)
                    polygon = patches.Polygon(polygon_points, closed=True, fill=True, facecolor=color, alpha=0.4, edgecolor='black', linewidth=2)
                    self.ax.add_patch(polygon)
                else:
                    size = 5
                    rect = patches.Rectangle((box.world_x - size/2, box.world_y - size/2), size, size, fill=True, facecolor=color, alpha=0.4, edgecolor='black', linewidth=2)
                    self.ax.add_patch(rect)
            except Exception as e:
                print(f"Error drawing raw box for {robot_id}: {e}")
        for tape in raw_tapes:
            try:
                if 'world_polygon' in tape and len(tape['world_polygon']) >= 3:
                    classification = self.classify_tape_polygon(tape['world_polygon'])
                    color = self.get_tape_color(tape, classification)
                    polygon_points = np.array(tape['world_polygon'])
                    polygon = patches.Polygon(polygon_points, closed=True, fill=True, facecolor=color, alpha=0.4, edgecolor='black', linewidth=2)
                    self.ax.add_patch(polygon)
            except Exception as e:
                print(f"Error drawing raw tape for {robot_id}: {e}")
    def toggle_avoid_obstacles(self, label):
        self.avoid_obstacles = not self.avoid_obstacles
        print(f"Avoid Obstacles: {self.avoid_obstacles}")
    def stop_navigation(self, event):
        if self.client:
            command = {
                'command': 'stop_navigation'
            }
            self.client.publish(f"{self.current_robot}/navigation", json.dumps(command))
            print("Navigation stopped")
    def handle_navigation_status(self, status_data, robot_id):
        if robot_id not in self.robot_data:
            return
        command = status_data.get('command')
        movement_type = status_data.get('movement_type', 'navigate')
        if self.debug:
            print(f"[DEBUG] Navigation status for {robot_id}: {command} ({movement_type})")
        if command == 'navigation_start':
            with self.data_lock:
                self.robot_data[robot_id]["navigating"] = True
                if movement_type == 'navigate':
                    target_x = status_data.get('target_x')
                    target_y = status_data.get('target_y')
                    if target_x is not None and target_y is not None:
                        self.robot_data[robot_id]["navigation_target"] = (target_x, target_y)
            print(f"Started {movement_type} for {robot_id}")
        elif command == 'navigation_complete':
            with self.data_lock:
                self.robot_data[robot_id]["navigating"] = False
                if movement_type == 'navigate':
                    self.robot_data[robot_id]["navigation_target"] = None
                if 'final_x' in status_data and 'final_y' in status_data:
                    self.robot_data[robot_id]["x"] = status_data['final_x']
                    self.robot_data[robot_id]["y"] = status_data['final_y']
                if 'final_heading' in status_data:
                    self.robot_data[robot_id]["heading"] = status_data['final_heading']
            print(f"Completed {movement_type} for {robot_id}")
        elif command == 'navigation_error':
            with self.data_lock:
                self.robot_data[robot_id]["navigating"] = False
                if movement_type == 'navigate':
                    self.robot_data[robot_id]["navigation_target"] = None
            print(f"Failed {movement_type} for {robot_id}: {status_data.get('error_message', 'Unknown error')}")
    def start_autonomous_navigation(self, event):
        print("Starting autonomous navigation...")
        self.autonomous_nav.robot_id = self.current_robot
        threading.Thread(target=self.autonomous_nav.start_autonomous_navigation, daemon=True).start()
    def debug_detection_boxes(self, event):
        print(f"\n=== DEBUG DETECTION BOXES FOR {self.current_robot.upper()} ===")
        robot_x = self.robot_data[self.current_robot]["x"]
        robot_y = self.robot_data[self.current_robot]["y"]
        robot_heading = self.robot_data[self.current_robot]["heading"]
        print(f"Robot position: ({robot_x:.1f}, {robot_y:.1f}) cm, heading: {robot_heading:.1f}°")
        heading_rad = math.radians(robot_heading)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        front_box = [
            (12.0, -10.0),
            (12.0, 10.0),
            (0.0, 10.0),
            (0.0, -10.0)
        ]
        right_box = [
            (5.0, 0.0),
            (5.0, 14.0),
            (0.0, 14.0),
            (0.0, 0.0)
        ]
        left_box = [
            (5.0, -14.0),
            (5.0, 0.0),
            (0.0, 0.0),
            (0.0, -14.0)
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
        raw_boxes = self.robot_data[self.current_robot]["raw_detected_boxes"]
        raw_tapes = self.robot_data[self.current_robot]["raw_detected_tapes"]
        print(f"\nChecking RAW DETECTIONS: {len(raw_boxes)} boxes, {len(raw_tapes)} tapes")
        print(f"NOTE: Only using raw boxes for obstacle detection (raw tapes may be craters)")
        print(f"\n=== RAW BOX DETAILS ===")
        for i, obj in enumerate(raw_boxes):
            print(f"  Raw Box {i}: class={obj.class_name}, pos=({obj.world_x:.1f}, {obj.world_y:.1f})")
            if hasattr(obj, 'world_polygon') and obj.world_polygon:
                print(f"    Has polygon with {len(obj.world_polygon)} points")
            else:
                print(f"    No polygon")
        for i, obj in enumerate(raw_boxes):
            box_results = check_object_in_boxes(obj)
            print(f"  Raw Box {i} ({obj.class_name}) at ({obj.world_x:.1f}, {obj.world_y:.1f}): {box_results}")
            dx = obj.world_x - robot_x
            dy = obj.world_y - robot_y
            rel_x = dx * cos_h + dy * sin_h
            rel_y = -dx * sin_h + dy * cos_h
            print(f"    Robot-relative: ({rel_x:.1f}, {rel_y:.1f})")
            print(f"    Front box: x=0-12, y=-10 to 10")
            print(f"    In front box: {0 <= rel_x <= 12 and -10 <= rel_y <= 10}")
            if any(box_results.values()):
                if box_results['front']:
                    front_objects.append(f"Raw Box {i} ({obj.class_name})")
                if box_results['right']:
                    right_objects.append(f"Raw Box {i} ({obj.class_name})")
                if box_results['left']:
                    left_objects.append(f"Raw Box {i} ({obj.class_name})")
        print(f"\n=== SKIPPING RAW TAPES (may be craters) ===")
        for i, tape in enumerate(raw_tapes):
            print(f"  Raw Tape {i}: skipping for obstacle detection")
        print(f"\n=== RAW DETECTION BOX SUMMARY ===")
        if front_objects:
            print(f"FRONT BOX ({len(front_objects)} objects): {', '.join(front_objects)}")
        else:
            print("FRONT BOX: No obstacles detected")
        if right_objects:
            print(f"RIGHT BOX ({len(right_objects)} objects): {', '.join(right_objects)}")
        else:
            print("RIGHT BOX: No obstacles detected")
        if left_objects:
            print(f"LEFT BOX ({len(left_objects)} objects): {', '.join(left_objects)}")
        else:
            print("LEFT BOX: No obstacles detected")
        if front_objects:
            print(f"\n🚨 OBSTACLE DETECTED IN FRONT! Robot would need to avoid {len(front_objects)} objects")
        else:
            print(f"\n✅ FRONT CLEAR - Robot can move forward safely")
        print("=" * 50)
    def debug_general_box(self, event):
        print(f"\n=== DEBUG GENERAL BOX FUNCTION ===")
        result = self.autonomous_nav.check_objects_in_box()
        print(result)
        print(f"=== END DEBUG GENERAL BOX ===\n")
    def debug_tape_approach_rects(self, event):
        print(f"\n=== DEBUG TAPE APPROACH RECTANGLES FOR {self.current_robot.upper()} ===")
        result = self.autonomous_nav.check_tape_in_approach_rectangles()
        for rect in ['rect1', 'rect2', 'rect3']:
            tapes = result.get(rect, [])
            print(f"{rect}: {len(tapes)} tape(s)")
            for i, tape in enumerate(tapes):
                if hasattr(tape, 'polygon'):
                    print(f"  Tape {i}: polygon with {len(tape.polygon)} points")
                elif hasattr(tape, 'center'):
                    print(f"  Tape {i}: center at {tape.center}")
                else:
                    print(f"  Tape {i}: unknown format")
        print(f"=== END DEBUG TAPE APPROACH RECTANGLES ===\n")
    def start_both_autonomous_navigation(self, event):
        print("Starting autonomous navigation for BOTH robots...")
        import threading
        from autonomous_navigation import AutonomousNavigation
        nav1 = AutonomousNavigation(self.client, "robot1", self)
        threading.Thread(target=nav1.start_autonomous_navigation, daemon=True).start()
        nav2 = AutonomousNavigation(self.client, "robot2", self)
        threading.Thread(target=nav2.start_autonomous_navigation, daemon=True).start()
    def toggle_minimal_mode(self, label):
        self.show_minimal_mode[0] = not self.show_minimal_mode[0]
        print(f"Minimal Mode: {self.show_minimal_mode[0]}")
    def toggle_independant_mode(self, label):
        self.independant_mode[0] = not self.independant_mode[0]
        print(f"Independant Mode: {self.independant_mode[0]}")
    def set_filter_mode(self, mode):
        self.filter_mode = mode
        self.update_filter_button_colors()
        for robot_id in self.available_robots:
            if mode == 1:
                self.robot_data[robot_id]["camera_filter"] = CameraViewFilter(
                    margin_tape=0.0,
                    margin_crater=2.0,
                    margin_cube=2.0,
                    margin_box=2.0
                )
            elif mode == 2:
                self.robot_data[robot_id]["camera_filter"] = CameraViewFilter()
        filter_names = ["No Filter", "Simple Filter", "Full Filter"]
        print(f"Filter Mode: {filter_names[mode]}")
    def update_filter_button_colors(self):
        self.no_filter_button.color = 'lightgray' if self.filter_mode != 0 else 'lightgreen'
        self.simple_filter_button.color = 'lightgray' if self.filter_mode != 1 else 'lightyellow'
        self.full_filter_button.color = 'lightgray' if self.filter_mode != 2 else 'lightcoral'
        self.fig.canvas.draw_idle()
    def add_tracked_objects_legend(self):
        import matplotlib.patches as mpatches
        legend_handles = []
        box_patch = mpatches.Circle((0,0), 25, facecolor=self.get_color('box'), edgecolor='black', alpha=0.3, linewidth=2, label='Tracked Box (50cm diameter)')
        legend_handles.append(box_patch)
        big_cube_patch = mpatches.Rectangle((0,0), 16, 16, facecolor=self.get_color('red'), edgecolor='black', alpha=0.8, linewidth=2, label='Big Cube (16cm)')
        legend_handles.append(big_cube_patch)
        small_cube_patch = mpatches.Rectangle((0,0), 4, 4, facecolor=self.get_color('blue'), edgecolor='black', alpha=0.8, linewidth=2, label='Small Cube (4cm)')
        legend_handles.append(small_cube_patch)
        self.ax.legend(handles=legend_handles, loc='upper right')
    def send_autonav2_command(self, event):
        print("Sending autonav2 command to BOTH robots...")
        if self.client:
            command = {
                'command': 'autonav2',
            }
            for robot_id in self.available_robots:
                self.client.publish(f"{robot_id}/m", json.dumps(command))
                print(f"AutoNav2 command sent to {robot_id}/m")
        else:
            print("MQTT not available - cannot send autonav2 command")
    def show_ready_toast(self, robot_id):
        y_start = 0.95
        y_gap = 0.06
        now = time.time()
        self.ready_toasts = [(t, t0) for t, t0 in self.ready_toasts if t.get_visible()]
        for idx, (toast, t0) in enumerate(self.ready_toasts):
            toast.set_position((0.5, y_start - y_gap * (idx + 1)))
        toast_text = self.fig.text(0.5, y_start, f"{robot_id.upper()} is READY!", color='green', fontsize=18, ha='center', va='top', bbox=dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.5'))
        self.fig.canvas.draw_idle()
        self.ready_toasts.insert(0, (toast_text, now))
    def send_autonav3_command(self, event):
        print("Sending autonav3 command to BOTH robots...")
        if self.client:
            command = {
                'command': 'autonav3',
            }
            for robot_id in self.available_robots:
                self.client.publish(f"{robot_id}/m", json.dumps(command))
                print(f"AutoNav3 command sent to {robot_id}/m")
        else:
            print("MQTT not available - cannot send autonav3 command")
def get_robot_phone_type(robot_id, robots_config_path="robots_config.json"):
    with open(robots_config_path, "r") as f:
        configs = json.load(f)
    for config in configs:
        if config.get("robot_id") == robot_id:
            return config.get("phone_type", "default")
    return "default"
def calculate_camera_bounds(phone_type):
    config = PHONE_CONFIGS.get(phone_type)
    if not config:
        print(f"Unknown phone type: {phone_type}")
        return None
    image_width = config["image_width"]
    image_height = config["image_height"]
    src_pts = config["calibration_points"]["src_pts_pixels"]
    dst_pts = config["calibration_points"]["dst_pts_realworld"]
    offset_x_inches = config["offset"].get("x_inches", 0.0)
    offset_y_inches = config["offset"].get("y_inches", 0.0)
    H, _ = cv2.findHomography(src_pts, dst_pts)
    if H is None:
        print(f"Could not compute homography for {phone_type}")
        return None
    sample_interval = 100
    boundary_points = []
    for x in range(0, image_width + 1, sample_interval):
        boundary_points.append([x, 0])
    for y in range(0, image_height + 1, sample_interval):
        boundary_points.append([image_width, y])
    for x in range(image_width, -1, -sample_interval):
        boundary_points.append([x, image_height])
    for y in range(image_height, -1, -sample_interval):
        boundary_points.append([0, y])
    boundary_points = np.array(boundary_points, dtype=np.float32)
    valid_points = []
    for point in boundary_points:
        try:
            transformed_point = cv2.perspectiveTransform(
                point.reshape(-1, 1, 2),
                H
            ).reshape(-1, 2)[0]
            if not np.isnan(transformed_point[0]) and not np.isnan(transformed_point[1]):
                x_cm = (transformed_point[0] + offset_x_inches) * INCHES_TO_CM
                y_cm = (transformed_point[1] + offset_y_inches) * INCHES_TO_CM
                valid_points.append([x_cm, y_cm])
        except Exception as e:
            print(f"Failed to transform point {point}: {e}")
    if len(valid_points) >= 3:
        points_array = np.array(valid_points)
        hull = cv2.convexHull(points_array.astype(np.float32))
        hull_points = hull.reshape(-1, 2)
        return hull_points.tolist()
    else:
        print("Not enough valid points for camera view polygon")
        return None
def get_camera_view_polygon_robot_frame(phone_type):
    config = PHONE_CONFIGS.get(phone_type)
    if not config:
        print(f"Unknown phone type: {phone_type}")
        return None
    image_width = config["image_width"]
    image_height = config["image_height"]
    src_pts = config["calibration_points"]["src_pts_pixels"]
    dst_pts = config["calibration_points"]["dst_pts_realworld"]
    offset_x_inches = config["offset"].get("x_inches", 0.0)
    offset_y_inches = config["offset"].get("y_inches", 0.0)
    H, _ = cv2.findHomography(src_pts, dst_pts)
    if H is None:
        print(f"Could not compute homography for {phone_type}")
        return None
    sample_interval = 100
    boundary_points = []
    for x in range(0, image_width + 1, sample_interval):
        boundary_points.append([x, 0])
    for y in range(0, image_height + 1, sample_interval):
        boundary_points.append([image_width, y])
    for x in range(image_width, -1, -sample_interval):
        boundary_points.append([x, image_height])
    for y in range(image_height, -1, -sample_interval):
        boundary_points.append([0, y])
    boundary_points = np.array(boundary_points, dtype=np.float32)
    valid_points = []
    for point in boundary_points:
        try:
            transformed_point = cv2.perspectiveTransform(
                point.reshape(-1, 1, 2),
                H
            ).reshape(-1, 2)[0]
            if not np.isnan(transformed_point[0]) and not np.isnan(transformed_point[1]):
                x_cm = (transformed_point[0] + offset_x_inches) * INCHES_TO_CM
                y_cm = (transformed_point[1] + offset_y_inches) * INCHES_TO_CM
                valid_points.append([x_cm, y_cm])
        except Exception as e:
            print(f"Failed to transform point {point}: {e}")
    if len(valid_points) >= 3:
        points_array = np.array(valid_points)
        hull = cv2.convexHull(points_array.astype(np.float32))
        hull_points = hull.reshape(-1, 2)
        return hull_points.tolist()
    else:
        print("Not enough valid points for camera view polygon")
        return None
def transform_camera_polygon_to_world(robot_x, robot_y, robot_heading_deg, camera_polygon):
    heading_rad = math.radians(robot_heading_deg)
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)
    world_points = []
    for point in camera_polygon:
        cam_x, cam_y = point
        world_x = robot_x + (cam_y * cos_h - cam_x * sin_h)
        world_y = robot_y + (cam_y * sin_h + cam_x * cos_h)
        world_points.append([world_x, world_y])
    return world_points if len(world_points) >= 3 else None
def decode_polyline_to_points(polyline_str):
    if not polyline_str:
        return []
    coords = polyline.decode(polyline_str, precision=2)
    return [[y, x] for x, y in coords]
def main():
    import argparse
    parser = argparse.ArgumentParser(description='Simple Robot Map Visualizer')
    parser.add_argument('--broker', default='100.69.37.102', help='MQTT broker address')
    parser.add_argument('--port', type=int, default=1883, help='MQTT broker port')
    args = parser.parse_args()
    visualizer = SimpleMapVisualizer(mqtt_broker=args.broker, mqtt_port=args.port)
    visualizer.start_visualization()
if __name__ == "__main__":
    main()