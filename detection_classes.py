import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any
import json
@dataclass
class Box:
    cam_x: float
    cam_y: float
    cam_polygon: List[List[float]]
    world_x: float
    world_y: float
    world_polygon: List[List[float]]
    class_name: str
    color: str
    size_classification: str
    area_sq_cm: float
    confidence: float
    def __init__(self, cam_x, cam_y, world_x, world_y, class_name="unknown", color="unknown",
                 size_classification="unknown", area_sq_cm=0, confidence=0, world_polygon=None):
        self.cam_x = cam_x
        self.cam_y = cam_y
        self.world_x = world_x
        self.world_y = world_y
        self.class_name = class_name
        self.color = color
        self.size_classification = size_classification
        self.area_sq_cm = area_sq_cm
        self.confidence = confidence
        self.world_polygon = world_polygon
    @classmethod
    def from_detection_data(cls, detection_data, robot_x, robot_y, robot_heading):
        try:
            if 'center_realworld_cm' in detection_data:
                cam_x, cam_y = detection_data['center_realworld_cm']
                from main_integration import transform_point_to_world
                world_x, world_y = transform_point_to_world(robot_x, robot_y, robot_heading, cam_x, cam_y)
            elif 'center_realworld_xy' in detection_data:
                cam_x, cam_y = detection_data['center_realworld_xy']
                world_x, world_y = transform_point_to_world(robot_x, robot_y, robot_heading, cam_x, cam_y)
            elif 'world_x' in detection_data and 'world_y' in detection_data:
                world_x = detection_data['world_x']
                world_y = detection_data['world_y']
                cam_x = detection_data.get('cam_x', 0)
                cam_y = detection_data.get('cam_y', 0)
            else:
                raise ValueError("No valid coordinates found in detection data")
            class_name = detection_data.get('class', 'unknown')
            color = detection_data.get('color', 'gray')
            if not color and ' ' in class_name:
                color = class_name.split()[0]
            size_classification = detection_data.get('size_classification', 'unknown')
            area_sq_cm = detection_data.get('area_sq_cm', 0)
            confidence = detection_data.get('confidence', 0)
            world_polygon = None
            if 'bbox_polygon_realworld_cm' in detection_data:
                world_polygon = []
                for point in detection_data['bbox_polygon_realworld_cm']:
                    if len(point) >= 2:
                        world_point_x, world_point_y = transform_point_to_world(
                            robot_x, robot_y, robot_heading, point[0], point[1]
                        )
                        world_polygon.append([world_point_x, world_point_y])
            elif 'world_polygon' in detection_data:
                world_polygon = detection_data['world_polygon']
            return cls(
                cam_x=cam_x,
                cam_y=cam_y,
                world_x=world_x,
                world_y=world_y,
                class_name=class_name,
                color=color,
                size_classification=size_classification,
                area_sq_cm=area_sq_cm,
                confidence=confidence,
                world_polygon=world_polygon
            )
        except Exception as e:
            print(f"Error creating Box from detection data: {e}")
            raise
    def to_dict(self) -> Dict[str, Any]:
        return {
            'world_x': self.world_x,
            'world_y': self.world_y,
            'cam_x': self.cam_x,
            'cam_y': self.cam_y,
            'class': self.class_name,
            'color': self.color,
            'size_classification': self.size_classification,
            'area_sq_cm': self.area_sq_cm,
            'confidence': self.confidence,
            'world_polygon': self.world_polygon
        }
@dataclass
class Robot:
    x: float
    y: float
    heading: float
    moving: bool = False
    current_direction: Optional[str] = None
    def to_dict(self) -> Dict[str, Any]:
        return {
            'x': self.x,
            'y': self.y,
            'heading': self.heading,
            'moving': self.moving,
            'current_direction': self.current_direction,
            'timestamp': None
        }
    def update_pose(self, x: float, y: float, heading: float):
        self.x = x
        self.y = y
        self.heading = heading
    def update_movement(self, moving: bool, direction: Optional[str] = None):
        self.moving = moving
        self.current_direction = direction