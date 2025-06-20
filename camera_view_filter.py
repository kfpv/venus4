import numpy as np
from typing import List, Tuple, Dict, Optional
import math
from shapely.geometry import Polygon, Point
import pyclipper
class CameraViewFilter:
    def __init__(self, margin_tape: float = 8.0, margin_crater: float = 2.0, margin_cube: float = 2.0, margin_box: float = 2.0):
        self.margin_tape = margin_tape
        self.margin_crater = margin_crater
        self.margin_cube = margin_cube
        self.margin_box = margin_box
        self.camera_bounds = None
        self.margin_polygon_tape = None
        self.margin_polygon_crater_cube = None
        self.margin_polygon_box = None
    def set_camera_bounds(self, bounds: List[Tuple[float, float]]):
        if len(bounds) < 3:
            raise ValueError("Camera bounds must have at least 3 points")
        self.camera_bounds = bounds
        scale = 1e6
        polygon_scaled = [(int(x * scale), int(y * scale)) for x, y in bounds]
        pc_tape = pyclipper.PyclipperOffset()
        pc_tape.AddPath(polygon_scaled, pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)
        solution_tape = pc_tape.Execute(-self.margin_tape * scale)
        self.margin_polygon_tape = [(pt[0] / scale, pt[1] / scale) for pt in solution_tape[0]] if solution_tape and len(solution_tape[0]) >= 3 else None
        pc_crater_cube = pyclipper.PyclipperOffset()
        pc_crater_cube.AddPath(polygon_scaled, pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)
        solution_crater_cube = pc_crater_cube.Execute(-self.margin_crater * scale)
        self.margin_polygon_crater_cube = [(pt[0] / scale, pt[1] / scale) for pt in solution_crater_cube[0]] if solution_crater_cube and len(solution_crater_cube[0]) >= 3 else None
        self.margin_polygon_box = self.margin_polygon_crater_cube
    def scale_polygon(self, polygon: List[Tuple[float, float]], scale: float) -> List[Tuple[float, float]]:
        poly_np = np.array(polygon)
        centroid = poly_np.mean(axis=0)
        scaled = centroid + (poly_np - centroid) * scale
        return [tuple(pt) for pt in scaled]
    def _point_in_polygon(self, point: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
        return Polygon(polygon).contains(Point(point))
    def _clip_polygon_to_bounds(self, polygon: List[Tuple[float, float]], margin_polygon: List[Tuple[float, float]]) -> Optional[List[Tuple[float, float]]]:
        if not margin_polygon:
            return polygon
        poly = Polygon(polygon)
        if not poly.is_valid:
                print(f"[DEBUG] Input polygon is invalid")
                return None
        margin = Polygon(margin_polygon)
        if not margin.is_valid:
                print(f"[DEBUG] Margin polygon is invalid")
                return None
        intersection = poly.intersection(margin)
        if intersection.is_empty:
                print("[DEBUG] Intersection is empty")
                return None
        if intersection.geom_type == 'Polygon':
                result = list(intersection.exterior.coords[:-1])
                print(f"[DEBUG] Intersection successful, result has {len(result)} points")
                return result
        else:
                print(f"[DEBUG] Intersection resulted in non-polygon type: {intersection.geom_type}")
                return None
    def filter_cube(self, cube_data: Dict) -> Optional[Dict]:
        if not self.margin_polygon_crater_cube:
            return cube_data
        if 'world_polygon' in cube_data and cube_data['world_polygon']:
            cube_poly = Polygon(cube_data['world_polygon'])
            margin_poly = Polygon(self.margin_polygon_crater_cube)
            if not cube_poly.within(margin_poly):
                return None
            if cube_poly.crosses(margin_poly) or cube_poly.touches(margin_poly):
                return None
        return cube_data
    def filter_tape(self, tape_data: Dict) -> Optional[Dict]:
        if not self.margin_polygon_tape or 'world_polygon' not in tape_data:
            print(f"[DEBUG] No margin polygon or world polygon in tape data: {tape_data}")
            return tape_data
        polygon = [(p[0], p[1]) for p in tape_data['world_polygon']]
        if len(polygon) != len(set(polygon)):
                print("[DEBUG] Tape has duplicate points, removing duplicates")
                polygon = list(dict.fromkeys(polygon))
        if len(polygon) < 3:
                print("[DEBUG] Tape has less than 3 points after deduplication")
                return None
        filtered_polygon = self._clip_polygon_to_bounds(polygon, self.margin_polygon_tape)
        if filtered_polygon is None:
            return None
        filtered_tape = tape_data.copy()
        filtered_tape['world_polygon'] = [[p[0], p[1]] for p in filtered_polygon]
        return filtered_tape
    def filter_crater(self, crater_data: Dict) -> Optional[Dict]:
        if not self.margin_polygon_crater_cube:
            return crater_data
        if 'world_polygon' in crater_data and crater_data['world_polygon']:
            crater_poly = Polygon(crater_data['world_polygon'])
            if not crater_poly.is_valid:
                print(f"[DEBUG] Crater polygon is invalid")
                return None
            margin_poly = Polygon(self.margin_polygon_crater_cube)
            if not margin_poly.is_valid:
                print(f"[DEBUG] Margin polygon is invalid")
                return None
            if not crater_poly.within(margin_poly):
                return None
            if crater_poly.crosses(margin_poly) or crater_poly.touches(margin_poly):
                return None
        return crater_data
    def filter_box(self, box_data: Dict) -> Optional[Dict]:
        if not self.margin_polygon_box:
            return box_data
        if 'world_polygon' in box_data and box_data['world_polygon']:
            box_poly = Polygon(box_data['world_polygon'])
            margin_poly = Polygon(self.margin_polygon_box)
            intersection = box_poly.intersection(margin_poly)
            if intersection.is_empty:
                return None
            area_inside = intersection.area
            area_total = box_poly.area
            ratio = area_inside / area_total if area_total > 0 else 0
            if area_total == 0 or ratio < 0.7:
                return None
        return box_data