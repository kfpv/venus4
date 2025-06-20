from typing import List, Dict, Tuple, Optional
from shapely.geometry import Polygon, Point
import numpy as np
from dataclasses import dataclass
from datetime import datetime
from shapely.ops import unary_union
@dataclass
class TrackedObject:
    id: str
    center: Tuple[float, float]
    polygon: List[Tuple[float, float]]
    last_seen: datetime
    confidence: float = 1.0
    color: str = "unknown"
    size_classification: str = "medium"
    observed_polygons: List[Tuple[List[Tuple[float, float]], float, Tuple[float, float]]] = None
    avg_area: float = 0.0
class TrackedBox:
    def __init__(self, id: str, center: Tuple[float, float], last_seen: datetime):
        self.id = id
        self.center = center
        self.last_seen = last_seen
class ObjectTracker:
    BIG_CUBE_AREA_THRESHOLD = 80
    def __init__(self, overlap_threshold: float = 0.01):
        self.overlap_threshold = overlap_threshold * 2
        self.tracked_tapes: List[TrackedObject] = []
        self.tracked_craters: List[TrackedObject] = []
        self.tracked_cubes: Dict[str, List[TrackedObject]] = {}
        self.tracked_boxes: List[TrackedBox] = []
        self.next_id = 0
    def _get_new_id(self) -> str:
        id = f"obj_{self.next_id}"
        self.next_id += 1
        return id
    def _calculate_overlap(self, poly1: Polygon, poly2: Polygon) -> float:
        if poly1.is_empty or poly2.is_empty:
            return 0.0
        intersection = poly1.intersection(poly2).area
        union = poly1.union(poly2).area
        return intersection / union if union > 0 else 0.0
    def _average_polygons(self, polygons: List[List[Tuple[float, float]]]) -> List[Tuple[float, float]]:
        if not polygons:
            return []
        try:
            poly_arrays = [np.array(poly) for poly in polygons]
            centers = [np.mean(poly, axis=0) for poly in poly_arrays]
            avg_center = np.mean(centers, axis=0)
            base_poly = poly_arrays[0]
            result_points = []
            for i in range(len(base_poly)):
                point = base_poly[i]
                relative_points = []
                for j, poly in enumerate(poly_arrays):
                    distances = np.linalg.norm(poly - point, axis=1)
                    closest_idx = np.argmin(distances)
                    relative_points.append(poly[closest_idx] - centers[j])
                avg_relative = np.mean(relative_points, axis=0)
                result_points.append(tuple(avg_center + avg_relative))
            return result_points
        except Exception as e:
            print(f"Error averaging polygons: {e}")
            return max(polygons, key=lambda p: Polygon(p).area)
    def _average_centers(self, centers: List[Tuple[float, float]]) -> Tuple[float, float]:
        if not centers:
            return (0.0, 0.0)
        return tuple(np.mean(centers, axis=0))
    def add_tapes(self, tapes: List[Dict]):
        for tape in tapes:
            if 'world_polygon' not in tape or len(tape['world_polygon']) < 3:
                continue
            try:
                tape_poly = Polygon(tape['world_polygon'])
                if not tape_poly.is_valid:
                    tape_poly = tape_poly.buffer(0).simplify(0.1)
                if tape_poly.is_empty:
                    continue
                center = tuple(np.mean(tape['world_polygon'], axis=0))
                merged = False
                for tracked in self.tracked_tapes:
                    try:
                        tracked_poly = Polygon(tracked.polygon)
                        if not tracked_poly.is_valid:
                            tracked_poly = tracked_poly.buffer(0).simplify(0.1)
                        if tracked_poly.is_empty:
                            continue
                        if self._calculate_overlap(tape_poly, tracked_poly) > self.overlap_threshold:
                            union_poly = unary_union([tracked_poly, tape_poly])
                            if union_poly.geom_type == 'Polygon':
                                tracked.polygon = list(union_poly.exterior.coords)
                            elif union_poly.geom_type == 'MultiPolygon':
                                largest = max(union_poly.geoms, key=lambda p: p.area)
                                tracked.polygon = list(largest.exterior.coords)
                            tracked.center = self._average_centers([tracked.center, center])
                            tracked.last_seen = datetime.now()
                            merged = True
                            break
                    except Exception as e:
                        print(f"Error processing tracked tape: {e}")
                        continue
                if not merged:
                    self.tracked_tapes.append(TrackedObject(
                        id=self._get_new_id(),
                        center=center,
                        polygon=tape['world_polygon'],
                        last_seen=datetime.now()
                    ))
            except Exception as e:
                print(f"Error processing tape: {e}")
                continue
    def add_craters(self, craters: List[Dict]):
        for crater in craters:
            if 'world_polygon' not in crater or len(crater['world_polygon']) < 3:
                continue
            crater_poly = Polygon(crater['world_polygon'])
            center = tuple(np.mean(crater['world_polygon'], axis=0))
            merged = False
            for tracked in self.tracked_craters:
                tracked_poly = Polygon(tracked.polygon)
                if self._calculate_overlap(crater_poly, tracked_poly) > self.overlap_threshold:
                    tracked.polygon = self._average_polygons([tracked.polygon, crater['world_polygon']])
                    tracked.center = self._average_centers([tracked.center, center])
                    tracked.last_seen = datetime.now()
                    merged = True
                    break
            if not merged:
                self.tracked_craters.append(TrackedObject(
                    id=self._get_new_id(),
                    center=center,
                    polygon=crater['world_polygon'],
                    last_seen=datetime.now()
                ))
    def add_cubes(self, cubes: List[Dict]):
        for cube in cubes:
            if 'world_polygon' not in cube or len(cube['world_polygon']) < 3:
                continue
            cube_poly = Polygon(cube['world_polygon'])
            center = tuple(np.mean(cube['world_polygon'], axis=0))
            color = cube.get('color', cube.get('class', 'unknown')).lower()
            area = cube_poly.area
            if color not in self.tracked_cubes:
                self.tracked_cubes[color] = []
            merged = False
            for tracked in self.tracked_cubes[color]:
                tracked_poly = Polygon(tracked.polygon)
                if self._calculate_overlap(cube_poly, tracked_poly) > self.overlap_threshold:
                    if tracked.observed_polygons is None:
                        tracked.observed_polygons = [(tracked.polygon, Polygon(tracked.polygon).area, tracked.center)]
                    tracked.observed_polygons.append((cube['world_polygon'], area, center))
                    all_centers = [c for _, _, c in tracked.observed_polygons]
                    tracked.center = self._average_centers(all_centers)
                    all_areas = [a for _, a, _ in tracked.observed_polygons]
                    tracked.avg_area = float(np.mean(all_areas))
                    tracked.polygon = cube['world_polygon']
                    tracked.last_seen = datetime.now()
                    merged = True
                    break
            if not merged:
                tracked_obj = TrackedObject(
                    id=self._get_new_id(),
                    center=center,
                    polygon=cube['world_polygon'],
                    last_seen=datetime.now(),
                    color=color,
                    confidence=cube.get('confidence', 1.0),
                    size_classification='medium',
                    observed_polygons=[(cube['world_polygon'], area, center)],
                    avg_area=area
                )
                self.tracked_cubes[color].append(tracked_obj)
        for cubes in self.tracked_cubes.values():
            for tracked in cubes:
                if hasattr(tracked, 'avg_area'):
                    if tracked.avg_area > self.BIG_CUBE_AREA_THRESHOLD:
                        tracked.size_classification = 'big'
                    else:
                        tracked.size_classification = 'small'
    def add_boxes(self, boxes: List[Dict]):
        for box in boxes:
            if 'world_polygon' not in box or len(box['world_polygon']) < 3:
                continue
            polygon = np.array(box['world_polygon'])
            color = box.get('color', box.get('class', 'box')).lower()
            robot_x = box.get('robot_x', 0)
            robot_y = box.get('robot_y', 0)
            robot_heading = box.get('robot_heading', 0)
            centroid = np.mean(polygon, axis=0)
            theta = -np.deg2rad(robot_heading)
            R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
            local_points = np.dot(polygon - np.array([robot_x, robot_y]), R.T)
            min_local_y = float('inf')
            chosen_mid = None
            for i in range(len(local_points)):
                lp1 = local_points[i]
                lp2 = local_points[(i+1)%len(local_points)]
                mid_local = (lp1 + lp2) / 2
                if mid_local[1] < min_local_y:
                    min_local_y = mid_local[1]
                    wp1 = polygon[i]
                    wp2 = polygon[(i+1)%len(polygon)]
                    chosen_mid = (wp1 + wp2) / 2
            if chosen_mid is not None:
                direction = centroid - chosen_mid
                norm = np.linalg.norm(direction)
                if norm > 0:
                    direction = direction / norm
                else:
                    direction = np.array([0.0, 0.0])
                center = chosen_mid + direction * 5.0
            else:
                center = tuple(centroid)
            merged = False
            for tracked in self.tracked_boxes:
                distance = np.linalg.norm(np.array(tracked.center) - np.array(center))
                if distance < 40.0:
                    tracked.center = tuple(np.mean([tracked.center, center], axis=0))
                    tracked.last_seen = datetime.now()
                    tracked.color = color
                    merged = True
                    break
            if not merged:
                tracked_box = TrackedBox(
                    id=self._get_new_id(),
                    center=center,
                    last_seen=datetime.now()
                )
                tracked_box.color = color
                self.tracked_boxes.append(tracked_box)
    def get_all_tracked_objects(self) -> Dict:
        return {
            'tapes': self.tracked_tapes,
            'craters': self.tracked_craters,
            'cubes': [cube for cubes in self.tracked_cubes.values() for cube in cubes],
            'boxes': self.tracked_boxes
        }