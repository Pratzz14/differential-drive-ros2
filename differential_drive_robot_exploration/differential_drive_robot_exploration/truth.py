"""Gazebo truth parsing, alignment, and occupancy-map scoring."""
# flake8: noqa

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseArray

from .frontier import GridSpec, OCCUPIED_MIN, UNKNOWN


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    width: float
    depth: float
    yaw: float = 0.0
    name: str = ''

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        local_x = c * (x - self.x) + s * (y - self.y)
        local_y = -s * (x - self.x) + c * (y - self.y)
        return (
            abs(local_x) <= self.width / 2.0 + margin
            and abs(local_y) <= self.depth / 2.0 + margin
        )


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Alignment:
    """Frozen world_T_map transform captured at exploration startup."""

    x: float
    y: float
    yaw: float

    def map_to_world(self, x: float, y: float) -> tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * x - s * y, self.y + s * x + c * y


def _float(element: ET.Element | None, default: float = 0.0) -> float:
    return float(element.text) if element is not None and element.text else default


def _pose(text: str | None) -> tuple[float, float, float, float, float, float]:
    values = [float(item) for item in (text or '').split()]
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])  # type: ignore[return-value]


def parse_static_boxes(world_file: str | Path) -> tuple[Box, ...]:
    """Extract static box collisions from the rendered Gazebo world."""
    root = ET.parse(world_file).getroot()
    world = root.find('world')
    if world is None:
        raise ValueError(f'world element missing in {world_file}')
    boxes: list[Box] = []
    for model in world.findall('model'):
        static = model.findtext('static', default='false').strip().lower() == 'true'
        if not static:
            continue
        model_pose = _pose(model.findtext('pose'))
        mx, my, _, _, _, myaw = model_pose
        for collision in model.findall('./link/collision'):
            geometry = collision.find('geometry')
            box = geometry.find('box') if geometry is not None else None
            size = box.findtext('size') if box is not None else None
            if not size:
                continue
            width, depth, _ = (float(value) for value in size.split())
            cx, cy, _, _, _, cyaw = _pose(collision.findtext('pose'))
            c, s = math.cos(myaw), math.sin(myaw)
            boxes.append(
                Box(
                    x=mx + c * cx - s * cy,
                    y=my + s * cx + c * cy,
                    width=width,
                    depth=depth,
                    yaw=myaw + cyaw,
                    name=f"{model.get('name', '')}/{collision.get('name', '')}",
                )
            )
    if not boxes:
        raise ValueError(f'no static box collisions found in {world_file}')
    return tuple(boxes)


def pose_from_pose_array(message: PoseArray, index: int) -> Pose2D | None:
    """Read the robot pose from the deterministic static-world Pose_V index."""
    if not 0 <= index < len(message.poses):
        return None
    pose = message.poses[index]
    q = pose.orientation
    return Pose2D(
        pose.position.x,
        pose.position.y,
        math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
    )


def freeze_alignment(map_base: Pose2D, world_base: Pose2D) -> Alignment:
    """Compute world_T_map from matching map and simulator robot poses."""
    # world_T_map = world_T_base * inverse(map_T_base).
    yaw = world_base.yaw - map_base.yaw
    c, s = math.cos(yaw), math.sin(yaw)
    return Alignment(
        world_base.x - (c * map_base.x - s * map_base.y),
        world_base.y - (s * map_base.x + c * map_base.y),
        yaw,
    )


def _truth_occupied(boxes: tuple[Box, ...], x: float, y: float, margin: float = 0.0) -> bool:
    return any(box.contains(x, y, margin) for box in boxes)


def score_grid(
    data: list[int] | tuple[int, ...],
    spec: GridSpec,
    boxes: tuple[Box, ...],
    alignment: Alignment,
    start_world: tuple[float, float],
    *,
    robot_radius: float = 0.34,
    occupied_tolerance: float = 0.10,
) -> dict[str, float | int | None]:
    """Score a SLAM grid against reachable simulator-truth geometry."""
    if len(data) != spec.width * spec.height:
        raise ValueError('map data does not match map dimensions')
    # Evaluate against the complete static arena extent, not only the current
    # SLAM bounding box (which would make an initially tiny map appear complete).
    min_x = min(box.x - box.width / 2.0 for box in boxes)
    max_x = max(box.x + box.width / 2.0 for box in boxes)
    min_y = min(box.y - box.depth / 2.0 for box in boxes)
    max_y = max(box.y + box.depth / 2.0 for box in boxes)
    eval_resolution = spec.resolution
    eval_width = max(1, int(math.ceil((max_x - min_x) / eval_resolution)))
    eval_height = max(1, int(math.ceil((max_y - min_y) / eval_resolution)))
    eval_spec = GridSpec(eval_width, eval_height, eval_resolution, min_x, min_y)
    world_points = [
        eval_spec.world(x, y)
        for y in range(eval_spec.height)
        for x in range(eval_spec.width)
    ]
    occupied = [
        _truth_occupied(boxes, x, y) for x, y in world_points
    ]
    reachable = [
        not _truth_occupied(boxes, x, y, robot_radius)
        for x, y in world_points
    ]
    sx = int(math.floor((start_world[0] - min_x) / eval_resolution))
    sy = int(math.floor((start_world[1] - min_y) / eval_resolution))
    reachable_region = [False] * len(world_points)
    if eval_spec.inside(sx, sy) and reachable[eval_spec.index(sx, sy)]:
        queue = deque([(sx, sy)])
        reachable_region[eval_spec.index(sx, sy)] = True
        while queue:
            x, y = queue.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if not eval_spec.inside(nx, ny):
                    continue
                index = eval_spec.index(nx, ny)
                if reachable[index] and not reachable_region[index]:
                    reachable_region[index] = True
                    queue.append((nx, ny))

    exposed_occupied = []
    exposed_radius = max(1, int(math.ceil((robot_radius + 0.15) / eval_resolution)))
    for y in range(eval_spec.height):
        for x in range(eval_spec.width):
            index = eval_spec.index(x, y)
            if not occupied[index]:
                exposed_occupied.append(False)
                continue
            exposed = False
            for dy in range(-exposed_radius, exposed_radius + 1):
                for dx in range(-exposed_radius, exposed_radius + 1):
                    nx, ny = x + dx, y + dy
                    if eval_spec.inside(nx, ny) and reachable_region[eval_spec.index(nx, ny)]:
                        exposed = True
            exposed_occupied.append(exposed)

    predicted: list[str] = []
    for world_x, world_y in world_points:
        map_x, map_y = _inverse_alignment(alignment, world_x, world_y)
        c, s = math.cos(-spec.origin_yaw), math.sin(-spec.origin_yaw)
        local_x = c * (map_x - spec.origin_x) - s * (map_y - spec.origin_y)
        local_y = s * (map_x - spec.origin_x) + c * (map_y - spec.origin_y)
        map_cell_x = int(math.floor(local_x / spec.resolution))
        map_cell_y = int(math.floor(local_y / spec.resolution))
        if not spec.inside(map_cell_x, map_cell_y):
            value = UNKNOWN
        else:
            value = data[spec.index(map_cell_x, map_cell_y)]
        predicted.append('unknown' if value == UNKNOWN else
                         'occupied' if value >= OCCUPIED_MIN else
                         'free' if value <= 20 else 'unknown')
    # The evaluation grid spans the complete truth arena and is intentionally
    # larger than an incomplete SLAM grid. Iterating over len(data) here would
    # silently drop the still-unmapped part of the arena from the denominator.
    free_indices = [index for index in range(len(world_points)) if reachable_region[index]]
    occupied_indices = [index for index in range(len(world_points)) if exposed_occupied[index]]
    coverage = (
        sum(predicted[index] != 'unknown' for index in free_indices) / len(free_indices)
        if free_indices else 0.0
    )
    observed_free = [index for index in free_indices if predicted[index] != 'unknown']
    observed_occupied = [index for index in occupied_indices if predicted[index] != 'unknown']
    observed_eval = {
        index for index, value in enumerate(predicted)
        if value != 'unknown' and (reachable_region[index] or exposed_occupied[index])
    }
    free_recall = (
        sum(predicted[index] == 'free' for index in observed_free) / len(observed_free)
        if observed_free else None
    )
    free_predicted = sum(predicted[index] == 'free' for index in observed_eval)
    free_precision = (
        sum(predicted[index] == 'free' for index in observed_free) / free_predicted
        if free_predicted else None
    )
    tolerance_cells = max(0, int(math.ceil(occupied_tolerance / eval_resolution)))

    def near_occupied(index: int) -> bool:
        x, y = index % eval_spec.width, index // eval_spec.width
        for dy in range(-tolerance_cells, tolerance_cells + 1):
            for dx in range(-tolerance_cells, tolerance_cells + 1):
                nx, ny = x + dx, y + dy
                if eval_spec.inside(nx, ny) and predicted[eval_spec.index(nx, ny)] == 'occupied':
                    return True
        return False

    occupied_recall = (
        sum(near_occupied(index) for index in observed_occupied) / len(observed_occupied)
        if observed_occupied else None
    )
    free_f1 = (
        2.0 * free_precision * free_recall / (free_precision + free_recall)
        if free_precision is not None and free_recall is not None
        and free_precision + free_recall > 0 else None
    )
    predicted_occupied = {
        index for index, value in enumerate(predicted)
        if value == 'occupied' and (reachable_region[index] or exposed_occupied[index])
    }
    truth_occupied = set(occupied_indices)

    def near_truth(index: int) -> bool:
        x, y = index % eval_spec.width, index // eval_spec.width
        return any(
            abs((candidate % eval_spec.width) - x) <= tolerance_cells
            and abs((candidate // eval_spec.width) - y) <= tolerance_cells
            for candidate in truth_occupied
        )

    true_positive = sum(near_truth(index) for index in predicted_occupied)
    occupied_precision = true_positive / len(predicted_occupied) if predicted_occupied else None
    occupied_f1 = (
        2.0 * occupied_precision * occupied_recall / (occupied_precision + occupied_recall)
        if occupied_precision is not None and occupied_recall is not None
        and occupied_precision + occupied_recall > 0 else None
    )
    # Balanced map accuracy always gives equal weight to free and occupied
    # structure. A completely missing class scores zero instead of being
    # dropped, which prevents a free-only partial map from reporting 100%.
    accuracy = (
        ((free_f1 or 0.0) + (occupied_f1 or 0.0)) / 2.0
        if free_f1 is not None or occupied_f1 is not None else None
    )
    occupied_iou = (
        true_positive / len(predicted_occupied | truth_occupied)
        if predicted_occupied | truth_occupied else None
    )
    return {
        'coverage_percent': round(100.0 * coverage, 3),
        'map_accuracy_percent': round(100.0 * accuracy, 3) if accuracy is not None else None,
        'free_precision_percent': round(100.0 * free_precision, 3) if free_precision is not None else None,
        'free_recall_percent': round(100.0 * free_recall, 3) if free_recall is not None else None,
        'occupied_recall_percent': round(100.0 * occupied_recall, 3) if occupied_recall is not None else None,
        'occupied_precision_percent': round(100.0 * occupied_precision, 3) if occupied_precision is not None else None,
        'occupied_f1_percent': round(100.0 * occupied_f1, 3) if occupied_f1 is not None else None,
        'occupied_iou_percent': round(100.0 * occupied_iou, 3) if occupied_iou is not None else None,
        'reachable_free_cells': len(free_indices),
        'observed_free_cells': len(observed_free),
        'exposed_occupied_cells': len(occupied_indices),
    }


def _inverse_alignment(alignment: Alignment, x: float, y: float) -> tuple[float, float]:
    c, s = math.cos(alignment.yaw), math.sin(alignment.yaw)
    dx, dy = x - alignment.x, y - alignment.y
    return c * dx + s * dy, -s * dx + c * dy
