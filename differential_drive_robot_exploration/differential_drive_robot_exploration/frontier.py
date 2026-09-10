"""Pure frontier extraction helpers used by the exploration node."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable, Sequence


FREE_MAX = 20
OCCUPIED_MIN = 65
UNKNOWN = -1
NEIGHBOURS_8 = tuple(
    (dx, dy) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if (dx, dy) != (0, 0)
)


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def index(self, x: int, y: int) -> int:
        return y * self.width + x

    def inside(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def world(self, x: int, y: int) -> tuple[float, float]:
        local_x = (x + 0.5) * self.resolution
        local_y = (y + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (
            self.origin_x + c * local_x - s * local_y,
            self.origin_y + s * local_x + c * local_y,
        )


@dataclass(frozen=True)
class Frontier:
    cells: tuple[tuple[int, int], ...]
    goal_x: float
    goal_y: float
    yaw: float
    information_gain_m: float
    clearance_m: float


def _occupied_distance(data: Sequence[int], spec: GridSpec) -> list[float]:
    """Return an approximate distance-to-occupied value for every cell."""
    distances = [math.inf] * (spec.width * spec.height)
    queue: deque[tuple[int, int]] = deque()
    for y in range(spec.height):
        for x in range(spec.width):
            if data[spec.index(x, y)] >= OCCUPIED_MIN:
                distances[spec.index(x, y)] = 0.0
                queue.append((x, y))
    while queue:
        x, y = queue.popleft()
        current = distances[spec.index(x, y)]
        for dx, dy in NEIGHBOURS_8:
            nx, ny = x + dx, y + dy
            if not spec.inside(nx, ny):
                continue
            step = math.hypot(dx, dy) * spec.resolution
            index = spec.index(nx, ny)
            if current + step < distances[index]:
                distances[index] = current + step
                queue.append((nx, ny))
    return distances


def extract_frontiers(
    data: Sequence[int],
    spec: GridSpec,
    *,
    min_cells: int = 5,
    robot_radius: float = 0.34,
) -> list[Frontier]:
    """Extract safe frontier clusters from a row-major occupancy grid."""
    if len(data) != spec.width * spec.height:
        raise ValueError('occupancy data does not match grid dimensions')
    distances = _occupied_distance(data, spec)
    frontier_cells: set[tuple[int, int]] = set()
    for y in range(spec.height):
        for x in range(spec.width):
            value = data[spec.index(x, y)]
            if not (0 <= value <= FREE_MAX):
                continue
            if distances[spec.index(x, y)] < robot_radius:
                continue
            if any(
                spec.inside(x + dx, y + dy)
                and data[spec.index(x + dx, y + dy)] == UNKNOWN
                for dx, dy in NEIGHBOURS_8
            ):
                frontier_cells.add((x, y))

    frontiers: list[Frontier] = []
    while frontier_cells:
        seed = frontier_cells.pop()
        component = [seed]
        queue = deque([seed])
        while queue:
            x, y = queue.popleft()
            for dx, dy in NEIGHBOURS_8:
                neighbour = (x + dx, y + dy)
                if neighbour in frontier_cells:
                    frontier_cells.remove(neighbour)
                    component.append(neighbour)
                    queue.append(neighbour)
        if len(component) < min_cells:
            continue

        centroid_x = sum(x for x, _ in component) / len(component)
        centroid_y = sum(y for _, y in component) / len(component)
        representative = min(
            component,
            key=lambda cell: (
                (cell[0] - centroid_x) ** 2 + (cell[1] - centroid_y) ** 2,
                -distances[spec.index(*cell)],
            ),
        )
        goal_x, goal_y = spec.world(*representative)
        unknown_neighbours = [
            (x + dx, y + dy)
            for x, y in component
            for dx, dy in NEIGHBOURS_8
            if spec.inside(x + dx, y + dy)
            and data[spec.index(x + dx, y + dy)] == UNKNOWN
        ]
        if unknown_neighbours:
            target_x = sum(x for x, _ in unknown_neighbours) / len(unknown_neighbours)
            target_y = sum(y for _, y in unknown_neighbours) / len(unknown_neighbours)
            yaw = math.atan2(target_y - representative[1], target_x - representative[0])
        else:
            yaw = 0.0
        frontiers.append(
            Frontier(
                cells=tuple(sorted(component)),
                goal_x=goal_x,
                goal_y=goal_y,
                yaw=yaw,
                information_gain_m=len(component) * spec.resolution,
                clearance_m=distances[spec.index(*representative)],
            )
        )
    return sorted(frontiers, key=lambda frontier: -frontier.information_gain_m)


def path_length(points: Iterable[tuple[float, float]]) -> float:
    points = list(points)
    return sum(math.dist(first, second) for first, second in zip(points, points[1:]))
