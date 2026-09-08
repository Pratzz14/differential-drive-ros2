"""Deterministic, route-safe obstacle generation for the navigation arena."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import random
import secrets
from typing import Iterable


# Safe robot-centre bounds already account for the exterior walls and the
# conservative circumscribed footprint radius.
ARENA_X = (-5.55, 5.55)
ARENA_Y = (-3.55, 3.55)
START = (-5.0, 0.0)
GOAL = (5.0, 0.0)
ROBOT_CLEARANCE = 0.34
RESERVED_RADIUS = 0.85
GRID_RESOLUTION = 0.10
MAX_OBSTACLES = 12


@dataclass(frozen=True)
class Rectangle:
    x: float
    y: float
    width: float
    depth: float

    def expanded(self, margin: float) -> "Rectangle":
        return Rectangle(
            self.x,
            self.y,
            self.width + 2.0 * margin,
            self.depth + 2.0 * margin,
        )

    def contains(self, x: float, y: float) -> bool:
        return (
            abs(x - self.x) <= self.width / 2.0
            and abs(y - self.y) <= self.depth / 2.0
        )

    def intersects(self, other: "Rectangle", margin: float = 0.0) -> bool:
        return (
            abs(self.x - other.x)
            < (self.width + other.width) / 2.0 + margin
            and abs(self.y - other.y)
            < (self.depth + other.depth) / 2.0 + margin
        )


@dataclass(frozen=True)
class Obstacle(Rectangle):
    name: str
    red: float
    green: float
    blue: float


@dataclass(frozen=True)
class Layout:
    seed: int
    obstacles: tuple[Obstacle, ...]


PERMANENT_OBSTACLES = (Rectangle(0.0, 0.0, 0.15, 4.0),)


def _clear_of_reserved_pose(obstacle: Rectangle, pose: tuple[float, float]) -> bool:
    nearest_x = max(
        obstacle.x - obstacle.width / 2.0,
        min(pose[0], obstacle.x + obstacle.width / 2.0),
    )
    nearest_y = max(
        obstacle.y - obstacle.depth / 2.0,
        min(pose[1], obstacle.y + obstacle.depth / 2.0),
    )
    return math.hypot(pose[0] - nearest_x, pose[1] - nearest_y) >= RESERVED_RADIUS


def _candidate_is_valid(
    candidate: Rectangle,
    obstacles: Iterable[Rectangle],
    reserved_poses: Iterable[tuple[float, float]] = (START, GOAL),
) -> bool:
    half_width = candidate.width / 2.0
    half_depth = candidate.depth / 2.0
    if not (
        ARENA_X[0] + half_width + 0.15 <= candidate.x <= ARENA_X[1] - half_width - 0.15
        and ARENA_Y[0] + half_depth + 0.15 <= candidate.y <= ARENA_Y[1] - half_depth - 0.15
    ):
        return False
    if not all(_clear_of_reserved_pose(candidate, pose) for pose in reserved_poses):
        return False
    return not any(candidate.intersects(other, margin=0.15) for other in obstacles)


def route_exists(
    obstacles: Iterable[Rectangle],
    clearance: float = ROBOT_CLEARANCE,
    *,
    start: tuple[float, float] = START,
    goal: tuple[float, float] = GOAL,
) -> bool:
    """Return whether a four-connected inflated-grid route joins start to goal."""
    blocked = [rect.expanded(clearance) for rect in (*PERMANENT_OBSTACLES, *obstacles)]
    width = int(round((ARENA_X[1] - ARENA_X[0]) / GRID_RESOLUTION)) + 1
    height = int(round((ARENA_Y[1] - ARENA_Y[0]) / GRID_RESOLUTION)) + 1

    def to_cell(point: tuple[float, float]) -> tuple[int, int]:
        return (
            int(round((point[0] - ARENA_X[0]) / GRID_RESOLUTION)),
            int(round((point[1] - ARENA_Y[0]) / GRID_RESOLUTION)),
        )

    def is_free(cell: tuple[int, int]) -> bool:
        column, row = cell
        if not (0 <= column < width and 0 <= row < height):
            return False
        x = ARENA_X[0] + column * GRID_RESOLUTION
        y = ARENA_Y[0] + row * GRID_RESOLUTION
        return not any(rect.contains(x, y) for rect in blocked)

    start_cell = to_cell(start)
    goal_cell = to_cell(goal)
    if not is_free(start_cell) or not is_free(goal_cell):
        return False

    queue = deque([start_cell])
    visited = {start_cell}
    while queue:
        current = queue.popleft()
        if current == goal_cell:
            return True
        for delta_x, delta_y in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (current[0] + delta_x, current[1] + delta_y)
            if neighbor not in visited and is_free(neighbor):
                visited.add(neighbor)
                queue.append(neighbor)
    return False


def _fallback_layout(
    seed: int,
    count: int,
    start: tuple[float, float] = START,
    goal: tuple[float, float] = GOAL,
) -> Layout:
    positions = (
        (-3.6, 1.0), (-2.6, -0.8), (-3.2, 2.5), (-3.6, -2.5),
        (3.6, 1.0), (2.6, -0.8), (3.2, 2.5), (3.6, -2.5),
        (-1.4, 3.0), (1.4, -3.0), (-4.4, 2.2), (4.4, -2.2),
    )
    obstacles = tuple(
        Obstacle(x, y, 0.40, 0.40, f"random_obstacle_{index}", 0.85, 0.28, 0.08)
        for index, (x, y) in enumerate(positions[:count])
    )
    if not all(
        _clear_of_reserved_pose(obstacle, pose)
        for obstacle in obstacles
        for pose in (start, goal)
    ) or not route_exists(obstacles, start=start, goal=goal):
        raise RuntimeError("The deterministic fallback obstacle layout is not traversable")
    return Layout(seed, obstacles)


def generate_layout(
    seed: int,
    count: int = 6,
    max_layout_attempts: int = 100,
    *,
    start: tuple[float, float] = START,
    goal: tuple[float, float] = GOAL,
) -> Layout:
    """Generate a deterministic collision-free layout with a valid route."""
    if not 0 <= count <= MAX_OBSTACLES:
        raise ValueError(f"obstacle_count must be between 0 and {MAX_OBSTACLES}")
    for name, pose in (('start', start), ('goal', goal)):
        if not (
            ARENA_X[0] <= pose[0] <= ARENA_X[1]
            and ARENA_Y[0] <= pose[1] <= ARENA_Y[1]
        ):
            raise ValueError(f"{name} pose {pose} is outside the navigable arena")
        if any(
            obstacle.expanded(ROBOT_CLEARANCE).contains(*pose)
            for obstacle in PERMANENT_OBSTACLES
        ):
            raise ValueError(f"{name} pose {pose} intersects a permanent obstacle")
    rng = random.Random(seed)
    for _ in range(max_layout_attempts):
        obstacles: list[Obstacle] = []
        for index in range(count):
            placed = False
            for _ in range(500):
                width = rng.uniform(0.35, 0.80)
                depth = rng.uniform(0.35, 0.80)
                candidate = Rectangle(
                    rng.uniform(ARENA_X[0] + 0.6, ARENA_X[1] - 0.6),
                    rng.uniform(ARENA_Y[0] + 0.6, ARENA_Y[1] - 0.6),
                    width,
                    depth,
                )
                if _candidate_is_valid(
                    candidate,
                    (*PERMANENT_OBSTACLES, *obstacles),
                    (start, goal),
                ):
                    obstacles.append(
                        Obstacle(
                            candidate.x,
                            candidate.y,
                            width,
                            depth,
                            f"random_obstacle_{index}",
                            rng.uniform(0.55, 0.95),
                            rng.uniform(0.12, 0.38),
                            rng.uniform(0.05, 0.20),
                        )
                    )
                    placed = True
                    break
            if not placed:
                break
        if len(obstacles) == count and route_exists(
            obstacles, start=start, goal=goal
        ):
            return Layout(seed, tuple(obstacles))
    return _fallback_layout(seed, count, start, goal)


def _obstacle_sdf(obstacle: Obstacle) -> str:
    return f"""
    <model name="{obstacle.name}">
      <static>true</static>
      <pose>{obstacle.x:.6f} {obstacle.y:.6f} 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{obstacle.width:.6f} {obstacle.depth:.6f} 0.5</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{obstacle.width:.6f} {obstacle.depth:.6f} 0.5</size></box></geometry>
          <material><ambient>{obstacle.red:.3f} {obstacle.green:.3f} {obstacle.blue:.3f} 1</ambient><diffuse>{obstacle.red:.3f} {obstacle.green:.3f} {obstacle.blue:.3f} 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def generate_world(
    template_path: str | Path,
    output_directory: str | Path,
    seed: int | None,
    count: int,
    *,
    start: tuple[float, float] = START,
    goal: tuple[float, float] = GOAL,
) -> tuple[Layout, Path, Path]:
    """Render an arena and metadata file, returning their absolute paths."""
    actual_seed = secrets.randbits(31) if seed is None else seed
    layout = generate_layout(actual_seed, count, start=start, goal=goal)
    template = Path(template_path).read_text(encoding="utf-8")
    marker = "<!-- RANDOM_OBSTACLES -->"
    if marker not in template:
        raise ValueError(f"Obstacle marker missing from {template_path}")

    output_root = Path(output_directory) / f"seed_{actual_seed}"
    output_root.mkdir(parents=True, exist_ok=True)
    world_path = output_root / "navigation_arena.sdf"
    metadata_path = output_root / "layout.json"
    world_path.write_text(
        template.replace(marker, "\n".join(_obstacle_sdf(item) for item in layout.obstacles)),
        encoding="utf-8",
    )
    metadata_path.write_text(
        json.dumps(
            {
                "seed": layout.seed,
                "start": {"x": start[0], "y": start[1], "yaw": 0.0},
                "goal": {"x": goal[0], "y": goal[1], "yaw": 0.0},
                "obstacles": [asdict(item) for item in layout.obstacles],
            },
            indent=2,
        ) + "\n",
        encoding="utf-8",
    )
    return layout, world_path, metadata_path
