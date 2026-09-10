"""Seeded static obstacles and bounded moving-box patrol generation."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
import random
import secrets
from typing import Iterable

ARENA_X = (-5.55, 5.55)
ARENA_Y = (-3.55, 3.55)
START = (-5.0, 0.0)
GOAL = (5.0, 0.0)
ROBOT_CLEARANCE = 0.34
MAX_STATIC = 8
MAX_DYNAMIC = 8


@dataclass(frozen=True)
class Rect:
    x: float
    y: float
    width: float
    depth: float

    def expanded(self, margin: float) -> "Rect":
        return Rect(self.x, self.y, self.width + 2 * margin, self.depth + 2 * margin)

    def intersects(self, other: "Rect", margin: float = 0.0) -> bool:
        return (abs(self.x - other.x) < (self.width + other.width) / 2 + margin and
                abs(self.y - other.y) < (self.depth + other.depth) / 2 + margin)


@dataclass(frozen=True)
class StaticObstacle(Rect):
    name: str
    red: float = 0.85
    green: float = 0.28
    blue: float = 0.08


@dataclass(frozen=True)
class DynamicObstacle(Rect):
    name: str
    end_x: float
    end_y: float
    speed: float
    red: float = 0.15
    green: float = 0.65
    blue: float = 0.95


@dataclass(frozen=True)
class Layout:
    seed: int
    static_obstacles: tuple[StaticObstacle, ...]
    dynamic_obstacles: tuple[DynamicObstacle, ...]


PERMANENT = (Rect(0.0, 0.0, 0.15, 4.0),)


def _clear_pose(rect: Rect, pose: tuple[float, float], margin: float = ROBOT_CLEARANCE) -> bool:
    nearest_x = max(rect.x - rect.width / 2, min(pose[0], rect.x + rect.width / 2))
    nearest_y = max(rect.y - rect.depth / 2, min(pose[1], rect.y + rect.depth / 2))
    return ((pose[0] - nearest_x) ** 2 + (pose[1] - nearest_y) ** 2) ** 0.5 >= margin


def _in_bounds(rect: Rect) -> bool:
    return (ARENA_X[0] + rect.width / 2 + 0.15 <= rect.x <= ARENA_X[1] - rect.width / 2 - 0.15 and
            ARENA_Y[0] + rect.depth / 2 + 0.15 <= rect.y <= ARENA_Y[1] - rect.depth / 2 - 0.15)


def _route_exists(obstacles: Iterable[Rect], start=START, goal=GOAL) -> bool:
    # A coarse connectivity check keeps the generated static scene useful even
    # before Nav2's higher-resolution costmap is available.
    resolution = 0.15
    blocked = [r.expanded(ROBOT_CLEARANCE) for r in (*PERMANENT, *obstacles)]
    width = int(round((ARENA_X[1] - ARENA_X[0]) / resolution)) + 1
    height = int(round((ARENA_Y[1] - ARENA_Y[0]) / resolution)) + 1

    def cell(point):
        return (round((point[0] - ARENA_X[0]) / resolution),
                round((point[1] - ARENA_Y[0]) / resolution))

    def free(c):
        if not (0 <= c[0] < width and 0 <= c[1] < height):
            return False
        p = (ARENA_X[0] + c[0] * resolution, ARENA_Y[0] + c[1] * resolution)
        return not any(r.x - r.width / 2 <= p[0] <= r.x + r.width / 2 and
                       r.y - r.depth / 2 <= p[1] <= r.y + r.depth / 2 for r in blocked)

    begin, finish = cell(start), cell(goal)
    if not free(begin) or not free(finish):
        return False
    queue, visited = [begin], {begin}
    while queue:
        current = queue.pop(0)
        if current == finish:
            return True
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (current[0] + dx, current[1] + dy)
            if nxt not in visited and free(nxt):
                visited.add(nxt)
                queue.append(nxt)
    return False


def generate_layout(seed: int, static_count: int = 2, dynamic_count: int = 4,
                    min_speed: float = 0.10, max_speed: float = 0.22) -> Layout:
    if not 0.0 < min_speed <= max_speed <= 0.30:
        raise ValueError('Obstacle speeds must satisfy 0 < min_speed <= max_speed <= 0.30 m/s')
    if not 0 <= static_count <= MAX_STATIC:
        raise ValueError(f"static_count must be between 0 and {MAX_STATIC}")
    if not 0 <= dynamic_count <= MAX_DYNAMIC:
        raise ValueError(f"dynamic_count must be between 0 and {MAX_DYNAMIC}")
    rng = random.Random(seed)
    static: list[StaticObstacle] = []
    occupied: list[Rect] = [*PERMANENT]
    for index in range(static_count):
        for _ in range(1000):
            candidate = Rect(rng.uniform(-4.4, 4.4), rng.uniform(-3.0, 3.0),
                             rng.uniform(0.35, 0.70), rng.uniform(0.35, 0.70))
            if (_in_bounds(candidate) and all(_clear_pose(candidate, p, 0.85) for p in (START, GOAL))
                    and not any(candidate.intersects(other, 0.18) for other in occupied)):
                item = StaticObstacle(candidate.x, candidate.y, candidate.width, candidate.depth,
                                      f"static_obstacle_{index}")
                static.append(item)
                occupied.append(item)
                break
        else:
            raise RuntimeError("unable to place static obstacle")

    dynamic: list[DynamicObstacle] = []
    for index in range(dynamic_count):
        for _ in range(1500):
            horizontal = rng.random() < 0.5
            width, depth = (rng.uniform(0.35, 0.55), rng.uniform(0.35, 0.55))
            length = rng.uniform(1.0, 2.0)
            x = rng.uniform(-4.3, 4.3)
            y = rng.uniform(-2.8, 2.8)
            if horizontal:
                end_x, end_y = x + (length if rng.random() < 0.5 else -length), y
            else:
                end_x, end_y = x, y + (length if rng.random() < 0.5 else -length)
            start_rect = Rect(x, y, width, depth)
            end_rect = Rect(end_x, end_y, width, depth)
            corridor = Rect((x + end_x) / 2, (y + end_y) / 2,
                            max(width, abs(end_x - x) + width),
                            max(depth, abs(end_y - y) + depth))
            if not (_in_bounds(start_rect) and _in_bounds(end_rect)
                    and all(_clear_pose(corridor, pose, 0.85) for pose in (START, GOAL))
                    and not any(corridor.intersects(other, 0.20) for other in occupied)):
                continue
            item = DynamicObstacle(x, y, width, depth, f"dynamic_obstacle_{index}",
                                   end_x, end_y, rng.uniform(min_speed, max_speed))
            dynamic.append(item)
            occupied.append(corridor)
            break
        else:
            raise RuntimeError("unable to place dynamic obstacle")

    if not _route_exists(static):
        return generate_layout(seed + 1, static_count, dynamic_count, min_speed, max_speed)
    return Layout(seed, tuple(static), tuple(dynamic))


def _static_sdf(item: StaticObstacle) -> str:
    return f'''<model name="{item.name}"><static>true</static>
      <pose>{item.x:.4f} {item.y:.4f} 0.25 0 0 0</pose><link name="link">
      <collision name="collision"><geometry><box><size>{item.width:.4f} {item.depth:.4f} 0.5</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>{item.width:.4f} {item.depth:.4f} 0.5</size></box></geometry>
      <material><ambient>{item.red} {item.green} {item.blue} 1</ambient><diffuse>{item.red} {item.green} {item.blue} 1</diffuse></material></visual>
      </link></model>'''


def _dynamic_sdf(item: DynamicObstacle) -> str:
    return f'''<model name="{item.name}"><static>false</static>
      <pose>{item.x:.4f} {item.y:.4f} 0.25 0 0 0</pose><link name="link"><gravity>false</gravity><inertial><mass>50</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="collision"><geometry><box><size>{item.width:.4f} {item.depth:.4f} 0.5</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>{item.width:.4f} {item.depth:.4f} 0.5</size></box></geometry>
      <material><ambient>{item.red} {item.green} {item.blue} 1</ambient><diffuse>{item.red} {item.green} {item.blue} 1</diffuse></material></visual></link>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose><update_frequency>20</update_frequency></plugin>
      </model>'''


def generate_world(template_path: str | Path, output_directory: str | Path,
                   seed: int | None, static_count: int, dynamic_count: int,
                   min_speed: float = 0.10, max_speed: float = 0.22) -> tuple[Layout, Path, Path]:
    actual_seed = secrets.randbits(31) if seed is None else seed
    layout = generate_layout(actual_seed, static_count, dynamic_count, min_speed, max_speed)
    template = Path(template_path).read_text(encoding="utf-8")
    output_root = Path(output_directory) / f"seed_{actual_seed}"
    output_root.mkdir(parents=True, exist_ok=True)
    world_path = output_root / "dynamic_arena.sdf"
    metadata_path = output_root / "layout.json"
    world = template.replace("<!-- STATIC_OBSTACLES -->", "\n".join(_static_sdf(i) for i in layout.static_obstacles))
    world = world.replace("<!-- DYNAMIC_OBSTACLES -->", "\n".join(_dynamic_sdf(i) for i in layout.dynamic_obstacles))
    world_path.write_text(world, encoding="utf-8")
    metadata_path.write_text(json.dumps({"seed": layout.seed, "start": {"x": START[0], "y": START[1]},
                                         "goal": {"x": GOAL[0], "y": GOAL[1]},
                                         "static_obstacles": [asdict(i) for i in layout.static_obstacles],
                                         "dynamic_obstacles": [asdict(i) for i in layout.dynamic_obstacles]}, indent=2) + "\n",
                              encoding="utf-8")
    return layout, world_path, metadata_path
