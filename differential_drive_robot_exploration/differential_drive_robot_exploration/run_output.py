"""Portable run artifacts; no ROS executor or service calls are needed."""

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shlex
import tempfile


def create_run_directory(root, seed):
    root = Path(root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    return Path(tempfile.mkdtemp(prefix=f'{stamp}_seed_{seed}_', dir=root))


def atomic_write(path, data):
    path = Path(path)
    descriptor, temporary = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(data if isinstance(data, bytes) else data.encode('utf-8'))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def write_map(directory, grid):
    """Write Nav2 trinary PGM/YAML, publishing YAML last as the entry point."""
    if grid is None:
        raise ValueError('No occupancy map received')
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    info = grid.info
    if info.width * info.height != len(grid.data) or not grid.data:
        raise ValueError('Invalid occupancy grid dimensions')
    pixels = bytearray()
    for row in range(info.height - 1, -1, -1):
        for value in grid.data[row * info.width:(row + 1) * info.width]:
            pixels.append(205 if value < 0 else 0 if value >= 65 else 254 if value <= 25 else 205)
    header = f'P5\n# differential_drive_robot_exploration\n{info.width} {info.height}\n255\n'
    atomic_write(directory / 'map.pgm', header.encode('ascii') + pixels)
    q = info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    atomic_write(directory / 'map.yaml',
                 f'image: map.pgm\nmode: trinary\nresolution: {info.resolution}\n'
                 f'origin: [{info.origin.position.x}, {info.origin.position.y}, {yaw}]\n'
                 'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return str(directory / 'map.yaml')


def navigation_command(result):
    return ('ros2 launch differential_drive_robot_exploration saved_map_navigation.launch.py '
            f"map:={shlex.quote(result['map_filename'])} "
            f"world:={shlex.quote(result['world_file'])}")


def summary_text(result):
    def metric(key, unit=''):
        value = result.get(key)
        return 'unavailable' if value is None else f'{value:.2f}{unit}'

    lines = [
        f"Exploration: {result['status'].upper()}",
        f"Reason: {result['completion_reason']}",
        f"Reachable-area coverage: {metric('coverage_percent', '%')}",
        f"Exploration time: {metric('exploration_time_seconds', ' s')}",
        f"Distance travelled: {metric('distance_traveled_m', ' m')}",
        f"Simulation accuracy score: {metric('map_accuracy_percent', '%')}",
        f"Occupied IoU: {metric('occupied_iou_percent', '%')}",
        f"Recovery spins: {result['recovery_spins']} attempted, "
        f"{result['recovery_spins_succeeded']} succeeded, {result['recovery_spins_failed']} failed",
        f"Map: {result['map_status']} — {result.get('map_filename') or 'not available'}",
        f"Run folder: {result['output_directory']}",
    ]
    lines.extend(f'Warning: {warning}' for warning in result.get('warnings', []))
    return '\n'.join(lines)


def write_report(directory, result):
    report = '# Exploration run\n\n```text\n' + summary_text(result) + '\n```\n\n'
    report += ('Coverage and accuracy use this simulation\'s ground truth. '
               'The accuracy score is not a guarantee of geometric correctness.\n\n')
    if result.get('map_filename'):
        report += ('Close the exploration launch with Ctrl+C, source the ROS/workspace setup, '
                   'then reload the matching world and map:\n\n```bash\n'
                   + navigation_command(result) + '\n```\n\n'
                   'When AMCL and Nav2 are ready, use **2D Goal Pose** in RViz.\n')
    atomic_write(Path(directory) / 'summary.md', report)
    # result.json is the last artifact committed: consumers use it as completion.
    atomic_write(Path(directory) / 'result.json', json.dumps(result, indent=2) + '\n')
