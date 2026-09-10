"""Run reproducible exploration trials and aggregate their JSON results."""
# The benchmark command intentionally keeps launch command lines readable.
# flake8: noqa

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys

from .run_output import create_run_directory


METRICS = (
    'coverage_percent',
    'exploration_time_seconds',
    'distance_traveled_m',
    'map_accuracy_percent',
)


def _latest_result(root: Path) -> dict:
    candidates = sorted(root.rglob('result.json'), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f'no result.json found below {root}')
    return json.loads(candidates[-1].read_text(encoding='utf-8'))


def aggregate(results: list[dict]) -> dict:
    summary = {'trial_count': len(results), 'trials': results}
    for metric in METRICS:
        values = [float(item[metric]) for item in results if item.get(metric) is not None]
        if not values:
            summary[metric] = {'count': 0}
            continue
        ordered = sorted(values)
        middle = len(ordered) // 2
        median = ordered[middle] if len(ordered) % 2 else (ordered[middle - 1] + ordered[middle]) / 2.0
        mean = sum(values) / len(values)
        variance = sum((value - mean) ** 2 for value in values) / len(values)
        summary[metric] = {
            'count': len(values),
            'mean': round(mean, 3),
            'median': round(median, 3),
            'stddev': round(variance ** 0.5, 3),
            'min': round(min(values), 3),
            'max': round(max(values), 3),
        }
    return summary


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seeds', nargs='+', type=int, default=[7, 21, 42, 84, 126])
    parser.add_argument('--obstacle-count', type=int, default=6)
    parser.add_argument('--output-directory', default='/tmp/differential_drive_exploration_batch')
    parser.add_argument('--mission-timeout', type=float, default=900.0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--ros2-launch', default='ros2')
    args = parser.parse_args(argv)

    root = Path(args.output_directory)
    root.mkdir(parents=True, exist_ok=True)
    results: list[dict] = []
    for seed in args.seeds:
        trial_root = create_run_directory(root, seed)
        command = [
            args.ros2_launch, 'launch', 'differential_drive_robot_exploration',
            'exploration.launch.py', f'seed:={seed}',
            f'obstacle_count:={args.obstacle_count}',
            f'mission_timeout:={args.mission_timeout}',
            f'output_directory:={trial_root}',
            'finish_behavior:=shutdown',
            'rviz:=false', f'headless:={str(args.headless).lower()}',
        ]
        completed = subprocess.run(command, check=False)
        if completed.returncode != 0:
            print(f'seed {seed} exited with code {completed.returncode}', file=sys.stderr)
        try:
            results.append(_latest_result(trial_root))
        except FileNotFoundError as exception:
            results.append({'seed': seed, 'status': 'missing_result', 'failure_reason': str(exception)})

    summary = aggregate(results)
    (root / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    fields = ['seed', 'status', 'completion_reason', *METRICS]
    with (root / 'summary.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(results)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
