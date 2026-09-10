"""Validate durable, reloadable artifacts without starting a ROS graph."""

import json
import math
from unittest.mock import patch

from nav_msgs.msg import OccupancyGrid
import pytest
import yaml

from differential_drive_robot_exploration.run_output import (
    atomic_write, create_run_directory, summary_text, write_map, write_report,
)


def grid():
    message = OccupancyGrid()
    message.info.width = 3
    message.info.height = 2
    message.info.resolution = 0.05
    message.info.origin.position.x = -2.0
    message.info.origin.position.y = 3.0
    message.info.origin.orientation.z = math.sin(0.25)
    message.info.origin.orientation.w = math.cos(0.25)
    message.data = [0, 100, -1, 25, 65, 50]
    return message


def result(directory):
    return dict(status='succeeded', completion_reason='coverage_and_frontiers_complete',
                recovery_spins=0, recovery_spins_succeeded=0, recovery_spins_failed=0,
                output_directory=str(directory), map_status='saved_direct',
                map_filename=str(directory / 'map.yaml'),
                world_file=str(directory / 'navigation_arena.sdf'),
                coverage_percent=100.0, warnings=[])


def test_same_seed_never_overwrites_previous_run(tmp_path):
    first = create_run_directory(tmp_path, 42)
    second = create_run_directory(tmp_path, 42)
    assert first != second
    assert first.is_dir() and second.is_dir()


def test_map_coordinates_thresholds_and_row_orientation(tmp_path):
    filename = write_map(tmp_path, grid())
    metadata = yaml.safe_load(open(filename))
    assert metadata['origin'] == pytest.approx([-2, 3, 0.5])
    assert metadata['resolution'] == pytest.approx(0.05)
    assert metadata['image'] == 'map.pgm'
    pixels = (tmp_path / 'map.pgm').read_bytes().split(b'\n', 4)[4]
    assert pixels == bytes([254, 0, 205, 254, 0, 205])


def test_missing_map_is_an_error(tmp_path):
    with pytest.raises(ValueError, match='No occupancy map'):
        write_map(tmp_path, None)


def test_atomic_write_keeps_existing_file_on_failure(tmp_path):
    path = tmp_path / 'result.json'
    atomic_write(path, 'original')
    with patch('differential_drive_robot_exploration.run_output.os.replace', side_effect=OSError('disk error')):
        with pytest.raises(OSError):
            atomic_write(path, 'replacement')
    assert path.read_text() == 'original'
    assert list(tmp_path.iterdir()) == [path]


def test_report_includes_reload_command_and_unavailable_metrics(tmp_path):
    report = result(tmp_path)
    write_report(tmp_path, report)
    assert json.loads((tmp_path / 'result.json').read_text()) == report
    text = (tmp_path / 'summary.md').read_text()
    assert 'saved_map_navigation.launch.py' in text
    assert 'Simulation accuracy score: unavailable' in text
    assert str(tmp_path / 'map.yaml') in text
    assert 'not a guarantee' in text


def test_incomplete_is_not_presented_as_success(tmp_path):
    report = result(tmp_path)
    report['status'] = 'incomplete'
    assert 'INCOMPLETE' in summary_text(report)
