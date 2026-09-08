"""Ensure normal helper exits do not stop Gazebo, but lost essential nodes do."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import Shutdown
from launch_ros.actions import Node
import pytest


source = (Path(__file__).resolve().parents[2] / 'differential_drive_robot_simulation'
          / 'launch' / 'session.launch.py')
spec = importlib.util.spec_from_file_location('robot_session', source)
session = importlib.util.module_from_spec(spec)
spec.loader.exec_module(session)


@pytest.mark.parametrize('name,code,shutdown', [
    ('spawn_differential_drive_robot', 0, False),
    ('navigation_mission', 0, False),
    ('navigation_rviz', 0, False),
    ('spawn_differential_drive_robot', 1, True),
    ('navigation_mission', 1, True),
    ('robot_state_publisher', 0, True),
    ('navigation_ros_gz_bridge', -11, True),
])
def test_process_exit(name, code, shutdown):
    context = LaunchContext()
    action = Node(executable='/bin/true', name=name)
    # Expand real launch_ros names without starting a subprocess. This covers
    # the unspecified namespace case that occurs in the actual launch files.
    action._perform_substitutions(context)
    event = SimpleNamespace(action=action, returncode=code, process_name=name)
    result = session._stop_on_exit(event, context)
    assert bool(result) is shutdown
    if shutdown:
        assert isinstance(result[0], Shutdown)


def test_shutdown_does_not_recursively_trigger_shutdown():
    event = SimpleNamespace(returncode=-2)
    assert session._stop_on_exit(event, SimpleNamespace(is_shutdown=True)) == []
