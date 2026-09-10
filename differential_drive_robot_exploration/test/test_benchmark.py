from differential_drive_robot_exploration.benchmark import aggregate


def test_aggregate_reports_separate_metric_statistics():
    summary = aggregate([
        {'seed': 1, 'coverage_percent': 80, 'exploration_time_seconds': 10,
         'distance_traveled_m': 3, 'map_accuracy_percent': 90},
        {'seed': 2, 'coverage_percent': 100, 'exploration_time_seconds': 20,
         'distance_traveled_m': 5, 'map_accuracy_percent': 70},
    ])
    assert summary['trial_count'] == 2
    assert summary['coverage_percent']['mean'] == 90.0
    assert summary['map_accuracy_percent']['median'] == 80.0
