#!/usr/bin/env python3

import numpy as np

from _exploration_cpp import c_astar


def plan(traversability, weight):
    occupancy = np.zeros(traversability.shape, dtype=np.uint8)
    start = np.array([2, 0], dtype=np.int32)
    goal = np.array([2, 6], dtype=np.int32)
    obstacle_values = np.array([255], dtype=np.uint8)
    return c_astar(start, goal, occupancy, obstacle_values,
                   traversability.astype(np.float32), weight,
                   0.0, 1.0, 1, False)


def test_soft_traversability_cost_changes_route_without_becoming_obstacle():
    traversability = np.zeros((5, 7), dtype=np.float32)
    traversability[2, 1:6] = 1.0

    plain_success, plain_path = plan(traversability, 0.0)
    cost_success, cost_path = plan(traversability, 10.0)

    assert plain_success and cost_success
    assert np.all(plain_path[:, 0] == 2)
    assert np.any(cost_path[:, 0] != 2)


def test_traversability_shape_must_match_occupancy():
    occupancy = np.zeros((5, 7), dtype=np.uint8)
    traversability = np.zeros((4, 7), dtype=np.float32)
    start = np.array([2, 0], dtype=np.int32)
    goal = np.array([2, 6], dtype=np.int32)
    obstacle_values = np.array([255], dtype=np.uint8)

    try:
        c_astar(start, goal, occupancy, obstacle_values, traversability,
                1.0, 0.0, 1.0, 1, False)
    except RuntimeError as error:
        assert 'shape must match' in str(error)
    else:
        raise AssertionError('mismatched traversability grid was accepted')
