"""
Simple test examples for setting up Python testing with colcon.
"""

import launch
import launch_ros


def test_true():
    assert True, "Did not get True."


def test_false():
    assert not False, "Did not get False."


def test_sum():
    ans = 2 + 2
    assert ans == 4, "Math failure."

