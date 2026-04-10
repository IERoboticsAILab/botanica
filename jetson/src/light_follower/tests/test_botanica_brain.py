#!/usr/bin/env python3
"""
Unit tests for BOTanica Brain state machine.

Tests state transitions, priority interrupts, and frame separation.
Uses mocked ROS infrastructure — no roscore needed.

Run with:
  python3 -m pytest test_botanica_brain.py -v
  # or
  python3 test_botanica_brain.py
"""
import sys
import math
import os
import unittest
from unittest.mock import MagicMock, patch

# Mock all ROS/external modules before importing botanica_brain
for mod_name in [
    'rospy', 'cv2',
    'sensor_msgs', 'sensor_msgs.msg',
    'geometry_msgs', 'geometry_msgs.msg',
    'nav_msgs', 'nav_msgs.msg',
    'std_msgs', 'std_msgs.msg',
    'sensor_publisher', 'sensor_publisher.msg',
    'distancefield', 'distancefield.msg',
]:
    sys.modules[mod_name] = MagicMock()

import rospy

# Configure rospy mocks
_mock_time = MagicMock()
_mock_time.to_sec.return_value = 100.0
_mock_time.__sub__ = MagicMock(return_value=_mock_time)
rospy.Time.now.return_value = _mock_time
rospy.Duration = MagicMock()
rospy.get_param = MagicMock(side_effect=lambda key, default=None: default)
rospy.init_node = MagicMock()
rospy.Timer = MagicMock()
rospy.Subscriber = MagicMock()
rospy.Publisher = MagicMock(return_value=MagicMock())
rospy.spin = MagicMock()

# Import the module
import importlib.util

_spec = importlib.util.spec_from_file_location(
    "botanica_brain",
    os.path.join(os.path.dirname(__file__), "..", "scripts", "botanica_brain.py"),
)
bb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bb)

State = bb.State
NavigationMode = bb.NavigationMode


def make_brain():
    """Create a BOTanicaBrain with all state initialized but without calling __init__."""
    brain = object.__new__(bb.BOTanicaBrain)

    # Config
    brain.BATTERY_LOW_THRESHOLD = 0.20
    brain.BATTERY_FULL = 1.0
    brain.MOISTURE_LOW_THRESHOLD = 30
    brain.DAY_START_HOUR = 6
    brain.DAY_END_HOUR = 20
    brain.ARRIVAL_TOLERANCE = 0.15
    brain.DOSE_DURATION = 5.0
    brain.DOCK_COORDS = (0.0, 0.0)
    brain.WATER_COORDS = (1.0, 1.0)
    brain.OPTITRACK_FRAME = "world"
    brain.BRIGHTNESS_SCAN_THRESHOLD = 150
    brain.BRIGHTNESS_MOVE_THRESHOLD = 160
    brain.MIN_BRIGHT_ANGLES = 5
    brain.LIGHT_MOVE_SPEED = 0.1
    brain.BRIGHT_CONFIRM_COUNT = 3
    brain.OBSTACLE_STOP_DISTANCE = 0.4
    brain.OBSTACLE_SLOW_DISTANCE = 0.8
    brain.OBSTACLE_CHECK_WIDTH = 0.3
    brain.SUNBATHING_RECHECK_INTERVAL = 5.0
    brain.SUNBATHING_DROP_THRESHOLD = 100
    brain.SUNBATHING_DROP_COUNT = 3
    brain.MAX_SUNSPOTS = 5
    brain.SUNSPOT_ARRIVAL_BRIGHTNESS = 130

    # State
    brain.state = State.IDLE
    brain.nav_mode = NavigationMode.DIRECT
    brain.battery_percent = 1.0
    brain.soil_moisture = 100
    brain.image = None
    brain.depth_image = None
    brain.min_obstacle_dist = float('inf')
    brain.optitrack_pose = None
    brain.odom_yaw = 0.0
    brain.odom_pos = (0.0, 0.0)
    brain.scan_start_yaw = None
    brain.scan_last_yaw = None
    brain.scan_accumulated_rotation = 0.0
    brain.scan_start_time = None
    brain.brightness_log = []
    brain.target_yaw = 0.0
    brain.move_start_pos = None
    brain.bright_counter = 0
    brain._debug_cmd_count = 0
    brain._debug_last_cmd_time = 0
    brain._debug_yaw_samples = []
    brain._debug_scan_cmd_gaps = []
    brain.nav_target = None
    brain.gvf_active = False
    brain.dose_start_time = None
    brain.sunspot_memory = []
    brain.current_sunspot = None
    brain.sunbathing_drop_counter = 0
    brain.sunbathing_last_check = None

    # Mock publishers
    brain.cmd_pub = MagicMock()
    brain.path_pub = MagicMock()
    brain.nav_mode_pub = MagicMock()
    brain.debug_scan_pub = MagicMock()
    brain.event_pub = MagicMock()

    return brain


# ─── Priority Interrupts ───────────────────────────────────────────────

class TestPriorityInterrupts(unittest.TestCase):

    def test_battery_low_interrupts_light_scan(self):
        brain = make_brain()
        brain.state = State.LIGHT_SCAN
        brain.battery_percent = 0.15
        brain.optitrack_pose = (2.0, 2.0, 0.0)
        brain.update(None)
        self.assertEqual(brain.state, State.GO_TO_DOCK)

    def test_battery_low_does_not_interrupt_charging(self):
        brain = make_brain()
        brain.state = State.CHARGING
        brain.battery_percent = 0.15
        brain.update(None)
        self.assertEqual(brain.state, State.CHARGING)

    def test_battery_low_does_not_interrupt_go_to_dock(self):
        brain = make_brain()
        brain.state = State.GO_TO_DOCK
        brain.battery_percent = 0.10
        brain.nav_target = (0.0, 0.0)
        brain.optitrack_pose = (5.0, 5.0, 0.0)  # Far from dock
        brain.update(None)
        self.assertEqual(brain.state, State.GO_TO_DOCK)

    def test_moisture_low_interrupts_light_seeking(self):
        brain = make_brain()
        brain.state = State.LIGHT_SCAN
        brain.soil_moisture = 20
        brain.optitrack_pose = (2.0, 2.0, 0.0)
        brain.update(None)
        self.assertEqual(brain.state, State.GO_TO_WATER)

    def test_moisture_low_does_not_interrupt_battery_states(self):
        brain = make_brain()
        brain.state = State.GO_TO_DOCK
        brain.soil_moisture = 20
        brain.battery_percent = 0.50
        brain.nav_target = (0.0, 0.0)
        brain.optitrack_pose = (5.0, 5.0, 0.0)
        brain.update(None)
        # Should stay GO_TO_DOCK (moisture doesn't override battery)
        self.assertEqual(brain.state, State.GO_TO_DOCK)

    def test_battery_takes_priority_over_moisture(self):
        brain = make_brain()
        brain.state = State.LIGHT_SCAN
        brain.battery_percent = 0.10
        brain.soil_moisture = 10
        brain.optitrack_pose = (2.0, 2.0, 0.0)
        brain.update(None)
        self.assertEqual(brain.state, State.GO_TO_DOCK)


# ─── Frame Separation ──────────────────────────────────────────────────

class TestFrameSeparation(unittest.TestCase):

    def test_gvf_refuses_without_optitrack(self):
        brain = make_brain()
        brain.optitrack_pose = None
        result = brain.start_gvf_navigation((1.0, 1.0))
        self.assertFalse(result)
        self.assertFalse(brain.gvf_active)

    def test_gvf_succeeds_with_optitrack(self):
        brain = make_brain()
        brain.optitrack_pose = (0.5, 0.5, 0.0)
        result = brain.start_gvf_navigation((1.0, 1.0))
        self.assertTrue(result)
        self.assertTrue(brain.gvf_active)
        self.assertEqual(brain.nav_target, (1.0, 1.0))

    def test_gvf_path_uses_optitrack_frame(self):
        brain = make_brain()
        brain.optitrack_pose = (0.0, 0.0, 0.0)
        brain.start_gvf_navigation((1.0, 1.0))
        # The path_pub.publish was called — check the frame_id
        call_args = brain.path_pub.publish.call_args
        path_msg = call_args[0][0]
        self.assertEqual(path_msg.header.frame_id, "world")

    def test_distance_to_optitrack_inf_without_pose(self):
        brain = make_brain()
        brain.optitrack_pose = None
        self.assertEqual(brain.distance_to_optitrack((1.0, 1.0)), float('inf'))

    def test_distance_to_optitrack_correct(self):
        brain = make_brain()
        brain.optitrack_pose = (0.0, 0.0, 0.0)
        self.assertAlmostEqual(brain.distance_to_optitrack((3.0, 4.0)), 5.0)

    def test_check_arrival_false_without_optitrack(self):
        brain = make_brain()
        brain.optitrack_pose = None
        brain.nav_target = (1.0, 1.0)
        self.assertFalse(brain.check_arrival())

    def test_check_arrival_true_when_close(self):
        brain = make_brain()
        brain.optitrack_pose = (1.0, 1.05, 0.0)
        brain.nav_target = (1.0, 1.0)
        self.assertTrue(brain.check_arrival())

    def test_check_arrival_false_when_far(self):
        brain = make_brain()
        brain.optitrack_pose = (1.0, 2.0, 0.0)
        brain.nav_target = (1.0, 1.0)
        self.assertFalse(brain.check_arrival())


# ─── State Transitions ─────────────────────────────────────────────────

class TestStateTransitions(unittest.TestCase):

    def test_idle_to_light_scan_daytime_no_spots(self):
        brain = make_brain()
        brain.is_daytime = MagicMock(return_value=True)
        brain.get_best_sunspot = MagicMock(return_value=None)
        brain.do_idle()
        self.assertEqual(brain.state, State.LIGHT_SCAN)

    def test_idle_to_sunspot_daytime_with_spots(self):
        brain = make_brain()
        brain.is_daytime = MagicMock(return_value=True)
        spot = {'optitrack_pos': (2.0, 3.0), 'brightness': 200, 'fail_count': 0}
        brain.get_best_sunspot = MagicMock(return_value=spot)
        brain.do_idle()
        self.assertEqual(brain.state, State.GO_TO_SUNSPOT)
        self.assertEqual(brain.current_sunspot, spot)

    def test_idle_stays_idle_at_night(self):
        brain = make_brain()
        brain.is_daytime = MagicMock(return_value=False)
        brain.do_idle()
        self.assertEqual(brain.state, State.IDLE)

    def test_go_to_dock_transitions_to_charging(self):
        brain = make_brain()
        brain.state = State.GO_TO_DOCK
        brain.nav_target = (0.0, 0.0)
        brain.optitrack_pose = (0.05, 0.05, 0.0)  # Within 0.15m tolerance
        brain.gvf_active = True
        brain.do_go_to_dock()
        self.assertEqual(brain.state, State.CHARGING)

    def test_charging_to_idle_at_night(self):
        brain = make_brain()
        brain.battery_percent = 1.0
        brain.is_daytime = MagicMock(return_value=False)
        brain.do_charging()
        self.assertEqual(brain.state, State.IDLE)

    def test_charging_to_light_scan_daytime_no_spots(self):
        brain = make_brain()
        brain.battery_percent = 1.0
        brain.is_daytime = MagicMock(return_value=True)
        brain.get_best_sunspot = MagicMock(return_value=None)
        brain.do_charging()
        self.assertEqual(brain.state, State.LIGHT_SCAN)

    def test_go_to_water_transitions_to_dosing(self):
        brain = make_brain()
        brain.state = State.GO_TO_WATER
        brain.nav_target = (1.0, 1.0)
        brain.optitrack_pose = (1.0, 1.05, 0.0)
        brain.gvf_active = True
        brain.do_go_to_water()
        self.assertEqual(brain.state, State.DOSING)
        self.assertIsNotNone(brain.dose_start_time)


# ─── Sunspot Memory ────────────────────────────────────────────────────

class TestSunspotMemory(unittest.TestCase):

    def test_save_with_optitrack(self):
        brain = make_brain()
        brain.optitrack_pose = (1.5, 2.5, 0.3)
        brain.odom_pos = (0.5, 0.6)
        brain.get_brightness = MagicMock(return_value=180.0)
        brain.save_sunspot()
        self.assertEqual(len(brain.sunspot_memory), 1)
        self.assertEqual(brain.sunspot_memory[0]['optitrack_pos'], (1.5, 2.5))

    def test_save_without_optitrack(self):
        brain = make_brain()
        brain.optitrack_pose = None
        brain.odom_pos = (0.5, 0.6)
        brain.get_brightness = MagicMock(return_value=180.0)
        brain.save_sunspot()
        self.assertIsNone(brain.sunspot_memory[0]['optitrack_pos'])

    def test_best_sunspot_skips_no_optitrack(self):
        brain = make_brain()
        brain.sunspot_memory = [
            {'optitrack_pos': None, 'brightness': 200, 'fail_count': 0},
        ]
        self.assertIsNone(brain.get_best_sunspot())

    def test_best_sunspot_skips_high_fail_count(self):
        brain = make_brain()
        brain.sunspot_memory = [
            {'optitrack_pos': (1.0, 1.0), 'brightness': 200, 'fail_count': 3},
        ]
        self.assertIsNone(brain.get_best_sunspot())

    def test_best_sunspot_returns_brightest(self):
        brain = make_brain()
        brain.sunspot_memory = [
            {'optitrack_pos': (1.0, 1.0), 'brightness': 150, 'fail_count': 0},
            {'optitrack_pos': (2.0, 2.0), 'brightness': 200, 'fail_count': 0},
        ]
        self.assertEqual(brain.get_best_sunspot()['brightness'], 200)

    def test_memory_capped_at_max(self):
        brain = make_brain()
        brain.odom_pos = (0.0, 0.0)
        for i in range(7):
            brain.optitrack_pose = (float(i), 0.0, 0.0)
            brain.get_brightness = MagicMock(return_value=100.0 + i * 10)
            brain.save_sunspot()
        self.assertLessEqual(len(brain.sunspot_memory), 5)


# ─── Utilities ──────────────────────────────────────────────────────────

class TestAngleDiff(unittest.TestCase):

    def test_zero(self):
        brain = make_brain()
        self.assertAlmostEqual(brain.angle_diff(1.0, 1.0), 0.0)

    def test_positive(self):
        brain = make_brain()
        self.assertAlmostEqual(brain.angle_diff(1.0, 0.5), 0.5)

    def test_wraparound(self):
        brain = make_brain()
        diff = brain.angle_diff(-math.pi + 0.1, math.pi - 0.1)
        self.assertAlmostEqual(abs(diff), 0.2, places=5)


if __name__ == "__main__":
    unittest.main()
