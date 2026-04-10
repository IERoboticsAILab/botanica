#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Experiment Logger - Structured data logging for BOTanica technology validation.

Subscribes to all relevant ROS topics and writes 3 CSV files per session:
  1. plant_state.csv   - soil/light sensor readings (~5s interval)
  2. robot_state.csv   - pose, odometry, velocity, battery (1Hz)
  3. behavior_events.csv - state changes, nav goals, arrivals, etc. (on-change)

Plus a session_info.json with metadata (start time, params, end time).

All files go into ~/experiment_data/<session_id>/.
"""
import os
import csv
import json
import rospy
import math
from datetime import datetime

from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_publisher.msg import SensorData


def euler_from_quaternion(q):
    """Convert quaternion [x, y, z, w] to euler angles [roll, pitch, yaw]."""
    x, y, z, w = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class ExperimentLogger:
    def __init__(self):
        rospy.init_node("experiment_logger")

        # Session setup
        self.session_id = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.session_dir = os.path.expanduser(
            f"~/experiment_data/{self.session_id}"
        )
        os.makedirs(self.session_dir, exist_ok=True)

        # Open CSV writers
        self.plant_file, self.plant_writer = self._open_csv(
            "plant_state.csv",
            ["timestamp", "moisture_pct", "light_lux", "temp_c",
             "conductivity_us", "sensor_battery_pct"],
        )
        self.robot_file, self.robot_writer = self._open_csv(
            "robot_state.csv",
            ["timestamp", "optitrack_x", "optitrack_y", "optitrack_theta",
             "odom_x", "odom_y", "odom_theta", "vx", "vy", "omega",
             "battery_pct"],
        )
        self.event_file, self.event_writer = self._open_csv(
            "behavior_events.csv",
            ["timestamp", "event_type", "old_state", "new_state", "detail"],
        )

        # Latest values for robot_state sampling
        self.optitrack_x = ""
        self.optitrack_y = ""
        self.optitrack_theta = ""
        self.odom_x = ""
        self.odom_y = ""
        self.odom_theta = ""
        self.vx = ""
        self.vy = ""
        self.omega = ""
        self.battery_pct = ""

        # Subscribers
        rospy.Subscriber("/sensor_data", SensorData, self._sensor_cb)
        rospy.Subscriber(
            "/natnet_ros/umh_5/pose", PoseStamped, self._optitrack_cb
        )
        rospy.Subscriber("/odom", Odometry, self._odom_cb)
        rospy.Subscriber("/battery_level", Float32, self._battery_cb)
        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_cb)
        rospy.Subscriber(
            "/experiment/events", String, self._event_cb
        )

        # 1Hz timer for robot_state sampling
        rospy.Timer(rospy.Duration(1.0), self._write_robot_state)

        # Write session_info.json (start)
        self._write_session_info()

        # Clean shutdown
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            f"[ExperimentLogger] Session '{self.session_id}' "
            f"logging to {self.session_dir}"
        )

    # ---- CSV helpers ----

    def _open_csv(self, filename, headers):
        path = os.path.join(self.session_dir, filename)
        f = open(path, "w", newline="")
        writer = csv.writer(f)
        writer.writerow(headers)
        f.flush()
        return f, writer

    def _ts(self):
        """ISO-format timestamp for CSV rows."""
        return datetime.now().isoformat(timespec="milliseconds")

    # ---- Callbacks ----

    def _sensor_cb(self, msg):
        self.plant_writer.writerow([
            self._ts(),
            msg.moisture,
            msg.sunlight,
            msg.temperature,
            msg.fertility,
            msg.battery,
        ])
        self.plant_file.flush()

    def _optitrack_cb(self, msg):
        pos = msg.pose.position
        o = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.optitrack_x = f"{pos.x:.4f}"
        self.optitrack_y = f"{pos.y:.4f}"
        self.optitrack_theta = f"{yaw:.4f}"

    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        o = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.odom_x = f"{pos.x:.4f}"
        self.odom_y = f"{pos.y:.4f}"
        self.odom_theta = f"{yaw:.4f}"

    def _battery_cb(self, msg):
        self.battery_pct = f"{msg.data * 100:.1f}"

    def _cmd_vel_cb(self, msg):
        self.vx = f"{msg.linear.x:.4f}"
        self.vy = f"{msg.linear.y:.4f}"
        self.omega = f"{msg.angular.z:.4f}"

    def _event_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            rospy.logwarn(
                f"[ExperimentLogger] Bad JSON on /experiment/events: "
                f"{msg.data}"
            )
            return
        self.event_writer.writerow([
            data.get("timestamp", self._ts()),
            data.get("event_type", ""),
            data.get("old_state", ""),
            data.get("new_state", ""),
            json.dumps(data.get("detail", {})),
        ])
        self.event_file.flush()

    # ---- 1Hz robot state writer ----

    def _write_robot_state(self, _):
        self.robot_writer.writerow([
            self._ts(),
            self.optitrack_x,
            self.optitrack_y,
            self.optitrack_theta,
            self.odom_x,
            self.odom_y,
            self.odom_theta,
            self.vx,
            self.vy,
            self.omega,
            self.battery_pct,
        ])
        self.robot_file.flush()

    # ---- Session metadata ----

    def _write_session_info(self):
        info = {
            "session_id": self.session_id,
            "start_time": datetime.now().isoformat(),
            "ros_params": {
                "battery_low_threshold": rospy.get_param(
                    "/botanica_brain/battery_low_threshold", "N/A"
                ),
                "moisture_low_threshold": rospy.get_param(
                    "/botanica_brain/moisture_low_threshold", "N/A"
                ),
                "dock_x": rospy.get_param("/botanica_brain/dock_x", "N/A"),
                "dock_y": rospy.get_param("/botanica_brain/dock_y", "N/A"),
                "water_x": rospy.get_param("/botanica_brain/water_x", "N/A"),
                "water_y": rospy.get_param("/botanica_brain/water_y", "N/A"),
            },
        }
        path = os.path.join(self.session_dir, "session_info.json")
        with open(path, "w") as f:
            json.dump(info, f, indent=2)

    def _shutdown(self):
        rospy.loginfo("[ExperimentLogger] Shutting down, finalizing session.")
        # Update session_info with end time
        path = os.path.join(self.session_dir, "session_info.json")
        try:
            with open(path, "r") as f:
                info = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            info = {}
        info["end_time"] = datetime.now().isoformat()
        with open(path, "w") as f:
            json.dump(info, f, indent=2)

        # Close CSV files
        for fh in (self.plant_file, self.robot_file, self.event_file):
            try:
                fh.close()
            except Exception:
                pass


if __name__ == "__main__":
    ExperimentLogger()
    rospy.spin()
