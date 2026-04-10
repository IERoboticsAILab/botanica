#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel Multiplexer

Switches between two cmd_vel sources based on navigation mode:
- /cmd_vel_direct: Direct control from botanica_brain (light-seeking)
- /cmd_vel_gvf: GVF vectorfield_stack output (waypoint navigation)

Outputs to /cmd_vel which goes to the RoboMaster driver on the Pi.
"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import time


class CmdVelMux:
    def __init__(self):
        rospy.init_node("cmd_vel_mux")

        # Current mode: True = GVF, False = Direct
        self.use_gvf = False

        # Last received commands
        self.cmd_direct = Twist()
        self.cmd_gvf = Twist()

        # Timestamps for timeout
        self.last_direct_time = rospy.Time.now()
        self.last_gvf_time = rospy.Time.now()
        self.cmd_timeout = rospy.Duration(0.5)  # 500ms timeout

        # === DEBUG: Tracking ===
        self._debug_direct_count = 0
        self._debug_gvf_count = 0
        self._debug_pub_count = 0
        self._debug_timeout_count = 0
        self._debug_last_status_time = time.time()
        self._debug_is_turning = False
        self._debug_turn_start_time = None
        self._debug_last_angular = 0.0

        # Publisher - final cmd_vel to robot
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # === DEBUG: Status publisher ===
        self.debug_pub = rospy.Publisher("/debug/mux_status", String, queue_size=3)

        # Subscribers
        rospy.Subscriber("/cmd_vel_direct", Twist, self.direct_callback)
        rospy.Subscriber("/cmd_vel_gvf", Twist, self.gvf_callback)
        rospy.Subscriber("/nav_mode_gvf", Bool, self.mode_callback)

        rospy.loginfo("cmd_vel_mux initialized")
        rospy.loginfo("  Direct topic: /cmd_vel_direct")
        rospy.loginfo("  GVF topic: /cmd_vel_gvf")
        rospy.loginfo("  Output topic: /cmd_vel")
        rospy.loginfo("  Mode topic: /nav_mode_gvf")

        # Publish at 20Hz
        rospy.Timer(rospy.Duration(0.05), self.publish_cmd)
        rospy.spin()

    def direct_callback(self, msg):
        self.cmd_direct = msg
        self.last_direct_time = rospy.Time.now()
        self._debug_direct_count += 1

    def gvf_callback(self, msg):
        self.cmd_gvf = msg
        self.last_gvf_time = rospy.Time.now()
        self._debug_gvf_count += 1

    def mode_callback(self, msg):
        new_mode = msg.data
        if new_mode != self.use_gvf:
            self.use_gvf = new_mode
            mode_str = "GVF" if self.use_gvf else "DIRECT"
            rospy.loginfo(f"cmd_vel_mux: Switched to {mode_str} mode")

    def publish_cmd(self, _):
        now = rospy.Time.now()
        cmd = Twist()
        timed_out = False

        if self.use_gvf:
            # Use GVF command if recent
            if (now - self.last_gvf_time) < self.cmd_timeout:
                cmd = self.cmd_gvf
            else:
                timed_out = True
                rospy.logwarn_throttle(2, "GVF cmd_vel timeout - stopping")
        else:
            # Use direct command if recent
            if (now - self.last_direct_time) < self.cmd_timeout:
                cmd = self.cmd_direct
            else:
                timed_out = True
                rospy.logwarn_throttle(2, "Direct cmd_vel timeout - stopping")

        # === Track turning state ===
        is_turning = abs(cmd.angular.z) > 0.01
        was_turning = self._debug_is_turning

        if timed_out and was_turning:
            self._debug_timeout_count += 1
            rospy.logwarn(f"[MUX TIMEOUT] Turn interrupted by timeout! count={self._debug_timeout_count}")

        self._debug_is_turning = is_turning
        self._debug_last_angular = cmd.angular.z
        self._debug_pub_count += 1

        # === Periodic status ===
        now_time = time.time()
        if now_time - self._debug_last_status_time > 30.0:
            self._debug_last_status_time = now_time
            mode_str = "GVF" if self.use_gvf else "DIRECT"
            status = f"mode={mode_str} direct_rx={self._debug_direct_count} gvf_rx={self._debug_gvf_count} pub={self._debug_pub_count} timeouts={self._debug_timeout_count}"
            rospy.loginfo(f"[MUX STATUS] {status}")
            self.debug_pub.publish(String(data=status))

        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    CmdVelMux()
