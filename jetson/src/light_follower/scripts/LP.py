#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion

class LightPathFollower:
    BRIGHTNESS_SCAN_THRESHOLD = 150
    BRIGHTNESS_MOVE_THRESHOLD = 160
    MIN_BRIGHT_ANGLES = 5
    NORMAL_SPEED = 0.1
    BRIGHT_CONFIRM_COUNT = 3  # how many consecutive frames must be bright to stop early

    def __init__(self):
        rospy.init_node("light_path_follower")
        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.state = "SCAN"
        self.image = None
        self.yaw = 0.0
        self.start_yaw = None
        self.last_yaw = None
        self.accumulated_rotation = 0.0
        self.start_pos = None
        self.current_pos = (0, 0)
        self.brightness_log = []
        self.bright_counter = 0

        rospy.loginfo("Light Path Follower Initialized")

        rospy.loginfo("Waiting for /cmd_vel connection...")
        while self.cmd_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("/cmd_vel is connected.")

        rospy.Timer(rospy.Duration(0.1), self.update)
        rospy.spin()

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Image error: %s", e)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.yaw = yaw
        pos = msg.pose.pose.position
        self.current_pos = (pos.x, pos.y)

    def angle_diff(self, a, b):
        d = a - b
        while d > np.pi:
            d -= 2 * np.pi
        while d < -np.pi:
            d += 2 * np.pi
        return d

    def distance(self, p1, p2):
        return np.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def update(self, _):
        rospy.loginfo_throttle(2, "Current state: %s" % self.state)
        if self.image is None:
            rospy.logwarn_throttle(5, "Waiting for camera image...")
            return

        if self.state == "SCAN":
            self.do_scan()
        elif self.state == "ALIGN":
            self.do_align()
        elif self.state == "MOVE":
            self.do_move()
        elif self.state == "STOP":
            rospy.loginfo_throttle(5, "Robot stopped in well-lit area.")
            self.safe_publish(Twist())

    def do_scan(self):
        self.bright_counter = 0  # Reset brightness counter at the start of each scan
        if self.start_yaw is None:
            rospy.loginfo("Starting 360° scan...")
            self.start_yaw = self.yaw
            self.last_yaw = self.yaw
            self.accumulated_rotation = 0.0
            self.brightness_log = []

        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (21, 21), 0)
        avg_brightness = np.mean(blurred)

        rospy.loginfo_throttle(0.5, "Yaw: %.2f | Brightness: %.2f" % (self.yaw, avg_brightness))
        if abs(self.angle_diff(self.yaw, self.last_yaw)) > 0.01:
            self.brightness_log.append((self.yaw, avg_brightness))

        delta = self.angle_diff(self.yaw, self.last_yaw)
        self.accumulated_rotation += abs(delta)
        self.last_yaw = self.yaw

        if self.accumulated_rotation < 2 * np.pi:
            twist = Twist()
            twist.angular.z = 0.4
            self.safe_publish(twist)
        else:
            bright_angles = [b for _, b in self.brightness_log if b > self.BRIGHTNESS_SCAN_THRESHOLD]
            rospy.loginfo("Bright angles found: %d / %d", len(bright_angles), len(self.brightness_log))
            if len(bright_angles) >= self.MIN_BRIGHT_ANGLES:
                rospy.loginfo("Environment is well-lit. Stopping.")
                self.state = "STOP"
                self.safe_publish(Twist())
                return

            max_yaw = max(self.brightness_log, key=lambda x: x[1])[0]
            self.target_yaw = max_yaw
            rospy.loginfo("Scan complete. Brightest yaw: %.2f" % self.target_yaw)

            self.start_yaw = None
            self.accumulated_rotation = 0.0
            self.state = "ALIGN"

    def do_align(self):
        error = self.angle_diff(self.target_yaw, self.yaw)
        rospy.loginfo_throttle(0.5, "Aligning. Yaw error: %.2f" % error)
        if abs(error) > 0.05:
            twist = Twist()
            twist.angular.z = 0.3 if error > 0 else -0.3
            self.safe_publish(twist)
        else:
            rospy.loginfo("Aligned. Starting move.")
            self.start_pos = self.current_pos
            self.bright_counter = 0
            self.state = "MOVE"

    def do_move(self):
        dist = self.distance(self.current_pos, self.start_pos)
        rospy.loginfo_throttle(0.5, "Moving forward. Distance: %.2f" % dist)

        # Check real-time brightness
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (21, 21), 0)
        avg_brightness = np.mean(blurred)

        if avg_brightness > self.BRIGHTNESS_MOVE_THRESHOLD:
            self.bright_counter += 1
            rospy.loginfo_throttle(0.5, "Bright frame %d detected (%.2f)", self.bright_counter, avg_brightness)
            if self.bright_counter >= self.BRIGHT_CONFIRM_COUNT:
                rospy.loginfo("Brightness sustained. Switching to SCAN.")
                self.state = "SCAN"
                self.start_yaw = None
                self.start_pos = None
                self.safe_publish(Twist())
                return
        else:
            self.bright_counter = 0

        if dist < 1.0:
            twist = Twist()
            twist.linear.x = self.NORMAL_SPEED
            self.safe_publish(twist)
        else:
            rospy.loginfo("1m reached. Restarting scan.")
            self.state = "SCAN"
            self.start_yaw = None
            self.start_pos = None

    def safe_publish(self, twist):
        if self.cmd_pub.get_num_connections() > 0:
            self.cmd_pub.publish(twist)
        else:
            rospy.logwarn_throttle(2, "cmd_vel not connected. Skipping publish.")

if __name__ == "__main__":
    LightPathFollower()

