#!/usr/bin/env python3
import rospy
from robomaster import robot, config
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from threading import Thread
from cv_bridge import CvBridge
import numpy as np
import tf
import tf.transformations
import time

class RoboMasterDriver:
    def __init__(self):
        """Initialize the RoboMaster driver node"""
        rospy.loginfo("Starting RoboMaster driver initialization...")

        # Initialize robot
        self._robot = robot.Robot()
        self._connect_to_robot()

        # Robot parameters
        self.wheel_radius = 0.05    # Wheel radius in meters
        self.robot_width = 0.32     # Distance between left and right wheels
        self.robot_length = 0.32    # Distance between front and back wheels

        # Safety timeout parameters
        self._last_cmd_time = rospy.Time.now()
        self._cmd_timeout = rospy.Duration(0.5)  # 500ms timeout
        self._timed_out = False  # track whether we already sent the stop

        # TF broadcaster for odom -> base_link
        self._tf_br = tf.TransformBroadcaster()

        # Current state
        self._current_yaw = 0.0

        # === DEBUG: Turn tracking ===
        self._turn_start_yaw = None
        self._turn_accumulated = 0.0
        self._turn_last_yaw = None
        self._turn_cmd_count = 0
        self._turn_timeout_count = 0
        self._last_angular_cmd = 0.0
        self._cmd_received_count = 0
        self._cmd_zero_count = 0
        self._last_debug_time = time.time()
        
        # Publishers
        self._odom_pub = rospy.Publisher("odom", Odometry, queue_size=3)
        self._imu_pub = rospy.Publisher("imu/data", Imu, queue_size=3)
        self._joint_pub = rospy.Publisher("joint_states", JointState, queue_size=3)
        self._battery_pub = rospy.Publisher("battery_level", Float32, queue_size=3)

        # === DEBUG publishers ===
        self._debug_yaw_pub = rospy.Publisher("debug/yaw", Float32, queue_size=3)
        self._debug_turn_pub = rospy.Publisher("debug/turn_status", String, queue_size=3)
        
        # Subscribers
        self._cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self._cmd_vel_callback, queue_size=3)
        
        # Robot subscriptions
        self._robot.chassis.sub_position(cs=1, freq=50, callback=self._position_callback)
        self._robot.chassis.sub_attitude(freq=50, callback=self._attitude_callback)
        self._robot.chassis.sub_imu(freq=50, callback=self._imu_callback)
        self._robot.gimbal.sub_angle(freq=50, callback=self._gimbal_angle_callback)
        self._robot.battery.sub_battery_info(freq=1, callback=self._battery_callback)
        
        rospy.loginfo("RoboMaster driver initialization completed")

    def _connect_to_robot(self):
        """Establish connection to the RoboMaster robot"""
        try:
            # Get connection mode from ROS parameter
            conn_mode = rospy.get_param("~connection_mode", "USB")
            robot_ip = rospy.get_param("~robot_ip", "")

            if conn_mode == "WIFI":
                local_ip = rospy.get_param("~local_ip", "")
                if robot_ip:
                    config.ROBOT_IP_STR = robot_ip
                if local_ip:
                    config.LOCAL_IP_STR = local_ip
                    rospy.loginfo(f"Using local IP: {local_ip}")
                self._robot.initialize(conn_type="sta")
            elif conn_mode == "USB":
                self._robot.initialize(conn_type="rndis")
            else:
                rospy.logerr(f"Unknown connection mode: {conn_mode}")
                return
            
            # Set robot to FREE mode
            self._robot.set_robot_mode(mode=robot.FREE)
            
            # Initialize with zero velocity
            self._robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            
            rospy.loginfo("Successfully connected to RoboMaster")
            
        except Exception as e:
            rospy.logerr(f"Failed to connect to robot: {e}")
            raise

    def _cmd_vel_callback(self, msg):
        """
        Raw velocity command handler
        msg: geometry_msgs/Twist
        """
        try:
            self._last_cmd_time = rospy.Time.now()
            self._timed_out = False
            self._cmd_received_count += 1

            # Convert angular velocity from rad/s to deg/s
            # Negate because RoboMaster SDK uses CW-positive, ROS uses CCW-positive
            angular_deg = -msg.angular.z * 180.0 / np.pi

            # === DEBUG: Track turn commands ===
            is_turning = abs(msg.angular.z) > 0.01
            was_turning = abs(self._last_angular_cmd) > 0.01

            if is_turning and not was_turning:
                # Turn just started
                self._turn_start_yaw = self._current_yaw if self._current_yaw is not None else 0.0
                self._turn_accumulated = 0.0
                self._turn_last_yaw = self._current_yaw if self._current_yaw is not None else 0.0
                self._turn_cmd_count = 0
                yaw_str = f"{self._current_yaw:.2f}" if self._current_yaw is not None else "N/A"
                rospy.loginfo(f"[TURN START] yaw={yaw_str}° angular_cmd={angular_deg:.2f}°/s")

            if is_turning:
                self._turn_cmd_count += 1
                # Log every 10th command to avoid spam
                if self._turn_cmd_count % 10 == 0:
                    yaw_str = f"{self._current_yaw:.2f}" if self._current_yaw is not None else "N/A"
                    rospy.loginfo(f"[TURN] cmd#{self._turn_cmd_count} yaw={yaw_str}° accumulated={self._turn_accumulated:.2f}° angular_cmd={angular_deg:.2f}°/s")

            if was_turning and not is_turning:
                # Turn just ended
                if self._turn_start_yaw is not None:
                    rospy.loginfo(f"[TURN END] start_yaw={self._turn_start_yaw:.2f}° end_yaw={self._current_yaw:.2f}° accumulated={self._turn_accumulated:.2f}° total_cmds={self._turn_cmd_count}")
                self._turn_start_yaw = None

            if not is_turning:
                self._cmd_zero_count += 1
            else:
                self._cmd_zero_count = 0

            self._last_angular_cmd = msg.angular.z

            # === DEBUG: Periodic summary ===
            now = time.time()
            if now - self._last_debug_time > 2.0:
                self._last_debug_time = now
                yaw_str = f"{self._current_yaw:.1f}" if self._current_yaw is not None else "N/A"
                status = f"cmds={self._cmd_received_count} yaw={yaw_str}° timeouts={self._turn_timeout_count}"
                rospy.loginfo(f"[DRIVER STATUS] {status}")
                self._debug_turn_pub.publish(String(data=status))

            self._robot.chassis.drive_speed(
                x=msg.linear.x,
                y=msg.linear.y,
                z=angular_deg,
                timeout=1
            )

        except Exception as e:
            rospy.logerr(f"Velocity command failed: {e}")
            self._robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

    def _attitude_callback(self, data):
        """Raw attitude data handler"""
        yaw, pitch, roll = data

        # === DEBUG: Track yaw changes during turn ===
        old_yaw = self._current_yaw
        # Negate because RoboMaster SDK uses CW-positive, ROS uses CCW-positive
        self._current_yaw = -yaw

        # Publish debug yaw
        self._debug_yaw_pub.publish(Float32(data=yaw))

        # Track accumulated rotation if we're in a turn
        if self._turn_last_yaw is not None and self._turn_start_yaw is not None:
            # Calculate delta with wraparound handling (-180 to 180)
            delta = yaw - self._turn_last_yaw
            if delta > 180:
                delta -= 360
            elif delta < -180:
                delta += 360
            self._turn_accumulated += abs(delta)
            self._turn_last_yaw = yaw

        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu_link"

        # Convert degrees to radians for quaternion
        yaw_rad = yaw * np.pi / 180.0
        pitch_rad = pitch * np.pi / 180.0
        roll_rad = roll * np.pi / 180.0

        # Create quaternion from Euler angles
        cy = np.cos(yaw_rad * 0.5)
        sy = np.sin(yaw_rad * 0.5)
        cp = np.cos(pitch_rad * 0.5)
        sp = np.sin(pitch_rad * 0.5)
        cr = np.cos(roll_rad * 0.5)
        sr = np.sin(roll_rad * 0.5)

        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy

        self._imu_pub.publish(msg)

    def _imu_callback(self, data):
        """Raw IMU data handler"""
        ax, ay, az, wx, wy, wz = data
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu_link"
        
        # Raw accelerations (in g's)
        msg.linear_acceleration.x = ax * 9.81
        msg.linear_acceleration.y = ay * 9.81
        msg.linear_acceleration.z = az * 9.81
        
        # Raw angular velocities
        msg.angular_velocity.x = wx
        msg.angular_velocity.y = wy
        msg.angular_velocity.z = wz
        
        # Default orientation
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
            
        self._imu_pub.publish(msg)

    def _position_callback(self, data):
        """Raw position data handler"""
        # Check for command timeout — only send stop once when timeout first triggers
        if (rospy.Time.now() - self._last_cmd_time) > self._cmd_timeout:
            if not self._timed_out:
                self._timed_out = True
                # === DEBUG: Log timeout ===
                if self._turn_start_yaw is not None:
                    self._turn_timeout_count += 1
                    rospy.logwarn(f"[TIMEOUT] Turn interrupted! accumulated={self._turn_accumulated:.2f}° timeout_count={self._turn_timeout_count}")
                    self._turn_start_yaw = None
                self._robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        
        x, y, z = data
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        
        # Use yaw from IMU for odometry orientation
        yaw_rad = self._current_yaw * np.pi / 180.0
        msg.pose.pose.orientation.w = np.cos(yaw_rad/2)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = np.sin(yaw_rad/2)
            
        self._odom_pub.publish(msg)

        # Publish odom -> base_link TF transform
        self._tf_br.sendTransform(
            (x, y, z),
            (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w),
            msg.header.stamp,
            "base_link",
            "odom"
        )

    def _gimbal_angle_callback(self, data):
        """Raw gimbal angle handler"""
        pitch, yaw, _, _ = data
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["gimbal_yaw_joint", "gimbal_pitch_joint"]
        msg.position = [yaw/180.0*np.pi, pitch/180.0*np.pi]  # Convert to radians
        self._joint_pub.publish(msg)

    def _battery_callback(self, data):
        """Battery info handler from RoboMaster"""
        percent = data
        rospy.loginfo_throttle(10, f"[BATTERY] {percent}%")
        self._battery_pub.publish(Float32(data=float(percent) / 100.0))

    def shutdown(self):
        """Clean shutdown"""
        try:
            self._robot.chassis.drive_speed(x=0, y=0, z=0, timeout=1)
            self._robot.close()
        except:
            pass

if __name__ == "__main__":
    rospy.init_node("robomaster_driver")
    driver = RoboMasterDriver()
    rospy.on_shutdown(driver.shutdown)
    rospy.spin()
