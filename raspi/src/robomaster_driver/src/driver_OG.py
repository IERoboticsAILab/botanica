#!/usr/bin/env python3
import rospy
from robomaster import robot
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import tf
from threading import Thread
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState

class RoboMasterDriver:
    def __init__(self):
        """Initialize the RoboMaster driver node"""
        rospy.loginfo("Starting RoboMaster driver initialization...")
        
        # Initialize robot connection
        self._robot = robot.Robot()
        self._connect_to_robot()
        
        # Initialize robot parameters
        self.wheel_radius = 0.05    # Wheel radius in meters
        self.robot_width = 0.32     # Distance between left and right wheels
        self.robot_length = 0.32    # Distance between front and back wheels
        self.max_rpm = 150         # Maximum RPM for safety
        
        # Scaling factors (tuned for FREE mode)
        self.linear_scale = 19.0    # Scaling for linear motion
        self.angular_scale = 8.0    # Scaling for rotational motion
        
        # Prepare odometry publishing
        self._pub_odom_tf = rospy.get_param("~publish_odom_tf", False)
        self._odom_include_attitude = rospy.get_param("~odom_include_attitude", False)
        self._tf_br = tf.TransformBroadcaster()
        
        # Safety timeout parameters
        self._last_cmd_time = rospy.Time.now()
        self._cmd_timeout = rospy.Duration(0.5)  # 500ms timeout
        
        # Initialize ROS publishers and subscribers
        self._setup_ros_interface()
        
        # Store latest sensor data
        self._last_attitude = Quaternion(1, 0, 0, 0)  # Identity quaternion
        self._current_attitude = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self._standing = True
        
        # Add gravity constant
        self.GRAVITY = 9.861980434  # Local gravity constant
        
        # Add joint state publisher
        self._pub_joints = rospy.Publisher("joint_states", JointState, queue_size=3)
        
        # Subscribe to gimbal angles
        self._robot.gimbal.sub_angle(freq=50, callback=self._gimbal_angle_callback)
        
        rospy.loginfo("RoboMaster driver initialization completed")

    def _connect_to_robot(self):
        """Establish connection to the RoboMaster robot"""
        try:
            # Get connection mode from ROS parameter
            conn_mode = rospy.get_param("~connection_mode", "WIFI")
            
            if conn_mode == "WIFI":
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

    def _setup_ros_interface(self):
        """Setup ROS publishers and subscribers"""
        # Publishers
        self._odom_pub = rospy.Publisher("odom", Odometry, queue_size=3)
        self._imu_pub = rospy.Publisher("imu/data", Imu, queue_size=3)
        
        # Subscribers
        self._cmd_vel_sub = rospy.Subscriber(
            "cmd_vel", 
            Twist, 
            self._cmd_vel_callback, 
            queue_size=3
        )
        
        # Setup robot subscriptions
        self._robot.chassis.sub_position(cs=1, freq=50, callback=self._position_callback)
        self._robot.chassis.sub_attitude(freq=50, callback=self._attitude_callback)
        self._robot.chassis.sub_imu(freq=50, callback=self._imu_callback)

    def _cmd_vel_callback(self, msg):
        """
        Handle velocity commands from ROS
        msg: geometry_msgs/Twist
            linear.x: forward/backward velocity (m/s)
            linear.y: left/right velocity (m/s)
            angular.z: rotational velocity (rad/s)
        """
        try:
            # Debug print incoming command
            rospy.loginfo(f"Received cmd_vel - linear: ({msg.linear.x}, {msg.linear.y}), angular: {msg.angular.z}")
            
            # Update standing state based on command
            if msg.linear.x != 0 or msg.linear.y != 0 or msg.angular.z != 0:
                self._standing = False
            
            # Handle zero velocity case
            if msg.linear.x == 0 and msg.linear.y == 0 and msg.angular.z == 0:
                rospy.loginfo("Sending zero velocity command")
                self._robot.chassis.drive_speed(x=0, y=0, z=0, timeout=1)
                self._standing = True
                return

            # Handle non-zero velocity case
            if not self._standing:
                # Convert angular velocity from rad/s to deg/s with increased scaling
                # Multiply by 3.0 to make rotation more responsive
                angular_deg = -msg.angular.z * 180.0 / np.pi * 3.0
                
                # Scale linear velocities
                linear_x = msg.linear.x * 1.5  # Scale forward/backward
                linear_y = -msg.linear.y * 1.5  # Scale left/right
                
                # Debug print what we're sending to the robot
                rospy.loginfo(f"Sending to robot - x: {linear_x}, y: {linear_y}, z: {angular_deg}")
                
                self._robot.chassis.drive_speed(
                    x=linear_x,
                    y=linear_y,
                    z=angular_deg,
                    timeout=1
                )
                
        except Exception as e:
            rospy.logerr(f"Failed to send velocity command: {e}")
            # Emergency stop on error
            self._robot.chassis.drive_speed(x=0, y=0, z=0, timeout=1)

    def _gimbal_angle_callback(self, data):
        """
        Handle gimbal angle updates
        data: tuple (pitch, yaw, pitch_ground, yaw_ground)
        angles are in degrees
        """
        pitch, yaw, _, _ = data
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["gimbal_yaw_joint", "gimbal_pitch_joint"]
        # Convert degrees to radians and invert as needed
        msg.position = [-yaw/180.0*np.pi, -pitch/180.0*np.pi]
        self._pub_joints.publish(msg)

    def _attitude_callback(self, data):
        """
        Handle attitude updates from the robot
        data: tuple (yaw, pitch, roll) in degrees
        Using quaternion composition for more accurate orientation
        """
        yaw, pitch, roll = data
        
        # Create quaternion from Euler angles using axis-angle representation
        # Note: negative angles to convert from RoboMaster to ROS convention
        self._last_attitude = (Quaternion(axis=(0, 0, 1), degrees=-yaw) * 
                             Quaternion(axis=(0, 1, 0), degrees=-pitch) * 
                             Quaternion(axis=(1, 0, 0), degrees=roll))
        
        # Store angles in radians for other uses
        self._current_attitude['yaw'] = np.radians(-yaw)
        self._current_attitude['pitch'] = np.radians(-pitch)
        self._current_attitude['roll'] = np.radians(roll)

    def _imu_callback(self, data):
        """
        Handle IMU data from the robot
        data: tuple (ax, ay, az, wx, wy, wz)
        - Accelerations are in g's
        - Angular velocities are in rad/s
        """
        ax, ay, az, wx, wy, wz = data
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu_link"

        # Convert accelerations from g's to m/s^2 and apply correct orientations
        # Note: RoboMaster uses different coordinate convention
        msg.linear_acceleration.x = ax * self.GRAVITY
        msg.linear_acceleration.y = -ay * self.GRAVITY  # Flip Y axis
        msg.linear_acceleration.z = -az * self.GRAVITY  # Flip Z axis

        # Angular velocities (already in rad/s)
        msg.angular_velocity.x = wx
        msg.angular_velocity.y = wy
        msg.angular_velocity.z = wz

        # Use absolute orientation from last attitude
        if self._last_attitude is not None:
            msg.orientation.x = self._last_attitude.x
            msg.orientation.y = self._last_attitude.y
            msg.orientation.z = self._last_attitude.z
            msg.orientation.w = self._last_attitude.w
        else:
            # Use identity quaternion if no orientation data available
            msg.orientation.w = 1.0
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0

        # Add covariance matrices
        msg.orientation_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        msg.angular_velocity_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        msg.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]

        self._imu_pub.publish(msg)

    def _position_callback(self, data):
        """Handle position updates from the robot"""
        # Check for command timeout
        if (rospy.Time.now() - self._last_cmd_time) > self._cmd_timeout:
            self._robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        
        # Create and publish odometry message
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        # Set position
        x, y, z = data
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Set orientation if attitude data is available and enabled
        if self._odom_include_attitude and self._last_attitude is not None:
            qx, qy, qz, qw = self._euler_to_quaternion(
                yaw=self._current_attitude['yaw'],
                pitch=self._current_attitude['pitch'],
                roll=self._current_attitude['roll']
            )
            msg.pose.pose.orientation.w = qw
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
        else:
            # Use identity quaternion if no attitude data
            msg.pose.pose.orientation.w = 1.0
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = 0.0
        
        # Publish odometry message
        self._odom_pub.publish(msg)
        
        # Publish transform if enabled
        if self._pub_odom_tf:
            self._tf_br.sendTransform(
                (x, y, 0),
                (msg.pose.pose.orientation.x,
                 msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z,
                 msg.pose.pose.orientation.w),
                msg.header.stamp,
                msg.child_frame_id,
                msg.header.frame_id
            )

    def _euler_to_quaternion(self, yaw, pitch, roll):
        """
        Convert euler angles to quaternion.
        All angles are in radians.
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]

    def shutdown(self):
        """Clean shutdown procedure"""
        rospy.loginfo("Shutting down RoboMaster driver...")
        try:
            # Stop all motion
            self._robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            rospy.sleep(0.1)  # Short delay to ensure command is sent
            # Close connection
            self._robot.close()
            rospy.loginfo("RoboMaster shutdown completed")
        except Exception as e:
            rospy.logerr(f"Error during shutdown: {e}")

if __name__ == "__main__":
    rospy.init_node("robomaster_driver")
    driver = RoboMasterDriver()
    rospy.on_shutdown(driver.shutdown)
    rospy.spin()