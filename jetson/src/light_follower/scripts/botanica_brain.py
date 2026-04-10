#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BOTanica Brain - Priority-based behavior controller with GVF navigation

Priority 1 (highest): Battery low -> Go to dock, charge until 100%
Priority 2: Soil moisture low -> Go to water station, dose, resume light-seeking
Priority 3 (default): Daytime -> Seek brightest light
         Otherwise: Idle

Navigation is handled by vectorfield_stack (GVF).
This node publishes target paths and monitors arrival.
Light-seeking uses direct cmd_vel control (no GVF needed for rotation/short moves).
"""
import rospy
import rospkg
import cv2
import json
import os
import numpy as np
from datetime import datetime
from enum import Enum

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped, Point32
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float32
import math
import time

# cv_bridge replacement for Python 3
def imgmsg_to_cv2(img_msg, desired_encoding="bgr8"):
    """Convert ROS Image message to OpenCV image without cv_bridge"""
    dtype = np.uint8
    img = np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width, -1)
    if img_msg.encoding == "rgb8" and desired_encoding == "bgr8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "bgr8" and desired_encoding == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img

def depth_imgmsg_to_cv2(img_msg):
    """Convert ROS depth Image message to numpy array (in meters)"""
    # Depth is typically 16UC1 (16-bit unsigned, single channel) in millimeters
    if img_msg.encoding == "16UC1":
        dtype = np.uint16
        img = np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width)
        # Convert from mm to meters
        return img.astype(np.float32) / 1000.0
    elif img_msg.encoding == "32FC1":
        dtype = np.float32
        img = np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width)
        return img
    else:
        rospy.logwarn(f"Unknown depth encoding: {img_msg.encoding}")
        return None

def euler_from_quaternion(q):
    """Convert quaternion [x, y, z, w] to euler angles [roll, pitch, yaw]"""
    x, y, z, w = q
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

# Import custom sensor message
from sensor_publisher.msg import SensorData

# Import vectorfield_stack path message
from distancefield.msg import Path as GVFPath


class State(Enum):
    IDLE = "IDLE"
    LIGHT_SCAN = "LIGHT_SCAN"
    LIGHT_ALIGN = "LIGHT_ALIGN"
    LIGHT_MOVE = "LIGHT_MOVE"
    GO_TO_WATER = "GO_TO_WATER"
    DOSING = "DOSING"
    REVERSING = "REVERSING"
    GO_TO_DOCK = "GO_TO_DOCK"
    CHARGING = "CHARGING"
    SUNBATHING = "SUNBATHING"
    GO_TO_SUNSPOT = "GO_TO_SUNSPOT"


class NavigationMode(Enum):
    GVF = "GVF"           # Let vectorfield_stack control cmd_vel
    DIRECT = "DIRECT"     # This node controls cmd_vel directly


class BOTanicaBrain:
    # === DEFAULT CONFIGURATION ===
    # (Can be overridden via ROS parameters)

    # Thresholds
    DEFAULT_BATTERY_LOW_THRESHOLD = 0.20   # 20% - go to dock
    DEFAULT_BATTERY_FULL = 1.0             # 100% - leave dock
    DEFAULT_MOISTURE_LOW_THRESHOLD = 20    # 20% - go to water

    # Time-based day detection (24h format)
    DEFAULT_DAY_START_HOUR = 6             # 6 AM
    DEFAULT_DAY_END_HOUR = 20              # 8 PM

    # Default waypoints in OptiTrack frame (x, y)
    DEFAULT_DOCK_COORDS = (0.519, 4.284)
    DEFAULT_WATER_COORDS = (2.736, 4.160)

    # OptiTrack frame name (must match your natnet_ros setup)
    OPTITRACK_FRAME = "world"  # OptiTrack world frame

    # Navigation parameters
    DEFAULT_ARRIVAL_TOLERANCE = 0.15       # meters - how close to be "arrived" (sunspots)
    DOCK_ARRIVAL_TOLERANCE = 0.15          # meters - dock is a square, more forgiving
    WATER_ARRIVAL_TOLERANCE = 0.15         # meters - close enough for ultrasonic doser to detect

    # Light-seeking parameters
    BRIGHTNESS_SCAN_THRESHOLD = 150
    BRIGHTNESS_MOVE_THRESHOLD = 200
    MIN_BRIGHT_ANGLES = 5
    LIGHT_MOVE_SPEED = 0.1
    BRIGHT_CONFIRM_COUNT = 3
    MIN_MOVE_BEFORE_PARK = 0.20            # meters - must move at least 20cm before parking at sunspot

    # Obstacle avoidance parameters
    OBSTACLE_STOP_DISTANCE = 0.25          # meters - stop if obstacle closer than this
    OBSTACLE_SLOW_DISTANCE = 0.5           # meters - slow down if obstacle closer than this
    OBSTACLE_CHECK_WIDTH = 0.3             # fraction of image width to check (center 30%)

    # Dosing duration
    DEFAULT_DOSE_DURATION = 30.0           # seconds to wait for ultrasonic doser
    REVERSE_DURATION = 3.0                 # seconds to reverse after dosing
    REVERSE_SPEED = -0.10                  # m/s backward speed

    # Sunbathing parameters
    SUNBATHING_RECHECK_INTERVAL = 5.0      # seconds between brightness checks
    SUNBATHING_DROP_THRESHOLD = 100        # brightness below this = "light dropped"
    SUNBATHING_DROP_COUNT = 6              # consecutive low checks before rescanning
    MAX_SUNSPOTS = 5                       # max remembered positions
    SUNSPOT_ARRIVAL_BRIGHTNESS = 130       # min brightness to confirm sunspot is still good

    # Window memory parameters
    WINDOW_PROMOTION_MINUTES = 30          # min sunbathing time to promote spot to "window"
    WINDOW_FAIL_DEMOTE_COUNT = 3           # consecutive failures to demote a window
    MEMORY_STALE_DAYS = 7                  # days without confirmation before reducing confidence
    MEMORY_FILE = "sunspot_memory.json"    # filename inside package memory/ dir

    def __init__(self):
        rospy.init_node("botanica_brain")

        # Load configuration from ROS parameters (with defaults)
        self.BATTERY_LOW_THRESHOLD = rospy.get_param("~battery_low_threshold", self.DEFAULT_BATTERY_LOW_THRESHOLD)
        self.BATTERY_FULL = rospy.get_param("~battery_full", self.DEFAULT_BATTERY_FULL)
        self.MOISTURE_LOW_THRESHOLD = rospy.get_param("~moisture_low_threshold", self.DEFAULT_MOISTURE_LOW_THRESHOLD)
        self.DAY_START_HOUR = rospy.get_param("~day_start_hour", self.DEFAULT_DAY_START_HOUR)
        self.DAY_END_HOUR = rospy.get_param("~day_end_hour", self.DEFAULT_DAY_END_HOUR)
        self.ARRIVAL_TOLERANCE = rospy.get_param("~arrival_tolerance", self.DEFAULT_ARRIVAL_TOLERANCE)
        self.DOSE_DURATION = rospy.get_param("~dose_duration", self.DEFAULT_DOSE_DURATION)

        # Load waypoints from ROS parameters
        dock_x = rospy.get_param("~dock_x", self.DEFAULT_DOCK_COORDS[0])
        dock_y = rospy.get_param("~dock_y", self.DEFAULT_DOCK_COORDS[1])
        water_x = rospy.get_param("~water_x", self.DEFAULT_WATER_COORDS[0])
        water_y = rospy.get_param("~water_y", self.DEFAULT_WATER_COORDS[1])
        self.DOCK_COORDS = (dock_x, dock_y)
        self.WATER_COORDS = (water_x, water_y)

        # Using custom imgmsg_to_cv2 instead of CvBridge for Python 3 compatibility
        self.state = State.IDLE
        self.nav_mode = NavigationMode.DIRECT

        # Sensor data
        self.battery_percent = 1.0     # Start assuming full
        self.soil_moisture = 100       # Start assuming watered
        self.image = None
        self.depth_image = None        # Depth image for obstacle avoidance
        self.min_obstacle_dist = float('inf')  # Minimum distance to obstacle

        # Position data — two distinct coordinate frames, never mixed
        # OptiTrack: used ONLY for GVF waypoint navigation (dock, water, sunspots)
        self.optitrack_pose = None     # (x, y, yaw) from OptiTrack, or None if unavailable
        # Odom: used for light-seeking (scan, align, move) and as fallback display
        self.odom_yaw = 0.0            # Yaw from robot odometry (for light seeking)
        self.odom_pos = (0.0, 0.0)     # Position from robot odometry (for light seeking)

        # Frame alignment: offset from odom yaw to world yaw
        # world_yaw = odom_yaw + yaw_offset
        self.yaw_offset = None         # Estimated once robot moves enough
        self._prev_optitrack_xy = None # Previous OptiTrack position for bearing estimation
        self._prev_odom_yaw_at_sample = None  # odom_yaw when prev position was recorded

        # Light-seeking state
        self.scan_start_yaw = None
        self.scan_last_yaw = None
        self.scan_accumulated_rotation = 0.0
        self.scan_start_time = None
        self.brightness_log = []
        self.target_yaw = 0.0
        self.move_start_pos = None
        self.bright_counter = 0

        # === DEBUG: Turn tracking ===
        self._debug_cmd_count = 0
        self._debug_last_cmd_time = time.time()
        self._debug_yaw_samples = []
        self._debug_scan_cmd_gaps = []  # Track gaps between commands

        # Navigation state
        self.nav_target = None
        self.gvf_active = False

        # Dosing state
        self.dose_start_time = None

        # Sunspot memory (persistent across sessions)
        self.memory_dir = os.path.join(
            rospkg.RosPack().get_path('light_follower'), 'memory')
        os.makedirs(self.memory_dir, exist_ok=True)
        self.sunspot_memory = []
        self.current_sunspot = None
        self.pre_interrupt_sunspot = None  # Saved sunspot to return to after dock/water
        # Disabled: sunspot memory not useful until robot can stay 30+ min
        # self.load_memory()

        # Sunbathing state
        self.sunbathing_drop_counter = 0
        self.sunbathing_last_check = None
        self.sunbathing_start_time = None  # track duration for window promotion

        # === PUBLISHERS ===
        # Direct cmd_vel for light-seeking (scan/align/move)
        self.cmd_pub = rospy.Publisher("/cmd_vel_direct", Twist, queue_size=10)

        # Path publisher for vectorfield_stack GVF navigation
        self.path_pub = rospy.Publisher("/gvf/path", GVFPath, queue_size=1)

        # Mux control: switch between GVF and direct control
        # True = use GVF cmd_vel, False = use direct cmd_vel
        self.nav_mode_pub = rospy.Publisher("/nav_mode_gvf", Bool, queue_size=1)

        # === DEBUG: Scan status publisher ===
        self.debug_scan_pub = rospy.Publisher("/debug/scan_status", String, queue_size=3)

        # Experiment event publisher (JSON-encoded strings)
        self.event_pub = rospy.Publisher("/experiment/events", String, queue_size=50)

        # === SUBSCRIBERS ===
        # Battery from RoboMaster via Pi
        rospy.Subscriber("/battery_level", Float32, self.battery_callback)

        # Soil moisture from BLE sensor
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)

        # Position from OptiTrack (adjust topic/type as needed)
        rospy.Subscriber("/natnet_ros/umh_5/pose", PoseStamped, self.pose_callback)

        # Camera for light detection
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # Depth camera for obstacle avoidance
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)

        # Odometry as backup / for yaw
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Manual state override (publish state name as string)
        rospy.Subscriber("/brain/override", String, self.override_callback)

        rospy.loginfo("BOTanica Brain initialized")
        rospy.loginfo(f"  Obstacle stop distance: {self.OBSTACLE_STOP_DISTANCE}m")
        rospy.loginfo(f"  Battery threshold: {self.BATTERY_LOW_THRESHOLD*100}%")
        rospy.loginfo(f"  Moisture threshold: {self.MOISTURE_LOW_THRESHOLD}%")
        rospy.loginfo(f"  Day hours: {self.DAY_START_HOUR}:00 - {self.DAY_END_HOUR}:00")
        rospy.loginfo(f"  Dock coords: {self.DOCK_COORDS}")
        rospy.loginfo(f"  Water coords: {self.WATER_COORDS}")

        # Main control loop at 10Hz
        rospy.Timer(rospy.Duration(0.1), self.update)
        rospy.spin()

    # === EXPERIMENT EVENTS ===

    def publish_event(self, event_type, detail=None):
        """Publish a JSON-encoded experiment event to /experiment/events."""
        msg_data = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "event_type": event_type,
            "old_state": "",
            "new_state": "",
            "detail": detail or {},
        }
        self.event_pub.publish(String(data=json.dumps(msg_data)))

    def set_state(self, new_state):
        """Transition state and publish a state_change event."""
        old = self.state
        self.state = new_state
        msg_data = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "event_type": "state_change",
            "old_state": old.value,
            "new_state": new_state.value,
            "detail": {},
        }
        self.event_pub.publish(String(data=json.dumps(msg_data)))

    # === CALLBACKS ===

    def battery_callback(self, msg):
        self.battery_percent = msg.data  # 0.0 - 1.0

    def sensor_callback(self, msg):
        self.soil_moisture = msg.moisture  # 0-100%

    def pose_callback(self, msg):
        """OptiTrack pose callback — only used for GVF waypoint navigation"""
        pos = msg.pose.position
        orient = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.optitrack_pose = (pos.x, pos.y, yaw)

        # Calibrate yaw offset from straight-line movement direction.
        # The puck may be mounted rotated, so we can't use OptiTrack yaw directly.
        # Compare movement direction (from OptiTrack positions) to odom yaw.
        xy = (pos.x, pos.y)
        if self._prev_optitrack_xy is not None:
            if self.yaw_offset is None:
                dx = xy[0] - self._prev_optitrack_xy[0]
                dy = xy[1] - self._prev_optitrack_xy[1]
                moved = math.hypot(dx, dy)
                # Only calibrate if moved >15cm during forward driving (not scanning/spinning)
                if moved > 0.15 and self.state not in (State.LIGHT_SCAN, State.IDLE):
                    world_bearing = math.atan2(dy, dx)
                    odom_yaw_mid = (self._prev_odom_yaw_at_sample + self.odom_yaw) / 2.0
                    self.yaw_offset = self.angle_diff(world_bearing, odom_yaw_mid)
                    rospy.loginfo(f"[YAW OFFSET] Calibrated: {np.degrees(self.yaw_offset):.1f}° "
                                  f"(world_bearing={np.degrees(world_bearing):.1f}° odom_mid={np.degrees(odom_yaw_mid):.1f}°)")
                    self._prev_optitrack_xy = xy
                    self._prev_odom_yaw_at_sample = self.odom_yaw
                # Don't update reference while waiting to calibrate — need to accumulate distance
            else:
                # Already calibrated — keep reference fresh for potential recalibration
                self._prev_optitrack_xy = xy
                self._prev_odom_yaw_at_sample = self.odom_yaw
        else:
            self._prev_optitrack_xy = xy
            self._prev_odom_yaw_at_sample = self.odom_yaw

    def odom_callback(self, msg):
        """Odometry from RoboMaster — used ONLY for light-seeking (scan/align/move)"""
        orient = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.odom_yaw = yaw
        pos = msg.pose.pose.position
        self.odom_pos = (pos.x, pos.y)

    def image_callback(self, msg):
        try:
            self.image = imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Image error: {e}")

    def override_callback(self, msg):
        """Manual state override via /brain/override topic.

        Accepted commands:
            GO_TO_DOCK   - navigate to dock
            GO_TO_WATER  - navigate to water station
            LIGHT_SCAN   - start a fresh light scan
            SUNBATHING   - park and sunbathe (at current position)
            IDLE         - stop and idle
            STOP         - alias for IDLE
        """
        cmd = msg.data.strip().upper()
        rospy.logwarn(f"[OVERRIDE] Received manual command: '{cmd}'")

        # Stop whatever is happening
        if self.state == State.SUNBATHING:
            self.end_sunbathing()
        self.stop_gvf_navigation()
        self.reset_light_seeking()

        if cmd == "GO_TO_DOCK":
            self.set_state(State.GO_TO_DOCK)
            self.start_gvf_navigation(self.DOCK_COORDS)
        elif cmd == "GO_TO_WATER":
            self.set_state(State.GO_TO_WATER)
            self.start_gvf_navigation(self.WATER_COORDS)
        elif cmd == "LIGHT_SCAN":
            self.set_state(State.LIGHT_SCAN)
        elif cmd == "SUNBATHING":
            self.save_sunspot()
            self.set_state(State.SUNBATHING)
            self.sunbathing_last_check = None
            self.sunbathing_drop_counter = 0
        elif cmd in ("IDLE", "STOP"):
            self.set_state(State.IDLE)
        else:
            rospy.logwarn(f"[OVERRIDE] Unknown command: '{cmd}'. "
                          f"Valid: GO_TO_DOCK, GO_TO_WATER, LIGHT_SCAN, SUNBATHING, IDLE, STOP")
            return

        self.publish_event("manual_override", {"command": cmd})

    def depth_callback(self, msg):
        """Process depth image for obstacle detection"""
        try:
            depth = depth_imgmsg_to_cv2(msg)
            if depth is not None:
                self.depth_image = depth
                # Calculate minimum distance in center region of image
                self.min_obstacle_dist = self.get_min_obstacle_distance(depth)
        except Exception as e:
            rospy.logerr(f"Depth error: {e}")

    def get_min_obstacle_distance(self, depth):
        """Get minimum distance to obstacle in the center region of the depth image"""
        if depth is None:
            return float('inf')

        h, w = depth.shape
        # Check center portion of image (horizontally and vertically)
        # Vertical: use middle 60% to avoid floor and ceiling
        v_margin = int(h * 0.2)
        # Horizontal: use center region defined by OBSTACLE_CHECK_WIDTH
        margin = int(w * (1 - self.OBSTACLE_CHECK_WIDTH) / 2)
        center_region = depth[v_margin:h-v_margin, margin:w-margin]

        # Filter out invalid readings (0 or very large values)
        valid_depths = center_region[(center_region > 0.1) & (center_region < 10.0)]

        if len(valid_depths) == 0:
            return float('inf')

        # Return minimum distance (closest obstacle)
        return np.min(valid_depths)

    def is_obstacle_ahead(self):
        """Check if there's an obstacle too close ahead"""
        return self.min_obstacle_dist < self.OBSTACLE_STOP_DISTANCE

    def should_slow_down(self):
        """Check if we should slow down due to nearby obstacle"""
        return self.min_obstacle_dist < self.OBSTACLE_SLOW_DISTANCE

    # === PERSISTENT MEMORY ===

    def load_memory(self):
        """Load sunspot memory from disk."""
        path = os.path.join(self.memory_dir, self.MEMORY_FILE)
        if not os.path.exists(path):
            rospy.loginfo("[MEMORY] No saved memory found. Starting fresh.")
            return
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            self.sunspot_memory = data.get('sunspots', [])
            # Decay stale spots on load
            today = datetime.now().strftime('%Y-%m-%d')
            for spot in self.sunspot_memory:
                last = spot.get('last_confirmed')
                if last:
                    days_ago = (datetime.strptime(today, '%Y-%m-%d') -
                                datetime.strptime(last, '%Y-%m-%d')).days
                    if days_ago > self.MEMORY_STALE_DAYS:
                        spot['fail_count'] = spot.get('fail_count', 0) + 1
                        rospy.loginfo(f"[MEMORY] Stale spot (last confirmed {days_ago}d ago), "
                                      f"incremented fail_count to {spot['fail_count']}")
            rospy.loginfo(f"[MEMORY] Loaded {len(self.sunspot_memory)} sunspots from disk.")
            for i, s in enumerate(self.sunspot_memory):
                rospy.loginfo(f"  [{i}] pos={s.get('optitrack_pos')} window={s.get('is_window', False)} "
                              f"total_min={s.get('total_minutes', 0):.0f} "
                              f"hours={s.get('bright_hours', [])} "
                              f"fails={s.get('fail_count', 0)}")
        except Exception as e:
            rospy.logwarn(f"[MEMORY] Failed to load memory: {e}")

    def save_memory(self):
        """Persist sunspot memory to disk. Disabled until robot can stay 30+ min."""
        rospy.loginfo("[MEMORY] Save disabled (memory loading disabled).")
        return

    # === UTILITY FUNCTIONS ===

    def is_daytime(self):
        hour = datetime.now().hour
        return self.DAY_START_HOUR <= hour < self.DAY_END_HOUR

    def angle_diff(self, a, b):
        d = a - b
        while d > np.pi:
            d -= 2 * np.pi
        while d < -np.pi:
            d += 2 * np.pi
        return d

    def distance_to_optitrack(self, target):
        """Distance from current OptiTrack position to target. Returns inf if no OptiTrack."""
        if self.optitrack_pose is None:
            return float('inf')
        return np.hypot(self.optitrack_pose[0] - target[0],
                        self.optitrack_pose[1] - target[1])

    def get_brightness(self):
        if self.image is None:
            return 0
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (21, 21), 0)
        return np.mean(blurred)

    # === SUNSPOT MEMORY ===

    def save_sunspot(self):
        """Save current position as a known good sunspot."""
        brightness = self.get_brightness()
        optitrack_pos = list(self.optitrack_pose[:2]) if self.optitrack_pose else None
        odom_pos = list(self.odom_pos)
        now_hour = datetime.now().hour
        today = datetime.now().strftime('%Y-%m-%d')

        # Check if we already have a spot near this position (within arrival tolerance)
        existing = self._find_nearby_sunspot(optitrack_pos)
        if existing is not None:
            # Update existing spot instead of creating a duplicate
            existing['brightness'] = max(existing['brightness'], brightness)
            existing['fail_count'] = 0
            existing['last_confirmed'] = today
            if now_hour not in existing.get('bright_hours', []):
                existing.setdefault('bright_hours', []).append(now_hour)
                existing['bright_hours'].sort()
            self.current_sunspot = existing
            rospy.loginfo(f"[SUNSPOT] Updated existing sunspot: pos={optitrack_pos}, "
                          f"brightness={brightness:.0f}, hours={existing['bright_hours']}")
        else:
            spot = {
                'odom_pos': odom_pos,
                'optitrack_pos': optitrack_pos,
                'brightness': brightness,
                'fail_count': 0,
                'is_window': False,
                'total_minutes': 0.0,
                'bright_hours': [now_hour],
                'last_confirmed': today,
                'last_failed': None,
                'created': today,
            }
            self.sunspot_memory.append(spot)
            self.current_sunspot = spot

        # Cap list at MAX_SUNSPOTS, keeping windows first, then by brightness
        if len(self.sunspot_memory) > self.MAX_SUNSPOTS:
            self.sunspot_memory.sort(
                key=lambda s: (s.get('is_window', False), s.get('brightness', 0)),
                reverse=True)
            self.sunspot_memory = self.sunspot_memory[:self.MAX_SUNSPOTS]

        self.save_memory()
        rospy.loginfo(f"[SUNSPOT] Saved sunspot: optitrack={optitrack_pos}, brightness={brightness:.0f} "
                      f"(total: {len(self.sunspot_memory)})")
        self.publish_event("sunspot_saved", {
            "odom_pos": odom_pos,
            "optitrack_pos": optitrack_pos,
            "brightness": brightness,
            "total_spots": len(self.sunspot_memory),
            "hour": now_hour,
        })

    def _find_nearby_sunspot(self, optitrack_pos):
        """Find an existing sunspot near the given position."""
        if optitrack_pos is None:
            return None
        for spot in self.sunspot_memory:
            spos = spot.get('optitrack_pos')
            if spos is None:
                continue
            dist = np.hypot(spos[0] - optitrack_pos[0], spos[1] - optitrack_pos[1])
            if dist < self.ARRIVAL_TOLERANCE * 2:
                return spot
        return None

    def get_best_sunspot(self):
        """Return the best sunspot, preferring windows whose bright_hours match now.
        Returns None if no valid spot exists."""
        now_hour = datetime.now().hour
        valid = [s for s in self.sunspot_memory
                 if s.get('optitrack_pos') is not None
                 and s.get('fail_count', 0) < self.WINDOW_FAIL_DEMOTE_COUNT]
        if not valid:
            return None

        def score(spot):
            s = spot.get('brightness', 0)
            # Windows get a large bonus
            if spot.get('is_window', False):
                s += 200
            # Spots bright at the current hour get a bonus
            if now_hour in spot.get('bright_hours', []):
                s += 100
            return s

        return max(valid, key=score)

    # === NAVIGATION CONTROL ===

    def set_nav_mode(self, mode):
        """Switch between GVF and direct control"""
        self.nav_mode = mode
        msg = Bool()
        msg.data = (mode == NavigationMode.GVF)
        self.nav_mode_pub.publish(msg)
        rospy.loginfo(f"Navigation mode: {mode.value}")

    def publish_direct_cmd(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Publish direct velocity command (for light-seeking)"""
        if self.nav_mode != NavigationMode.DIRECT:
            self.set_nav_mode(NavigationMode.DIRECT)

        # === DEBUG: Track command timing ===
        now = time.time()
        gap = now - self._debug_last_cmd_time
        self._debug_last_cmd_time = now
        self._debug_cmd_count += 1

        # Warn if gap is too large (potential cause of timeout on raspi)
        if gap > 0.3 and abs(angular_z) > 0.01:
            rospy.logwarn_throttle(10, f"[CMD GAP] {gap*1000:.0f}ms between turn commands! cmd#{self._debug_cmd_count}")
            self._debug_scan_cmd_gaps.append(gap)

        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def stop(self):
        """Stop all movement"""
        self.publish_direct_cmd(0, 0, 0)

    # Waypoint navigation parameters
    NAV_LINEAR_SPEED = 0.15        # m/s cruise speed toward waypoint
    NAV_ANGULAR_GAIN = 1.5         # proportional gain for heading correction
    NAV_MAX_ANGULAR = 0.5          # rad/s max turning speed
    NAV_ALIGN_THRESHOLD = 0.3      # rad — align first if heading error exceeds this

    def start_gvf_navigation(self, target):
        """
        Start direct waypoint navigation to target (in OptiTrack frame).
        Uses proportional control with OptiTrack feedback.
        """
        if self.optitrack_pose is None:
            rospy.logwarn("Cannot start navigation: no OptiTrack pose available")
            return False

        self.nav_target = target
        self.gvf_active = True
        self.set_nav_mode(NavigationMode.DIRECT)

        # Reset calibration reference so it calibrates from fresh forward driving
        if self.yaw_offset is None:
            self._prev_optitrack_xy = (self.optitrack_pose[0], self.optitrack_pose[1])
            self._prev_odom_yaw_at_sample = self.odom_yaw

        rospy.loginfo(f"Navigation started: ({self.optitrack_pose[0]:.2f}, {self.optitrack_pose[1]:.2f}) -> ({target[0]:.2f}, {target[1]:.2f})")
        self.publish_event("nav_goal", {
            "from": [round(self.optitrack_pose[0], 3), round(self.optitrack_pose[1], 3)],
            "to": [round(target[0], 3), round(target[1], 3)],
        })
        return True

    def navigate_to_target(self):
        """Called each update cycle to drive toward nav_target using OptiTrack pose."""
        if self.nav_target is None or self.optitrack_pose is None:
            return

        if self.yaw_offset is None:
            # Drive forward slowly to bootstrap yaw offset calibration from OptiTrack movement
            rospy.logwarn_throttle(3, "[NAV] Calibrating yaw offset... creeping forward")
            self.publish_direct_cmd(linear_x=0.10, angular_z=0.0)
            return

        dx = self.nav_target[0] - self.optitrack_pose[0]
        dy = self.nav_target[1] - self.optitrack_pose[1]
        dist = math.hypot(dx, dy)

        # Desired heading toward target (in world frame)
        desired_yaw = math.atan2(dy, dx)
        # Convert odom yaw to world frame using calibrated offset
        current_yaw = self.odom_yaw + self.yaw_offset
        heading_error = self.angle_diff(desired_yaw, current_yaw)
        rospy.loginfo_throttle(15, f"[NAV] yaw_offset={np.degrees(self.yaw_offset):.1f}° odom_yaw={np.degrees(self.odom_yaw):.1f}° world_yaw={np.degrees(current_yaw):.1f}° desired={np.degrees(desired_yaw):.1f}°")

        # Proportional angular correction
        # When heading error is large (>90°), commit to turning one direction
        # to avoid oscillation near ±180°
        if abs(heading_error) > math.pi / 2:
            angular_cmd = self.NAV_MAX_ANGULAR  # always turn CCW until aligned
        else:
            angular_cmd = np.clip(self.NAV_ANGULAR_GAIN * heading_error,
                                  -self.NAV_MAX_ANGULAR, self.NAV_MAX_ANGULAR)

        # Only drive forward when roughly pointing at the target
        if abs(heading_error) > self.NAV_ALIGN_THRESHOLD:
            # Turn in place first
            self.publish_direct_cmd(linear_x=0.0, angular_z=angular_cmd)
            rospy.loginfo_throttle(5, f"[NAV] ALIGNING dist={dist:.2f}m heading_err={np.degrees(heading_error):.1f}°")
        else:
            # Drive forward with heading correction
            # Slow down as we approach
            speed = min(self.NAV_LINEAR_SPEED, self.NAV_LINEAR_SPEED * dist / 0.5)
            speed = max(speed, 0.05)  # minimum creep speed

            # Obstacle check only during light-seeking (GO_TO_SUNSPOT), not dock/water
            # The RealSense is unreliable (USB errors) and dock/water paths are known clear
            if self.state not in (State.GO_TO_DOCK, State.GO_TO_WATER):
                if self.is_obstacle_ahead():
                    rospy.logwarn(f"[NAV] Obstacle at {self.min_obstacle_dist:.2f}m! Stopping.")
                    self.publish_direct_cmd(0, 0, 0)
                    return
                elif self.should_slow_down():
                    speed *= 0.5

            self.publish_direct_cmd(linear_x=speed, angular_z=angular_cmd)
            pos_str = f"pos=({self.optitrack_pose[0]:.2f},{self.optitrack_pose[1]:.2f})" if self.optitrack_pose else "pos=N/A"
            rospy.loginfo_throttle(5, f"[NAV] DRIVING {pos_str} -> ({self.nav_target[0]:.2f},{self.nav_target[1]:.2f}) dist={dist:.2f}m err={np.degrees(heading_error):.1f}° spd={speed:.2f}")

    def check_arrival(self):
        """Check if robot has arrived at navigation target (uses OptiTrack)"""
        if self.nav_target is None:
            return False
        return self.distance_to_optitrack(self.nav_target) < self.ARRIVAL_TOLERANCE

    def stop_gvf_navigation(self):
        """Stop navigation and switch to direct control"""
        self.gvf_active = False
        self.nav_target = None
        self.set_nav_mode(NavigationMode.DIRECT)
        self.stop()

    # === MAIN UPDATE LOOP ===

    def update(self, _):
        # === PRIORITY CHECKS (run every cycle) ===

        # P1: Battery critical - override everything except charging
        if self.battery_percent < self.BATTERY_LOW_THRESHOLD:
            if self.state not in [State.GO_TO_DOCK, State.CHARGING]:
                rospy.logwarn(f"Battery low ({self.battery_percent*100:.1f}%)! Going to dock.")
                # Remember where we were so we can return after charging
                if self.current_sunspot is not None:
                    self.pre_interrupt_sunspot = self.current_sunspot
                if self.state == State.SUNBATHING:
                    self.end_sunbathing()
                self.stop_gvf_navigation()
                self.set_state(State.GO_TO_DOCK)
                self.reset_light_seeking()
                self.start_gvf_navigation(self.DOCK_COORDS)

        # P2: Moisture low - override light-seeking (but not battery states)
        elif self.soil_moisture < self.MOISTURE_LOW_THRESHOLD:
            if self.state in [State.IDLE, State.LIGHT_SCAN, State.LIGHT_ALIGN, State.LIGHT_MOVE, State.SUNBATHING, State.GO_TO_SUNSPOT]:
                rospy.logwarn(f"Moisture low ({self.soil_moisture}%)! Going to water.")
                # Remember where we were so we can return after dosing
                if self.current_sunspot is not None:
                    self.pre_interrupt_sunspot = self.current_sunspot
                if self.state == State.SUNBATHING:
                    self.end_sunbathing()
                self.stop_gvf_navigation()
                self.set_state(State.GO_TO_WATER)
                self.reset_light_seeking()
                self.start_gvf_navigation(self.WATER_COORDS)

        # Log current state
        rospy.loginfo_throttle(10, f"State: {self.state.value} | Nav: {self.nav_mode.value} | Battery: {self.battery_percent*100:.0f}% | Moisture: {self.soil_moisture}%")

        # === STATE MACHINE ===

        if self.state == State.IDLE:
            self.do_idle()

        elif self.state == State.GO_TO_DOCK:
            self.do_go_to_dock()

        elif self.state == State.CHARGING:
            self.do_charging()

        elif self.state == State.GO_TO_WATER:
            self.do_go_to_water()

        elif self.state == State.DOSING:
            self.do_dosing()

        elif self.state == State.REVERSING:
            self.do_reversing()

        elif self.state == State.LIGHT_SCAN:
            self.do_light_scan()

        elif self.state == State.LIGHT_ALIGN:
            self.do_light_align()

        elif self.state == State.LIGHT_MOVE:
            self.do_light_move()

        elif self.state == State.SUNBATHING:
            self.do_sunbathing()

        elif self.state == State.GO_TO_SUNSPOT:
            self.do_go_to_sunspot()

    # === STATE HANDLERS ===

    def do_idle(self):
        self.stop()
        if self.is_daytime():
            spot = self.get_best_sunspot()
            if spot is not None:
                rospy.loginfo("Daytime detected. Returning to known sunspot.")
                self.current_sunspot = spot
                self.set_state(State.GO_TO_SUNSPOT)
            else:
                rospy.loginfo("Daytime detected. Starting light-seeking.")
                self.set_state(State.LIGHT_SCAN)
                self.reset_light_seeking()

    def do_go_to_dock(self):
        """Navigate to dock using direct proportional control"""
        # Ensure nav target is dock (guards against race with other state handlers)
        if self.nav_target is None or self.nav_target != self.DOCK_COORDS:
            self.start_gvf_navigation(self.DOCK_COORDS)
        if self.distance_to_optitrack(self.DOCK_COORDS) < self.DOCK_ARRIVAL_TOLERANCE:
            rospy.loginfo("Arrived at dock. Charging...")
            self.publish_event("nav_arrival", {"station": "dock"})
            self.stop_gvf_navigation()
            self.publish_event("charge_start", {"battery_pct": self.battery_percent * 100})
            self.set_state(State.CHARGING)
        else:
            self.navigate_to_target()

    def do_charging(self):
        # Don't send any velocity commands — the driver's safety timeout
        # locks the wheels via drive_wheels(0,0,0,0) when no commands arrive.
        if self.battery_percent >= self.BATTERY_FULL:
            rospy.loginfo("Fully charged! Resuming behavior.")
            self.publish_event("charge_end", {"battery_pct": self.battery_percent * 100})
            self._return_after_interrupt()

    def do_go_to_water(self):
        """Navigate to water station using direct proportional control"""
        # Ensure nav target is water station (guards against race with other state handlers)
        if self.nav_target is None or self.nav_target != self.WATER_COORDS:
            self.start_gvf_navigation(self.WATER_COORDS)
        if self.distance_to_optitrack(self.WATER_COORDS) < self.WATER_ARRIVAL_TOLERANCE:
            rospy.loginfo("Arrived at water station. Dosing...")
            self.publish_event("nav_arrival", {"station": "water"})
            self.stop_gvf_navigation()
            self.publish_event("water_start", {"moisture_pct": self.soil_moisture})
            self.set_state(State.DOSING)
            self.dose_start_time = rospy.Time.now()
        else:
            self.navigate_to_target()

    def do_dosing(self):
        # Don't send any velocity commands — the driver's safety timeout
        # locks the wheels via drive_wheels(0,0,0,0) when no commands arrive.
        elapsed = (rospy.Time.now() - self.dose_start_time).to_sec()
        rospy.loginfo_throttle(5, f"Dosing... {elapsed:.1f}/{self.DOSE_DURATION}s")
        if elapsed >= self.DOSE_DURATION:
            rospy.loginfo("Dosing complete. Reversing away from doser...")
            self.publish_event("water_end", {"duration_s": elapsed})
            self.set_state(State.REVERSING)
            self.reverse_start_time = rospy.Time.now()

    def _return_after_interrupt(self):
        """Return to the sunspot we were at before dock/water interruption."""
        if self.pre_interrupt_sunspot is not None:
            rospy.loginfo(f"Returning to previous sunspot at {self.pre_interrupt_sunspot.get('optitrack_pos')}")
            self.current_sunspot = self.pre_interrupt_sunspot
            self.pre_interrupt_sunspot = None
            self.set_state(State.GO_TO_SUNSPOT)
            self.start_gvf_navigation(self.current_sunspot['optitrack_pos'])
        elif self.is_daytime():
            spot = self.get_best_sunspot()
            if spot is not None:
                rospy.loginfo("Returning to best known sunspot.")
                self.current_sunspot = spot
                self.set_state(State.GO_TO_SUNSPOT)
                self.start_gvf_navigation(spot['optitrack_pos'])
            else:
                rospy.loginfo("No sunspot memory. Starting fresh light scan.")
                self.set_state(State.LIGHT_SCAN)
                self.reset_light_seeking()
        else:
            self.set_state(State.IDLE)

    def do_reversing(self):
        """Reverse away from water doser before resuming navigation."""
        elapsed = (rospy.Time.now() - self.reverse_start_time).to_sec()
        if elapsed < self.REVERSE_DURATION:
            self.publish_direct_cmd(linear_x=self.REVERSE_SPEED, angular_z=0.0)
            rospy.loginfo_throttle(1, f"Reversing... {elapsed:.1f}/{self.REVERSE_DURATION}s")
        else:
            self.stop()
            rospy.loginfo("Reverse complete. Resuming behavior.")
            self._return_after_interrupt()

    def end_sunbathing(self):
        """Called when leaving SUNBATHING. Accumulates duration and may promote to window."""
        if self.sunbathing_start_time is None:
            return
        elapsed_min = (rospy.Time.now() - self.sunbathing_start_time).to_sec() / 60.0
        self.sunbathing_start_time = None
        self.sunbathing_last_check = None
        self.sunbathing_drop_counter = 0

        if self.current_sunspot is None:
            return

        self.current_sunspot['total_minutes'] = self.current_sunspot.get('total_minutes', 0) + elapsed_min
        now_hour = datetime.now().hour
        if now_hour not in self.current_sunspot.get('bright_hours', []):
            self.current_sunspot.setdefault('bright_hours', []).append(now_hour)
            self.current_sunspot['bright_hours'].sort()
        self.current_sunspot['last_confirmed'] = datetime.now().strftime('%Y-%m-%d')
        self.current_sunspot['fail_count'] = 0

        total = self.current_sunspot['total_minutes']
        was_window = self.current_sunspot.get('is_window', False)
        if total >= self.WINDOW_PROMOTION_MINUTES and not was_window:
            self.current_sunspot['is_window'] = True
            rospy.loginfo(f"[WINDOW] Promoted sunspot to WINDOW! total_minutes={total:.0f} "
                          f"hours={self.current_sunspot.get('bright_hours', [])}")
            self.publish_event("window_promoted", {
                "position": self.current_sunspot.get('optitrack_pos'),
                "total_minutes": total,
                "bright_hours": self.current_sunspot.get('bright_hours', []),
            })

        rospy.loginfo(f"[SUNBATHING] Ended after {elapsed_min:.1f}min (total={total:.0f}min, "
                      f"window={self.current_sunspot.get('is_window', False)})")
        self.save_memory()

    def do_sunbathing(self):
        """Park at a bright spot and periodically recheck brightness."""
        # Don't send any velocity commands — the driver's safety timeout
        # locks the wheels via drive_wheels(0,0,0,0) when no commands arrive.

        if not self.is_daytime():
            rospy.loginfo("[SUNBATHING] Night detected. Transitioning to IDLE.")
            self.end_sunbathing()
            self.set_state(State.IDLE)
            return

        now = rospy.Time.now()

        # Initialize on first entry
        if self.sunbathing_last_check is None:
            self.sunbathing_last_check = now
            self.sunbathing_start_time = now
            self.sunbathing_drop_counter = 0
            rospy.loginfo("[SUNBATHING] Parked at sunspot. Monitoring brightness.")
            return

        elapsed = (now - self.sunbathing_last_check).to_sec()
        if elapsed < self.SUNBATHING_RECHECK_INTERVAL:
            return

        # Time to recheck
        self.sunbathing_last_check = now
        brightness = self.get_brightness()

        if brightness < self.SUNBATHING_DROP_THRESHOLD:
            self.sunbathing_drop_counter += 1
            rospy.logwarn(f"[SUNBATHING] Brightness dropped: {brightness:.0f} < {self.SUNBATHING_DROP_THRESHOLD} "
                          f"(count: {self.sunbathing_drop_counter}/{self.SUNBATHING_DROP_COUNT})")
        else:
            self.sunbathing_drop_counter = 0

        rospy.loginfo_throttle(10, f"[SUNBATHING] brightness={brightness:.0f} drop_counter={self.sunbathing_drop_counter}")

        if self.sunbathing_drop_counter >= self.SUNBATHING_DROP_COUNT:
            rospy.logwarn("[SUNBATHING] Light dropped consistently. Rescanning.")
            self.end_sunbathing()
            self.set_state(State.LIGHT_SCAN)
            self.reset_light_seeking()

    def do_go_to_sunspot(self):
        """Navigate back to a known good sunspot using direct control."""
        if self.current_sunspot is None:
            rospy.logwarn("[GO_TO_SUNSPOT] No target sunspot. Falling back to LIGHT_SCAN.")
            self.set_state(State.LIGHT_SCAN)
            self.reset_light_seeking()
            return

        # Start navigation if not already active
        if not self.gvf_active:
            target = self.current_sunspot['optitrack_pos']
            rospy.loginfo(f"[GO_TO_SUNSPOT] Navigating to sunspot at {target}")
            self.start_gvf_navigation(target)
            return

        # Drive toward target
        self.navigate_to_target()

        # Check arrival (OptiTrack frame)
        target = self.current_sunspot['optitrack_pos']
        if self.distance_to_optitrack(target) < self.ARRIVAL_TOLERANCE:
            self.stop_gvf_navigation()
            brightness = self.get_brightness()
            rospy.loginfo(f"[GO_TO_SUNSPOT] Arrived at sunspot. Brightness: {brightness:.0f}")
            self.publish_event("nav_arrival", {"station": "sunspot", "brightness": brightness})

            if brightness >= self.SUNSPOT_ARRIVAL_BRIGHTNESS:
                rospy.loginfo("[GO_TO_SUNSPOT] Sunspot still bright! Entering SUNBATHING.")
                self.set_state(State.SUNBATHING)
                self.sunbathing_last_check = None
                self.sunbathing_drop_counter = 0
            else:
                rospy.logwarn(f"[GO_TO_SUNSPOT] Sunspot too dark ({brightness:.0f} < {self.SUNSPOT_ARRIVAL_BRIGHTNESS}). Marking failed.")
                self.publish_event("sunspot_failed", {
                    "brightness": brightness,
                    "threshold": self.SUNSPOT_ARRIVAL_BRIGHTNESS,
                    "position": list(target),
                })
                self.current_sunspot['fail_count'] = self.current_sunspot.get('fail_count', 0) + 1
                self.current_sunspot['last_failed'] = datetime.now().strftime('%Y-%m-%d')
                # Demote window if too many failures
                if (self.current_sunspot.get('is_window', False) and
                        self.current_sunspot['fail_count'] >= self.WINDOW_FAIL_DEMOTE_COUNT):
                    self.current_sunspot['is_window'] = False
                    rospy.logwarn(f"[WINDOW] Demoted window after {self.current_sunspot['fail_count']} failures.")
                    self.publish_event("window_demoted", {
                        "position": self.current_sunspot.get('optitrack_pos'),
                        "fail_count": self.current_sunspot['fail_count'],
                    })
                self.save_memory()

                # Try next best sunspot
                next_spot = self.get_best_sunspot()
                if next_spot is not None:
                    rospy.loginfo("[GO_TO_SUNSPOT] Trying next best sunspot.")
                    self.current_sunspot = next_spot
                    self.start_gvf_navigation(next_spot['optitrack_pos'])
                else:
                    rospy.loginfo("[GO_TO_SUNSPOT] No valid sunspots left. Falling back to LIGHT_SCAN.")
                    self.set_state(State.LIGHT_SCAN)
                    self.reset_light_seeking()

    def reset_light_seeking(self):
        self.scan_start_yaw = None
        self.scan_last_yaw = None
        self.scan_accumulated_rotation = 0.0
        self.brightness_log = []
        self.move_start_pos = None
        self.bright_counter = 0

    def do_light_scan(self):
        """360° scan to find brightest direction - uses DIRECT control"""
        if self.nav_mode != NavigationMode.DIRECT:
            self.set_nav_mode(NavigationMode.DIRECT)

        if self.image is None:
            rospy.logwarn_throttle(5, "Waiting for camera image...")
            return

        if self.scan_start_yaw is None:
            rospy.loginfo(f"[SCAN START] 360° light scan, yaw={np.degrees(self.odom_yaw):.1f}°")
            self.scan_start_yaw = self.odom_yaw
            self.scan_last_yaw = self.odom_yaw
            self.scan_accumulated_rotation = 0.0
            self.brightness_log = []
            self.scan_start_time = rospy.Time.now()
            self._debug_yaw_samples = []
            self._debug_scan_cmd_gaps = []
            self._debug_cmd_count = 0

        brightness = self.get_brightness()

        # Log brightness at each angle (use odom_yaw for light seeking)
        if abs(self.angle_diff(self.odom_yaw, self.scan_last_yaw)) > 0.01:
            self.brightness_log.append((self.odom_yaw, brightness))

        # Track rotation - count absolute rotation
        delta = self.angle_diff(self.odom_yaw, self.scan_last_yaw)
        self.scan_accumulated_rotation += abs(delta)

        # === DEBUG: Sample yaw for analysis ===
        self._debug_yaw_samples.append((time.time(), self.odom_yaw, delta))

        # === DEBUG: Detect if yaw is stuck ===
        if len(self._debug_yaw_samples) > 10:
            recent_deltas = [abs(s[2]) for s in self._debug_yaw_samples[-10:]]
            if all(d < 0.001 for d in recent_deltas):
                rospy.logwarn_throttle(5, f"[SCAN STUCK?] Yaw hasn't changed in 10 samples! yaw={np.degrees(self.odom_yaw):.1f}° (odom)")

        self.scan_last_yaw = self.odom_yaw

        # Require at least 6 seconds of scanning AND 330 degrees of rotation
        scan_duration = (rospy.Time.now() - self.scan_start_time).to_sec()
        target_rotation_deg = 330
        target_rotation_rad = 5.76  # 330 degrees in radians

        progress_pct = min(100, (self.scan_accumulated_rotation / target_rotation_rad) * 100)
        status_msg = f"{np.degrees(self.scan_accumulated_rotation):.0f}°/{target_rotation_deg}° ({progress_pct:.0f}%)"
        rospy.loginfo_throttle(5, f"[SCAN] {status_msg}")
        self.debug_scan_pub.publish(String(data=status_msg))

        if self.scan_accumulated_rotation < target_rotation_rad or scan_duration < 6.0:
            # Keep rotating
            self.publish_direct_cmd(angular_z=0.25)
        else:
            # Scan complete
            self.stop()
            rospy.loginfo(f"[SCAN COMPLETE] rotation={np.degrees(self.scan_accumulated_rotation):.0f}° duration={scan_duration:.1f}s")

            bright_angles = [b for _, b in self.brightness_log if b > self.BRIGHTNESS_SCAN_THRESHOLD]
            rospy.loginfo(f"Scan complete. Bright angles: {len(bright_angles)}/{len(self.brightness_log)}")

            self.publish_event("scan_complete", {
                "num_bright_angles": len(bright_angles),
                "total_angles": len(self.brightness_log),
                "brightest": max((b for _, b in self.brightness_log), default=0),
                "duration_s": scan_duration,
            })

            if self.brightness_log:
                best_yaw, best_brightness = max(self.brightness_log, key=lambda x: x[1])
                rospy.loginfo(f"Brightest direction: {np.degrees(best_yaw):.1f}° (brightness={best_brightness:.0f})")
                self.target_yaw = best_yaw
                self.set_state(State.LIGHT_ALIGN)
            else:
                rospy.logwarn("No brightness data. Rescanning.")
                self.reset_light_seeking()

    def do_light_align(self):
        """Align to brightest direction - uses DIRECT control"""
        if self.nav_mode != NavigationMode.DIRECT:
            self.set_nav_mode(NavigationMode.DIRECT)

        error = self.angle_diff(self.target_yaw, self.odom_yaw)
        rospy.loginfo_throttle(5, f"ALIGN: target={np.degrees(self.target_yaw):.1f}° current={np.degrees(self.odom_yaw):.1f}° error={np.degrees(error):.1f}° (odom)")

        if abs(error) > 0.1:  # ~6 degrees threshold
            # Turn in the direction that reduces the error
            # Positive error means target is counterclockwise from current -> turn counterclockwise (positive angular_z)
            # Negative error means target is clockwise from current -> turn clockwise (negative angular_z)
            turn_speed = 0.25 if error > 0 else -0.25
            self.publish_direct_cmd(angular_z=turn_speed)
        else:
            self.stop()
            rospy.loginfo("Aligned. Moving toward light.")
            # Use odometry position for light seeking distance tracking
            self.move_start_pos = self.odom_pos
            self.bright_counter = 0
            self.set_state(State.LIGHT_MOVE)

    def do_light_move(self):
        """Move toward light source - uses DIRECT control"""
        if self.nav_mode != NavigationMode.DIRECT:
            self.set_nav_mode(NavigationMode.DIRECT)

        if self.move_start_pos is None:
            self.set_state(State.LIGHT_SCAN)
            self.reset_light_seeking()
            return

        # Use odometry position for distance tracking
        dist = np.hypot(self.odom_pos[0] - self.move_start_pos[0],
                        self.odom_pos[1] - self.move_start_pos[1])

        brightness = self.get_brightness()

        # Check if sustained brightness (only after moving a minimum distance)
        if brightness > self.BRIGHTNESS_MOVE_THRESHOLD and dist >= self.MIN_MOVE_BEFORE_PARK:
            self.bright_counter += 1
            if self.bright_counter >= self.BRIGHT_CONFIRM_COUNT:
                rospy.loginfo(f"Found bright area at dist={dist:.2f}m. Saving sunspot and entering SUNBATHING.")
                self.stop()
                self.save_sunspot()
                self.set_state(State.SUNBATHING)
                self.sunbathing_last_check = None
                self.sunbathing_drop_counter = 0
                self.reset_light_seeking()
                return
        else:
            self.bright_counter = 0

        if dist < 2.0:
            # === OBSTACLE AVOIDANCE ===
            if self.is_obstacle_ahead():
                # Obstacle too close - stop and rescan
                rospy.logwarn(f"[OBSTACLE] Obstacle detected at {self.min_obstacle_dist:.2f}m! Stopping and rescanning.")
                self.publish_event("obstacle_detected", {"distance_m": self.min_obstacle_dist})
                self.stop()
                self.set_state(State.LIGHT_SCAN)
                self.reset_light_seeking()
                return

            # Determine speed based on obstacle distance
            if self.should_slow_down():
                # Slow down when approaching obstacle
                speed = self.LIGHT_MOVE_SPEED * 0.5
                rospy.loginfo_throttle(5, f"[MOVE] dist={dist:.2f}m obstacle={self.min_obstacle_dist:.2f}m SLOWING to {speed} m/s")
            else:
                speed = self.LIGHT_MOVE_SPEED
                rospy.loginfo_throttle(5, f"[MOVE] dist={dist:.2f}m brightness={brightness:.0f} spd={speed} m/s")

            self.publish_direct_cmd(linear_x=speed)
        else:
            rospy.loginfo("Moved 2m. Rescanning.")
            self.stop()
            self.set_state(State.LIGHT_SCAN)
            self.reset_light_seeking()


if __name__ == "__main__":
    BOTanicaBrain()
