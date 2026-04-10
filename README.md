# BOTanica

*An autonomous plant-inspired robot that seeks light, monitors its own environmental sensors, and navigates back to a charging dock when its battery runs low.*

---

## Two-Device Architecture

**BOTanica is a distributed system that runs on two physically separate computers** mounted on the robot. They are connected over the local network and communicate exclusively through ROS topics.

| Device | Role | ROS Distro | Code Lives In |
|---|---|---|---|
| **Raspberry Pi** | Low-level robot driver. Runs `roscore`. Talks to the DJI RoboMaster chassis over USB and exposes it as standard ROS topics (`/cmd_vel`, `/odom`, IMU). | Noetic | `raspi/` |
| **NVIDIA Jetson** | High-level "brain". Runs the state machine, sensor fusion, light-seeking behavior, GVF dock navigation, and battery/moisture monitoring. Subscribes to the Pi's topics and publishes velocity commands back. | Melodic | `jetson/` |

The two devices are **not interchangeable** — each runs its own catkin workspace with its own packages, and the code in `raspi/` will not run on the Jetson (and vice versa). The Jetson is configured to use the Pi as its ROS master via `ROS_MASTER_URI`.

```
   ┌──────────────────┐  ROS topics over LAN   ┌─────────────────────┐
   │   Raspberry Pi   │ ─────────────────────► │       Jetson        │
   │  (ROS Noetic)    │   /odom, /imu, ...     │   (ROS Melodic)     │
   │                  │ ◄───────────────────── │                     │
   │  roscore         │       /cmd_vel         │  botanica_brain     │
   │  robomaster_     │                        │  sensor_publisher   │
   │  driver_node     │                        │  cmd_vel_mux        │
   └──────────────────┘                        └─────────────────────┘
          │                                               │
          ▼                                               ▼
   DJI RoboMaster                              RealSense camera,
   chassis (USB)                               BLE sensors, OptiTrack
```

---

## Repository Layout

```
BOTanica/
├── jetson/                              # Runs on the Jetson (ROS Melodic)
│   ├── setup_ros_network.sh
│   └── src/
│       ├── light_follower/              # High-level brain package
│       │   ├── config/gvf_params.yaml   # GVF dock-navigation parameters
│       │   ├── launch/
│       │   │   ├── botanica_brain.launch
│       │   │   └── experiment.launch
│       │   ├── scripts/
│       │   │   ├── botanica_brain.py    # Main state machine
│       │   │   ├── cmd_vel_mux.py       # Multiplexes /cmd_vel sources
│       │   │   ├── experiment_logger.py
│       │   │   ├── light.py
│       │   │   ├── LP.py
│       │   │   └── pathPlanningLight.py
│       │   └── tests/test_botanica_brain.py
│       ├── sensor_publisher/            # BLE sensor → ROS bridge
│       │   ├── msg/SensorData.msg
│       │   ├── launch/sensor_system.launch
│       │   └── scripts/sensorInfo.py
│       └── librealsense/                # Vendored RealSense SDK
│
├── raspi/                               # Runs on the Raspberry Pi (ROS Noetic)
│   ├── setup_ros_network.sh
│   └── src/robomaster_driver/
│       ├── launch/
│       │   ├── robomaster_driver.launch
│       │   └── teleop.launch
│       ├── rviz/robomaster.rviz
│       └── src/
│           ├── robomaster_driver_node.py   # Live driver node
│           └── driver_OG.py                # Legacy reference
│
├── code/                                # Standalone scripts and prototypes
│   ├── Arduino code                     # Arduino sketch (charging dock?)
│   └── static_gvf_field.py
│
├── light-to-movement.py                 # Standalone light-tracking prototype
└── README.md
```

---

## Quick Start

### On the Raspberry Pi
```bash
# Start the ROS master
roscore

# In a second shell, launch the RoboMaster driver
cd ~/BOTanica/raspi
source devel/setup.bash
roslaunch robomaster_driver robomaster_driver.launch
```

### On the Jetson
```bash
# Point ROS at the Pi
export ROS_MASTER_URI=http://<PI_IP>:11311
export ROS_IP=<JETSON_IP>

cd ~/BOTanica/jetson
source devel/setup.bash

# Start the BLE sensor publisher
roslaunch sensor_publisher sensor_system.launch

# Start the brain
roslaunch light_follower botanica_brain.launch
```

The helper scripts `jetson/setup_ros_network.sh` and `raspi/setup_ros_network.sh` set the appropriate `ROS_MASTER_URI` / `ROS_IP` environment variables for each device.

---

## Behaviors

The brain (`botanica_brain.py`) is a priority-based state machine:

1. **Battery low** → navigate to dock via GVF (OptiTrack frame), then charge.
2. **Soil moisture low** → navigate to the watering station.
3. **Otherwise** → light-seeking: scan, identify the brightest direction, move toward it (odometry frame).

State transitions, scans, and dock arrivals are published as events through `experiment_logger.py` for offline analysis.

---

## Useful Topics

```bash
rostopic echo /sensor_data                # BLE sensor readings
rostopic echo /battery_level              # Battery state
rostopic echo /cmd_vel                    # Final velocity sent to chassis
rostopic echo /camera/color/image_raw     # RealSense color stream
```

---

## License
Apache 2.0. See [LICENSE](LICENSE).
