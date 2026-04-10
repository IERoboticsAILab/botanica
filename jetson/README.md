# Jetson ROS Setup Guide

This guide covers:
1. Running the BLE sensor publisher (`sensorInfo.py`).
2. Running the combined BLE sensor + RealSense camera system.
3. Executing the light path planner (`LP.py`).

---

## **Prerequisites**
Before starting:
1. **ROS Melodic** is installed and sourced:
   ```bash
   source /opt/ros/melodic/.bashrc
   ```
2. **BLE Sensor Dependencies**:
   ```bash
   sudo apt-get install python-gattlib
   sudo usermod -aG bluetooth $USER  # Add user to Bluetooth group
   ```
3. **RealSense Camera (Optional)**:
   ```bash
   sudo apt-get install ros-melodic-realsense2-camera
   ```
4. **Script Permissions**:
   ```bash
   chmod +x sensor_publisher/scripts/sensorInfo.py
   chmod +x light_follower/scripts/LP.py  
   ```

---

## **1. Running the BLE Sensor (Standalone)**
### **Method 1: Direct Execution**
```bash
rosrun sensor_publisher sensorInfo.py
```
- **Topics**:
  - `/sensor_data`: Publishes temperature, light, moisture, fertility

### **Method 2: Launch File**
Create `sensor_only.launch` in `sensor_publisher/launch/`:
```xml
<launch>
    <node name="sensor_publisher" pkg="sensor_publisher" type="sensorInfo.py" output="screen"/>
</launch>
```
Run:
```bash
roslaunch sensor_publisher sensor_only.launch
```

---

## **2. Running Sensor + Camera Together**
### **Combined Launch File**
```bash
roslaunch sensor_publisher sensor_system.launch
```
- **Topics**:
  - `/sensor_data`: BLE sensor data.
  - `/camera/color/image_raw`: Camera feed (if enabled).

---

## **3. Running Light Path Planning (`LP.py`)**
### **Prerequisites**
- Ensure the BLE sensor (and camera, if needed) are active.
- Install `LP.py` dependencies (e.g., `numpy`, `opencv`).

### **Execution**
```bash
rosrun light_follower LP.py  
# OR
python2 LP.py                  
```

---

## **Troubleshooting**
- **BLE Connection Issues**:
  - Confirm the sensor’s MAC address in `sensorInfo.py`.
  - Restart Bluetooth: `sudo systemctl restart bluetooth`.
- **Camera Errors**:
  - Reinstall RealSense drivers: `sudo apt reinstall ros-melodic-realsense2-camera`.
- **ROS Master**:
  - Ensure `roscore` is running or use `roslaunch`.
