#!/bin/bash
# =============================================================================
# BOTanica - Raspberry Pi ROS Network Setup
# =============================================================================
# Run this script before launching ROS nodes on the Pi.
# It configures the Pi to connect to the ROS Master on the server.
#
# Usage:
#   source setup_ros_network.sh <SERVER_IP>
#
# Example:
#   source setup_ros_network.sh 192.168.1.100
#
# =============================================================================

# Check if server IP was provided
if [ -z "$1" ]; then
    echo "=========================================="
    echo "  BOTanica Pi Network Setup"
    echo "=========================================="
    echo ""
    echo "Usage: source setup_ros_network.sh <SERVER_IP>"
    echo ""
    echo "Example:"
    echo "  source setup_ros_network.sh 192.168.1.100"
    echo ""
    echo "To find server IP, run 'hostname -I' on the server."
    echo "=========================================="
    return 1 2>/dev/null || exit 1
fi

SERVER_IP=$1

# Get this Pi's IP address (try common interfaces)
PI_IP=$(hostname -I | awk '{print $1}')

if [ -z "$PI_IP" ]; then
    echo "ERROR: Could not determine Pi's IP address."
    echo "Make sure you're connected to the network."
    return 1 2>/dev/null || exit 1
fi

# Set ROS environment variables
export ROS_MASTER_URI=http://${SERVER_IP}:11311
export ROS_IP=${PI_IP}

echo "=========================================="
echo "  BOTanica Pi Network Configured"
echo "=========================================="
echo ""
echo "  ROS_MASTER_URI = $ROS_MASTER_URI"
echo "  ROS_IP         = $ROS_IP"
echo ""
echo "  Server: $SERVER_IP"
echo "  This Pi: $PI_IP"
echo ""
echo "Now run:"
echo "  roslaunch robomaster_driver robomaster_driver.launch"
echo "=========================================="
