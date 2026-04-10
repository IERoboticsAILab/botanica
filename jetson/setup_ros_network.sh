#!/bin/bash
# =============================================================================
# BOTanica - Jetson ROS Network Setup
# =============================================================================
# Run this script before launching ROS nodes on the Jetson.
# It configures the Jetson to connect to the ROS Master on the server.
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
    echo "  BOTanica Jetson Network Setup"
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

# Get this Jetson's IP address
JETSON_IP=$(hostname -I | awk '{print $1}')

if [ -z "$JETSON_IP" ]; then
    echo "ERROR: Could not determine Jetson's IP address."
    echo "Make sure you're connected to the network."
    return 1 2>/dev/null || exit 1
fi

# Set ROS environment variables
export ROS_MASTER_URI=http://${SERVER_IP}:11311
export ROS_IP=${JETSON_IP}

echo "=========================================="
echo "  BOTanica Jetson Network Configured"
echo "=========================================="
echo ""
echo "  ROS_MASTER_URI = $ROS_MASTER_URI"
echo "  ROS_IP         = $ROS_IP"
echo ""
echo "  Server: $SERVER_IP"
echo "  This Jetson: $JETSON_IP"
echo ""
echo "Now run:"
echo "  roslaunch light_follower botanica_brain.launch"
echo "=========================================="
