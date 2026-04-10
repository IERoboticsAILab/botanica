#!/usr/bin/env python3
import rospy
from sensor_publisher.msg import SensorData
from gattlib import GATTRequester, GATTResponse
from struct import unpack
import time

class SensorDataResponse(GATTResponse):
    def __init__(self, publisher):
        self.publisher = publisher

    def on_response(self, data):
        try:
            # Unpack sensor data
            temperature, sunlight, moisture, fertility = unpack('<hxIBHxxxxxx', data)

            # Create and publish ROS message
            msg = SensorData()
            msg.temperature = temperature / 10.0  # Convert to °C
            msg.sunlight = sunlight
            msg.moisture = moisture
            msg.fertility = fertility
            self.publisher.publish(msg)

        except Exception as e:
            rospy.logerr("Error unpacking sensor data: {}".format(str(e)))

def main():
    rospy.init_node('sensor_publisher')
    pub = rospy.Publisher('sensor_data', SensorData, queue_size=10)

    address = "5C:85:7E:12:D0:2E"  # BLE device address
    requester = GATTRequester(address, False)
    response = SensorDataResponse(pub)

    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Connecting to device...")
            requester.connect(True)
            rospy.loginfo("Connected to device")

            # Read battery and firmware (optional)
            try:
                data = requester.read_by_handle(0x0038)[0]
                battery, version = unpack('<B6s', data)
                rospy.loginfo("Battery: {}%, Firmware: {}".format(battery, version.decode('utf-8').strip()))
            except Exception as e:
                rospy.logwarn("Error reading battery/firmware: {}".format(e))

            # Enable real-time data
            requester.write_by_handle(0x0033, bytes([0xa0, 0x1f]))
            requester.write_by_handle(0x0036, bytes([0x01, 0x00]))

            # Main loop
            while not rospy.is_shutdown():
                requester.read_by_handle_async(0x0035, response)
                time.sleep(5)  # Throttle publish rate

        except Exception as e:
            rospy.logerr("Connection error: {}. Retrying...".format(str(e)))
            time.sleep(5)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
