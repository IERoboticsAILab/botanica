from gattlib import GATTRequester, GATTResponse
from struct import unpack
import time
import serial
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)

# Define the MAC address of the BLE sensor
address = "5C:85:7E:12:D0:2E"

# Function to find the correct serial port
def find_serial_port():
    import glob
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    if not ports:
        raise IOError("No serial ports found")
    return ports[0]

# Initialize serial communication with Arduino
try:
    serial_port = find_serial_port()
    arduino = serial.Serial(serial_port, 9600, timeout=1)  # Adjust the port name as needed
    logging.info(f"Connected to Arduino on port {serial_port}")
except Exception as e:
    logging.error(f"Error initializing serial connection: {e}")
    exit(1)

class SensorDataResponse(GATTResponse):
    def on_response(self, data):
        try:
            # Unpack the data received from handle 0x0035
            temperature, sunlight, moisture, fertility = unpack('<hxIBHxxxxxx', data)
            logging.info(f"Light intensity: {sunlight} lux")
            logging.info(f"Temperature: {temperature / 10.} °C")
            logging.info(f"Soil moisture: {moisture} %")
            logging.info(f"Soil fertility: {fertility} uS/cm")

            # Send signal to Arduino if light intensity is below 140
            if sunlight < 140:
                arduino.write(b'F')  # Send 'F' to Arduino to move forward
                logging.info("Light intensity below 140 lux, sending signal to Arduino...")
                time.sleep(5)  # Wait for 5 seconds
                arduino.write(b'S')  # Send 'S' to Arduino to stop
                logging.info("Sent stop signal to Arduino after moving forward for 5 seconds.")
        except Exception as e:
            logging.error(f"Error unpacking sensor data: {str(e)}")

def main():
    try:
        requester = GATTRequester(address, False)

        while True:
            try:
                # Wait for connection
                logging.info("Connecting to device...")
                requester.connect(True)
                logging.info("Connected to device")

                # Read battery level and firmware version
                data = requester.read_by_handle(0x0038)[0]
                battery, version = unpack('<B6s', data)
                logging.info(f"Battery level: {battery} %")
                logging.info(f"Firmware version: {version.decode('utf-8').strip()}")

                # Enable real-time data reading
                requester.write_by_handle(0x0033, bytes([0xa0, 0x1f]))
                logging.info("Enabled real-time data reading")

                # Subscribe to sensor value notifications
                requester.write_by_handle(0x0036, bytes([0x01, 0x00]))
                logging.info("Subscribed to sensor value notifications")

                # Create a response handler
                response = SensorDataResponse()

                # Continuously check sensor data
                while True:
                    try:
                        # Read sensor data asynchronously
                        requester.read_by_handle_async(0x0035, response)
                        time.sleep(5)  # Wait for 5 seconds before reading again
                    except Exception as e:
                        logging.error(f"Error reading sensor data: {str(e)}")
                        break

            except Exception as e:
                logging.error(f"An error occurred: {str(e)}")
                logging.info("Attempting to reconnect...")
                time.sleep(5)  # Wait before trying to reconnect

    except Exception as e:
        logging.error(f"Fatal error: {str(e)}")
        exit(1)

if __name__ == "__main__":
    main()
