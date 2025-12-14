import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


PORT = "/dev/ttyACM0"
BAUD = 9600


def parse_sensor_line(line: str):
    """
    Parse a raw serial line received from the Arduino.

    Expected format:
        "TEMP:24.40,LIGHT:665,KEY:3"

    Output format for ROS topic:
        "temperature=24.40, light=665, key=3"

    Returns None if the format is invalid.
    """
    line = line.strip()
    if not line.startswith("TEMP:"):
        return None

    try:
        parts = line.split(",")
        # Example: ["TEMP:24.40", "LIGHT:665", "KEY:3"]
        temperature = float(parts[0].split(":")[1])
        light_value = int(parts[1].split(":")[1])
        key_value = parts[2].split(":")[1].strip()

        return f"temperature={temperature:.2f}, light={light_value}, key={key_value}"

    except (IndexError, ValueError):
        return None


class SensorBridgeNode(Node):
    """
    ROS2 node that:
    - Opens the serial port to communicate with Arduino
    - Reads temperature, light and keypad data
    - Publishes the parsed output to the /sensor_values topic

    This node replaces LED control logic from earlier versions
    and focuses solely on converting Arduino serial lines into
    ROS2 message strings.
    """

    def __init__(self):
        super().__init__("sensor_bridge_node")

        self.publisher_ = self.create_publisher(String, "sensor_values", 10)

        self.get_logger().info(f"Opening serial port: {PORT} @ {BAUD} baud")

        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {PORT}: {e}")
            raise

        # Allow Arduino to reset after connecting
        time.sleep(2.0)
        self.get_logger().info("Serial port ready. Starting timer.")

        # Read serial data at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """
        Read a line from Arduino, parse it, and publish to /sensor_values.
        """
        try:
            raw_line = self.ser.readline().decode("utf-8", errors="ignore").strip()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
            return

        if not raw_line:
            return

        parsed = parse_sensor_line(raw_line)
        if parsed is None:
            self.get_logger().warn(f"Invalid sensor line: '{raw_line}'")
            return

        msg = String()
        msg.data = parsed
        self.publisher_.publish(msg)

    def destroy_node(self):
        """
        Close serial port cleanly before shutting down.
        """
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

