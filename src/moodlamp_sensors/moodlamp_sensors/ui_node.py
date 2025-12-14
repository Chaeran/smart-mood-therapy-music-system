import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_kv_string(s: str) -> dict:
    """
    Parse a simple key-value string like:
      "temperature=24.40, light=665"
    or (optional key):
      "temperature=24.40, light=665, key=3"

    Returns a dict, e.g. {"temperature": "24.40", "light": "665", "key": "3"}
    """
    result = {}
    try:
        parts = [p.strip() for p in s.split(",")]
        for p in parts:
            if "=" in p:
                k, v = p.split("=", 1)
                result[k.strip()] = v.strip()
    except Exception:
        pass
    return result


class UINode(Node):
    """
    Terminal UI node that displays current system status.

    Subscribes:
      - sensor_values (std_msgs/String)
      - mood_state    (std_msgs/String)
      - current_music (std_msgs/String)  [published by music_player_node]

    Updates the terminal display periodically (default: 1 Hz).
    """

    def __init__(self):
        super().__init__("ui_node")

        # Internal state
        self.temperature = None
        self.light = None
        self.key = None
        self.mood = None
        self.current_music = None

        # Subscriptions
        self.create_subscription(String, "sensor_values", self.sensor_cb, 10)
        self.create_subscription(String, "mood_state", self.mood_cb, 10)
        self.create_subscription(String, "current_music", self.music_cb, 10)

        # UI refresh rate (Hz)
        self.declare_parameter("refresh_hz", 1.0)
        refresh_hz = self.get_parameter("refresh_hz").get_parameter_value().double_value
        refresh_period = 1.0 / max(refresh_hz, 0.1)

        self.timer = self.create_timer(refresh_period, self.render)

        self.get_logger().info("UINode started. Listening to /sensor_values, /mood_state, /current_music ...")

    def sensor_cb(self, msg: String):
        """
        Expected message examples:
          "temperature=24.40, light=665"
          "temperature=24.40, light=665, key=3" (optional)
        """
        data = parse_kv_string(msg.data)
        if "temperature" in data:
            self.temperature = data["temperature"]
        if "light" in data:
            self.light = data["light"]
        if "key" in data:
            self.key = data["key"]

    def mood_cb(self, msg: String):
        self.mood = msg.data.strip()

    def music_cb(self, msg: String):
        self.current_music = msg.data.strip()

    def render(self):
        # Clear terminal
        os.system("clear")

        print("==============================================")
        print("      Smart Mood Therapy Music System (UI)    ")
        print("==============================================")

        print(f"Temperature:    {self.temperature if self.temperature is not None else '---'}  °C")
        print(f"Light Level:    {self.light if self.light is not None else '---'}")
        print(f"Keypad Input:   {self.key if self.key is not None else '---'}")
        print("----------------------------------------------")
        print(f"Final Mood:     {self.mood if self.mood is not None else '---'}")
        print(f"Current Music:  {self.current_music if self.current_music is not None else '---'}")
        print("==============================================")
        print("Tip: Press keypad 1/2/3 to override (if enabled)")
        print("==============================================")


def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
