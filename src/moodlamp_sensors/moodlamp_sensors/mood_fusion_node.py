import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_sensor_string(data: str):
    """
    Parse data received from /sensor_values topic.

    Expected input format:
        "temperature=24.40, light=665, key=3"

    Returns:
        (temperature: float, light: int, key: str)
    or None if parsing fails.
    """
    try:
        parts = data.split(",")
        temperature = float(parts[0].split("=")[1].strip())
        light = int(parts[1].split("=")[1].strip())
        key = parts[2].split("=")[1].strip()
        return temperature, light, key
    except (IndexError, ValueError):
        return None


class MoodFusionNode(Node):
    """
    ROS2 node that computes the final mood of the environment.

    Inputs:
      - /sensor_values: temperature, light level, keypad key
      - /emotion_label: (optional) facial emotion such as
                        "happy", "sad", "neutral"

    Logic:
      1. If the keypad key is 1,2,3 → manual override mode
         - 1 → relaxed
         - 2 → energetic
         - 3 → focus

      2. Otherwise (key = A or NONE) → Auto mode:
         - If temperature < 18°C → too_cold (winter music)
         - If temperature > 28°C → too_hot (summer music)
         - Else (comfortable temperature):
             Combine emotion + light level:
               happy  + bright/moderate → energetic
               happy  + dark            → cozy
               sad    + dark/moderate  → relaxed
               neutral + moderate       → cozy
               neutral + bright         → energetic
               neutral + dark           → relaxed
    """

    def __init__(self):
        super().__init__("mood_fusion_node")

        # Subscribe to sensor values
        self.sensor_sub = self.create_subscription(
            String, "sensor_values", self.sensor_callback, 10
        )

        # Subscribe to optional emotion label
        self.emotion_sub = self.create_subscription(
            String, "emotion_label", self.emotion_callback, 10
        )

        # Publisher for final mood label
        self.publisher_ = self.create_publisher(String, "mood_state", 10)

        # Latest state variables
        self.current_temperature = None
        self.current_light = None
        self.current_key = "NONE"
        self.current_emotion = "neutral"  # default

        self.get_logger().info("MoodFusionNode initialized.")

    def sensor_callback(self, msg: String):
        """
        Handles incoming sensor data.
        """
        parsed = parse_sensor_string(msg.data)
        if parsed is None:
            self.get_logger().warn(f"Failed to parse sensor string: '{msg.data}'")
            return

        temperature, light, key = parsed
        self.current_temperature = temperature
        self.current_light = light
        self.current_key = key

        self.update_mood()

    def emotion_callback(self, msg: String):
        """
        Update the current emotion whenever an emotion label is received.
        """
        self.current_emotion = msg.data.strip().lower()
        self.update_mood()

    def update_mood(self):
        """
        Compute and publish the final mood state using all available information.
        """
        if self.current_temperature is None or self.current_light is None:
            return  # Not enough data yet

        mood_label = self.compute_mood(
            temperature=self.current_temperature,
            light=self.current_light,
            key=self.current_key,
            emotion=self.current_emotion,
        )

        msg = String()
        msg.data = mood_label
        self.publisher_.publish(msg)

        self.get_logger().info(
            f"temp={self.current_temperature:.2f} °C, "
            f"light={self.current_light}, key={self.current_key}, "
            f"emotion={self.current_emotion} → mood='{mood_label}'"
        )

    @staticmethod
    def compute_mood(temperature: float, light: int, key: str, emotion: str) -> str:
        """
        Determine the final mood state based on:
          - Manual override (keypad)
          - Environmental conditions (temperature + light)
          - Facial emotion
        """

        # ----- 1) Manual override mode -----
        if key == "1":
            return "relaxed"
        elif key == "2":
            return "energetic"
        elif key == "3":
            return "focus"

        # ----- 2) Automatic mode -----
        # Temperature classification
        if temperature < 18.0:
            return "too_cold"  # winter / carol music
        elif temperature > 28.0:
            return "too_hot"   # summer music

        # Comfortable temperature range
        # Light classification
        if light < 300:
            light_state = "dark"
        elif light > 800:
            light_state = "bright"
        else:
            light_state = "moderate"

        e = (emotion or "neutral").lower()

        # Emotion + light rules
        if e == "happy":
            if light_state in ("bright", "moderate"):
                return "energetic"
            else:
                return "cozy"

        elif e == "sad":
            if light_state in ("dark", "moderate"):
                return "relaxed"
            else:
                return "cozy"

        else:  # neutral emotion
            if light_state == "moderate":
                return "cozy"
            elif light_state == "bright":
                return "energetic"
            else:
                return "relaxed"


def main(args=None):
    rclpy.init(args=args)

    node = MoodFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

