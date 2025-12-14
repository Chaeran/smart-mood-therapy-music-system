import os
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MusicPlayerNode(Node):
    """
    ROS2 node that plays local audio files based on the current mood.

    Input:
      - /mood_state (std_msgs/String): e.g.
          "relaxed", "energetic", "cozy",
          "focus", "too_hot", "too_cold"

    Behavior:
      - Whenever the mood changes, the node stops the previous track
        and starts a new one that matches the current mood.
      - Audio files are stored in a local directory (music_dir).
      - An external command-line player (ffplay, mpg123, etc.) is used
        to play the audio files.
    """

    def __init__(self):
        super().__init__("music_player_node")

        # Subscribe to mood_state from MoodFusionNode
        self.mood_sub = self.create_subscription(
            String,
            "mood_state",
            self.mood_callback,
            10,
        )
        # Subscribe to current music from music_player_node
        self.current_music_pub = self.create_publisher(String, "current_music", 10)

        # Parameters:
        #  - music_dir: directory containing audio files
        #  - player_cmd: command-line audio player (e.g. "ffplay" or "mpg123")
        self.declare_parameter(
            "music_dir",
            "/home/lana/mood_music"  # TODO: change to your actual path
        )
        self.declare_parameter(
            "player_cmd",
            "ffplay"  # or "mpg123"
        )

        self.music_dir = self.get_parameter("music_dir").get_parameter_value().string_value
        self.player_cmd = self.get_parameter("player_cmd").get_parameter_value().string_value

        # Map each mood to a specific audio file name (relative to music_dir)
        # Make sure these file names exist in your music_dir.
        self.mood_to_file = {
            "relaxed": "relaxed.mp3",
            "cozy": "cozy.mp3",
            "focus": "focus.mp3",  # or "focus_satie_gymnopedie1.mp3"
            "energetic": "energetic.mp3",
            "too_hot": "too_hot.mp3",   # tropical / summer vibe
            "too_cold": "too_cold.mp3",  # winter / christmas vibe
        }

        self.current_mood: Optional[str] = None
        self.current_process: Optional[subprocess.Popen] = None

        self.get_logger().info(
            f"MusicPlayerNode started.\n"
            f"  music_dir = '{self.music_dir}'\n"
            f"  player_cmd = '{self.player_cmd}'"
        )

    # ------------------------------------------------------------------ #
    #  Helper methods                                                    #
    # ------------------------------------------------------------------ #

    def stop_music(self):
        """
        Stop the currently playing process, if any.
        """
        if self.current_process is not None:
            if self.current_process.poll() is None:
                # Process is still running → terminate it
                try:
                    self.current_process.terminate()
                except Exception as e:
                    self.get_logger().warn(f"Failed to terminate player process: {e}")
            self.current_process = None

    def play_music_for_mood(self, mood: str):
        """
        Look up the audio file for the given mood and start a new player process.
        """
        filename = self.mood_to_file.get(mood)
        if filename is None:
            # No music defined for this mood
            self.get_logger().warn(f"No audio file mapped for mood '{mood}'. Stopping music.")
            self.stop_music()
            self.current_music_pub.publish(String(data="(stopped)"))
            return
            
        filepath = os.path.join(self.music_dir, filename)
        if not os.path.isfile(filepath):
            self.get_logger().warn(
                f"Audio file '{filepath}' does not exist. "
                f"Check your music_dir and filenames."
            )
            self.stop_music()
            self.current_music_pub.publish(String(data="(missing file)"))
            return

        # Stop the previous track before starting a new one
        self.stop_music()

        # Build the player command
        if self.player_cmd == "ffplay":
            # ffplay: -nodisp (no video window), -autoexit (exit at end), quiet log
            cmd = [self.player_cmd, "-nodisp", "-autoexit", "-loglevel", "quiet", filepath]
        else:
            # Generic case (e.g. mpg123)
            cmd = [self.player_cmd, filepath]

        try:
            self.current_process = subprocess.Popen(cmd)
            self.get_logger().info(f"Playing mood '{mood}' with file '{filepath}'")
            self.current_music_pub.publish(String(data=filename))
        except FileNotFoundError:
            self.get_logger().error(
                f"Player command '{self.player_cmd}' not found. "
                f"Please install it or update the 'player_cmd' parameter."
            )
            self.current_process = None
            self.current_music_pub.publish(String(data="(play not found)"))

    # ------------------------------------------------------------------ #
    #  ROS callbacks                                                     #
    # ------------------------------------------------------------------ #

    def mood_callback(self, msg: String):
        """
        Handle incoming mood_state messages.
        Only react when the mood changes.
        """
        mood = msg.data.strip()

        # Ignore if mood did not change
        if mood == self.current_mood:
            return

        self.get_logger().info(f"Received new mood: '{mood}'")
        self.current_mood = mood
        self.play_music_for_mood(mood)

    def destroy_node(self):
        """
        Ensure that the player process is terminated on shutdown.
        """
        self.stop_music()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MusicPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

