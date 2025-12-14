# Smart Mood Therapy Music System (Arduino + ROS2)

This project is a simple mood-based music system.
Arduino reads environmental sensors (temperature + light) and sends values via serial.
ROS2 nodes bridge the sensor data, infer a mood state, and play a corresponding music track.

> Note: The project was initially planned as a mood lamp, so some folder/package names still contain `moodlamp`.

---

## System Overview

**Inputs**
- DHT11 temperature sensor (Arduino)
- LDR light sensor (Arduino)
- (Optional) keypad manual override (if enabled in your code)

**ROS2 Topics**
- `/sensor_values` → published by `sensor_bridge_node`
- `/mood_state` → published by `mood_fusion_node`
- `/current_music` → published by `music_player_node` (for UI display)

**Outputs**
- Plays local royalty-free audio using `ffplay` (or another CLI audio player)

---

## Repository Structure

src/moodlamp_sensors/
├─ moodlamp_sensors/
│ ├─ sensor_bridge_node.py
│ ├─ mood_fusion_node.py
│ ├─ music_player_node.py
│ └─ ui_node.py
├─ package.xml
├─ setup.py
├─ setup.cfg
├─ resource/
└─ test/


---

## Requirements

- Ubuntu + ROS2 (Python rclpy)
- Arduino IDE
- Python dependencies:
  - `pyserial` (for serial communication)
- Audio player:
  - `ffplay` (recommended)  
    Install on Ubuntu:
    ```bash
    sudo apt-get update
    sudo apt-get install -y ffmpeg
    ```

---

## Music Files

The music player expects a local folder named **`mood_music/`** at the ROS2 workspace root.


**Copyright note:**  
Audio files are royalty-free and/or not included in this repository to avoid copyright issues.
You can provide your own files with the same names.

---

## Build

From the workspace root (the folder that contains `src/`):

```bash
colcon build
source install/setup.bash
```

---

## Run (recommended order)

1) Start UI node (subscriber)
```bash
ros2 run moodlamp_sensors ui_node
```

3) Start music player node
```bash
ros2 run moodlamp_sensors music_player_node
```


If your music folder is not mood_music/ at workspace root, override the parameter:

ros2 run moodlamp_sensors music_player_node --ros-args -p music_dir:=/path/to/mood_music

3) Start mood fusion node
```bash
ros2 run moodlamp_sensors mood_fusion_node
```

5) Start sensor bridge node (reads Arduino serial)
```bash
ros2 run moodlamp_sensors sensor_bridge_node
```
---

## Arduino

Upload an Arduino sketch that prints sensor values over serial in a readable format.
Example expected format:

TEMP:26.00,LIGHT:720


Then connect Arduino via USB.

If your serial port differs, update the port setting in sensor_bridge_node.py
(e.g., /dev/ttyACM0 or /dev/ttyUSB0).
