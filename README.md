# Smart Farm Multi-Robot System

This project implements a smart farm simulation using a **multi-robot system** in **ROS2**. It features autonomous navigation, SLAM mapping, AprilTag detection, and YOLOv8-based object recognition. The environment is built using Gazebo and RViz, and includes launch files for various components to make testing and deployment easier.

## Features

- 🧠 Multi-Robot Coordination
- 🗺️ SLAM-based Mapping & Localization (`slam_toolbox`)
- 📷 AprilTag Detection for Visual Localization
- 🤖 YOLOv8 Object Recognition
- 🌍 Gazebo Simulation with pre-built smart farm world
- 🧭 Navigation with Nav2 (including costmaps, AMCL)
- 🧰 Custom launch files for streamlined operation

## Project Structure

```
b4_ws/
├── src/
│   ├── articubot_one/       # Robot simulation configuration and launch files
│   └── apriltag_detector/   # AprilTag-based perception node
├── my_map_save.yaml         # Saved map data
├── yolov8m.pt               # YOLOv8 object detection model
└── README.md
```

## Quick Start

```bash
# Launch simulation with Gazebo
ros2 launch articubot_one launch_sim.launch.py world:=$HOME/b4_ws/src/articubot_one/worlds/obstacles.world

# Start SLAM (localization mode)
ros2 launch articubot_one online_async_launch.py mode:=localization

# Launch navigation
ros2 launch articubot_one navigation_launch.py
```

> For detailed launch options, refer to the `README1bot.md` or `README1bot simple launch files.md`.

## License

This project is licensed under the **MIT License**.
