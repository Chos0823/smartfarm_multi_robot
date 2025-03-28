import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="apriltag_detector",
            executable="smart_farm.py",
            name="smart_farm_ui",
            output="screen",
        ),
        Node(
            package="apriltag_detector",
            executable="yolo_node.py",
            name="yolo_node",
            output="screen",
        )
    ])

