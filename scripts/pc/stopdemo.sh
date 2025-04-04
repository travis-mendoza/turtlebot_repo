#!/bin/bash

pkill -f "ros2 launch turtlebot3_cartographer cartographer.launch.py"
pkill -f "ros2 launch turtlebot3_navigation2 navigation2.launch.py"

# Correct pkill commands for the receiving streams:
pkill -f "gst-launch-1.0.*udpsrc port=5000"  # Video
pkill -f "gst-launch-1.0.*udpsrc port=5002"  # Audio

echo "All systems stopped."
