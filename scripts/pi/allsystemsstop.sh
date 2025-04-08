#!/bin/bash

# Stops the video, audio streams, and the Turtlebot.

pkill -f "ros2 launch turtlebot3_bringup robot.launch.py"
pkill -f "gst-launch-1.0.*v4l2src device=/dev/video0"
pkill -f "gst-launch-1.0.*alsasrc device=hw:3,0"

echo "All systems stopped."
