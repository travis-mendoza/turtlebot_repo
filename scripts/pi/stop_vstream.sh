#!/bin/bash

# Stops the video stream from the webcam.

pkill -f "gst-launch-1.0.*v4l2src device=/dev/video0"

echo "Video stream stopped."
