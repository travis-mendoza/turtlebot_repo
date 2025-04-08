#!/bin/bash

# Starts a video stream from the Raspberry Pi Camera Module using GStreamer

# Path additions
LIBCAMERA_PLUGIN_PATH="/opt/libcamera/build/src/gstreamer"
export GST_PLUGIN_PATH="${LIBCAMERA_PLUGIN_PATH}:${GST_PLUGIN_PATH}"

# Get IP address of the remote PC (replace with a static IP if you have one)
REMOTE_PC_IP=$(avahi-resolve -n -4 ernie.local | awk '{ print $2 }')
# REMOTE_PC_IP=192.168.14.44

echo "GST_PLUGIN_PATH=${GST_PLUGIN_PATH}"
echo "Starting GStreamer pipeline..."
# GStreamer pipeline to capture from webcam, encode, and send over UDP
gst-launch-1.0 -v \
    libcamerasrc \
    ! video/x-raw,width=1920,height=1080,framerate=30/1 \
    ! videoconvert \
    ! videorate \
    ! video/x-raw,framerate=30/1 \
    ! x264enc tune=zerolatency bitrate=4096 speed-preset=superfast \
    ! h264parse \
    ! rtph264pay config-interval=1 pt=96 \
    ! udpsink host=$REMOTE_PC_IP port=5000
