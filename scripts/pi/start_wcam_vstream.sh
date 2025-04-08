#!/bin/bash

# Starts a video stream from the webcam mounted to the rpi using GStreamer

# Get IP address of the remote PC (replace with a static IP if you have one)
REMOTE_PC_IP=$(avahi-resolve -n -4 ernie.local | awk '{ print $2 }')
# REMOTE_PC_IP=192.168.14.44

# GStreamer pipeline to capture from webcam, encode, and send over UDP
gst-launch-1.0 -v \
    v4l2src device=/dev/video0 \
    ! "image/jpeg,width=1920,height=1080,framerate=30/1" \
    ! jpegparse \
    ! jpegdec \
    ! videorate \
    ! "video/x-raw,framerate=30/1" \
    ! x264enc tune=zerolatency bitrate=4096 speed-preset=superfast \
    ! h264parse \
    ! rtph264pay config-interval=1 pt=96 \
    ! udpsink host=$REMOTE_PC_IP port=5000
