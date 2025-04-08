#!/bin/bash

# Starts the audio stream from the USB microphone using GStreamer

# Path additions for GStreamer plugins
LIBCAMERA_PLUGIN_PATH="/opt/libcamera/build/src/gstreamer"
export GST_PLUGIN_PATH="${LIBCAMERA_PLUGIN_PATH}:${GST_PLUGIN_PATH}"

# Get the IP address of the remote PC (ThinkPad)
REMOTE_PC_IP="192.168.14.44"

# GStreamer pipeline to capture audio from the webcam and send it to the remote PC
gst-launch-1.0 -v \
  alsasrc device=hw:3,0 ! \
  "audio/x-raw, format=S16LE, rate=48000, channels=1" ! \
  audioconvert ! \
  audioresample ! \
  audio/x-raw,channels=1 ! \
  opusenc bitrate=16000 ! \
  rtpopuspay ! \
  udpsink host=$REMOTE_PC_IP port=5002
