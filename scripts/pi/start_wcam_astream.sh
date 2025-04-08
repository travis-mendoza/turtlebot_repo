#!/bin/bash

# Starts the audio stream from the webcam mounted to the rpi using GStreamer

# Get the IP address of the remote PC (ThinkPad)
REMOTE_PC_IP="192.168.14.44"

# GStreamer pipeline to capture audio from the webcam and send it to the remote PC
gst-launch-1.0 -v \
  alsasrc device=hw:3,0 ! \
  "audio/x-raw, format=S16LE, rate=32000, channels=2" ! \
  audioconvert ! \
  audioresample ! \
  audio/x-raw,channels=1 ! \
  opusenc bitrate=16000 ! \
  rtpopuspay ! \
  udpsink host=$REMOTE_PC_IP port=5002
