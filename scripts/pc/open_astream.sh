#!/bin/bash

# Opens the audio stream from the webcam using GStreamer
# The audio stream must be started on the rpi before this script will work

# GStreamer pipeline to receive audio from the Raspberry Pi and play it using playbin
gst-launch-1.0 -v \
  udpsrc port=5002 caps="application/x-rtp, media=(string)audio, clock-rate=(int)48000, encoding-name=(string)OPUS, payload=(int)96" ! \
  rtpopusdepay ! \
  opusdec ! \
  audioconvert ! \
  audioresample ! \
  autoaudiosink
