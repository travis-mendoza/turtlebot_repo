#!/bin/bash

# Opens the Raspberry Pi video stream using GStreamer
# The video stream must be started on the rpi before this script can work

gst-launch-1.0 -v \
    udpsrc port=5000 \
    ! "application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264,payload=(int)96" \
    ! rtph264depay \
    ! h264parse \
    ! avdec_h264 \
    ! videoconvert \
    ! autovideosink sync=false
