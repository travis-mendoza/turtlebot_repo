#!/bin/bash
source .env

export AWS_DEFAULT_REGION=us-west-2

export GST_PLUGIN_PATH=/opt/libcamera/build/src/gstreamer 
export GST_PLUGIN_PATH=/home/trav/Downloads/kvs-producer-sdk-cpp/build/libgstkvssink.so:$GST_PLUGIN_PATH 

gst-launch-1.0 autovideosrc \
  ! videoconvert \
  ! video/x-raw,format=I420,width=640,height=480 \
  ! x264enc bframes=0 key-int-max=45 tune=zerolatency byte-stream=true speed-preset=ultrafast \
  ! h264parse \
  ! video/x-h264,stream-format=avc,alignment=au,profile=baseline \
  ! kvssink stream-name="ez-live-streaming-raspi-2"
