#!/bin/bash
source .env

# --- Configuration ---
KVS_STREAM_NAME="ez-live-streaming-raspi-2"
AWS_REGION="us-west-2"
export AWS_DEFAULT_REGION="${AWS_REGION}"
# --- GStreamer Plugin Paths ---
# Adjust these paths based on your actual installation locations
LIBCAMERA_PLUGIN_PATH="/opt/libcamera/build/src/gstreamer"
KVS_PRODUCER_BUILD_PATH="/home/trav/Downloads/kvs-producer-sdk-cpp/build"

# Prepend KVS sink path, then prepend libcamera path (order might matter depending on dependencies)
# Make sure both directories exist and contain the necessary plugin files (.so)
export GST_PLUGIN_PATH="${KVS_PRODUCER_BUILD_PATH}:${LIBCAMERA_PLUGIN_PATH}:${GST_PLUGIN_PATH}"

# --- GStreamer Pipeline ---
# Use libcamerasrc for Raspberry Pi Camera Module v2/v3 with libcamera

echo "GST_PLUGIN_PATH=${GST_PLUGIN_PATH}"
echo "Starting GStreamer pipeline..."

gst-launch-1.0 -v \
    libcamerasrc \
    ! video/x-raw,width=640,height=480,framerate=30/1 \
    ! videoconvert \
    ! x264enc bitrate=1000 bframes=0 key-int-max=30 tune=zerolatency speed-preset=ultrafast byte-stream=true \
    ! video/x-h264,profile=baseline,stream-format=avc,alignment=au \
    ! h264parse \
    ! kvssink stream-name="${KVS_STREAM_NAME}" storage-size=512

# Notes on gst-launch options:
# -v : Verbose output, helpful for debugging.
# libcamerasrc: Source element for libcamera compatible cameras on RPi.
# video/x-raw...: Specify desired output format from the camera. Adjust width/height/framerate as needed.
# videoconvert: Handles color space conversion if needed (libcamerasrc might output something x264enc doesn't directly accept).
# x264enc: H.264 encoder.
#   - bitrate=1000: Target bitrate in kbps. Adjust based on network and desired quality.
#   - bframes=0 key-int-max=30: Common settings for low-latency streaming. key-int-max controls GOP size (e.g., 30 frames).
#   - tune=zerolatency speed-preset=ultrafast: Prioritize speed and low latency over quality/compression efficiency.
#   - byte-stream=true: Important for some streaming sinks.
# video/x-h264...: Specify caps for the encoded stream. profile=baseline is widely compatible. stream-format=avc and alignment=au are often required by KVS.
# h264parse: Parses the H.264 stream, makes it easier for downstream elements.
# kvssink: The AWS KVS sink plugin.
#   - storage-size=512: Optional: KVS producer SDK buffer size in MB. Default is 128. Increase if you see buffer-related warnings/errors.
