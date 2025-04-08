#!/bin/bash

# Stops the audio stream from the webcam.

pkill -f "gst-launch-1.0.*alsasrc device=hw:3,0"

echo "Audio stream stopped."
