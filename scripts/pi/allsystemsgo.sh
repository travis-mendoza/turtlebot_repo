#!/bin/bash

# Starts the video, audio streams, and the Turtlebot.

# Start Turtlebot
ros2 launch turtlebot3_bringup robot.launch.py &
TURTLEBOT_PID=$!

# Start Video
$HOME/scripts/start_vstream.sh &
VIDEO_PID=$!

# Start Audio
$HOME/scripts/start_astream.sh &
AUDIO_PID=$!


# Check for errors (like the improved version above)
if [[ $? -eq 0 ]]; then # Check if the last command (start_audio.sh) was successful
  echo "Audio stream started."
else
  echo "Error starting audio stream."
  kill $VIDEO_PID # Kill the video stream if audio failed
  kill $TURTLEBOT_PID
  exit 1 # Exit with an error code
fi

if [[ $? -eq 0 ]]; then # Check if the last command (start_video.sh) was successful
    echo "Video stream started."
else
    echo "Error starting video stream."
    kill $AUDIO_PID # Kill the audio stream if video failed
    kill $TURTLEBOT_PID
    exit 1 # Exit with an error code
fi

if [[ $? -eq 0 ]]; then # Check if the last command (ros2 launch) was successful
    echo "Turtlebot launched."
else
    echo "Error launching Turtlebot."
    kill $AUDIO_PID # Kill the audio stream if video failed
    kill $VIDEO_PID
    exit 1 # Exit with an error code
fi


echo "All systems started in the background."
