#!/bin/bash

# Launch Cartographer, Navigation2, audio, and video streams.

# Launch Cartographer
ros2 launch turtlebot3_cartographer cartographer.launch.py &
CARTOGRAPHER_PID=$!

echo "Launching Cartographer..."

# Check if Cartographer is running (up to 10 seconds)
TIMEOUT=10
for i in $(seq 1 $TIMEOUT); do
  if ros2 topic list | grep -q /scan; then # Check for the /scan topic (a good indicator)
    echo "Cartographer appears to be running."
    break  # Exit the loop if /scan is found
  fi
  echo "Waiting for Cartographer (attempt $i/$TIMEOUT)..."
  sleep 1
done

# If the loop finished without finding /scan, exit with an error
if [[ $i -gt $TIMEOUT ]]; then
  echo "Error: Cartographer did not start within $TIMEOUT seconds."
  kill $CARTOGRAPHER_PID # Kill cartographer
  exit 1
fi

# Launch Navigation2
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml &
NAVIGATION2_PID=$!

echo "Launching Navigation2..."

# Launch Audio Stream
./open_astream.sh &
AUDIO_PID=$!

echo "Launching Audio Stream..."

# Launch Video Stream
./open_vstream.sh &
VIDEO_PID=$!

echo "Launching Video Stream..."

# Check for errors (like the improved version above)
if [[ $? -eq 0 ]]; then # Check if the last command (open_vstream.sh) was successful
  echo "Video Stream started."
else
  echo "Error starting Video Stream."
  kill $AUDIO_PID # Kill the audio stream if video failed
  kill $NAVIGATION2_PID
  kill $CARTOGRAPHER_PID
  exit 1 # Exit with an error code
fi

if [[ $? -eq 0 ]]; then # Check if the last command (open_astream.sh) was successful
    echo "Audio Stream started."
else
    echo "Error starting Audio Stream."
    kill $VIDEO_PID # Kill the video stream if audio failed
    kill $NAVIGATION2_PID
    kill $CARTOGRAPHER_PID
    exit 1 # Exit with an error code
fi

if [[ $? -eq 0 ]]; then # Check if the last command (ros2 launch nav2) was successful
    echo "Navigation2 launched."
else
    echo "Error launching Navigation2."
    kill $AUDIO_PID # Kill the audio stream if nav2 failed
    kill $VIDEO_PID
    kill $CARTOGRAPHER_PID
    exit 1 # Exit with an error code
fi

if [[ $? -eq 0 ]]; then # Check if the last command (ros2 launch cartographer) was successful
    echo "Cartographer launched."
else
    echo "Error launching Cartographer."
    kill $AUDIO_PID # Kill the audio stream if cartographer failed
    kill $VIDEO_PID
    kill $NAVIGATION2_PID
    exit 1 # Exit with an error code
fi


echo "All systems started in the background."
