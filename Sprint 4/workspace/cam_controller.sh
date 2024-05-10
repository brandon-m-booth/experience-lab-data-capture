#!/bin/bash

# Define color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

start_script="startcam.sh"
lock_file=".startcam_lock"

# Function to check if the startcam.sh process is running
check_startcam_running() {
  check_pid=$(cat "$lock_file")
  if [ -n "$check_pid" ] && ps -p "$check_pid" > /dev/null; then
    return 0
  else
    return 1
  fi
}

start_startcam() {
  if [ -e "$lock_file" ]; then
    echo -e "${RED}Another instance of $start_script is already running.${NC}"
  else
    ./$start_script &
    pid=$!

    # Store the main process PID (usb_cam) in the lock file
    echo "$pid" > "$lock_file"

    echo -e "${GREEN}Started $start_script (PID: $pid)${NC}"
  fi
}

start_recordcamcompressed() {
  if check_startcam_running; then
    if [ -e "$lock_file" ]; then
      # Start recording directly with the `ros2 bag record` command and log the output
      log_file="recordcam.log"
      {
        ros2 bag record /image_raw/compressed
      } > "$log_file" 2>&1 &

      pid=$!
      echo "Started recording (PID: $pid)"

      # You can also log the command itself
      echo "Recording command: ros2 bag record /image_raw/compressed" >> "$log_file"
    else
      echo -e "${RED}$start_script is not running.${NC}"
    fi
  else
    echo -e "${RED}$start_script is not running. Cannot start recording.${NC}"
  fi
}

stop_recordcamcompressed() {
  # Send a termination signal to the rosbag process
  pkill -SIGINT -f "ros2 bag record"

  # Check if the process was successfully terminated
  if [ "$?" -eq "0" ]; then
    echo "Stopped recording gracefully."
  else
    echo "Failed to stop recording gracefully. Attempting to force stop..."
    
    # If SIGINT didn't work, try a forceful termination
    pkill -SIGTERM -f "ros2 bag record"

    if [ "$?" -eq "0" ]; then
      echo "Recording forcefully stopped."
    else
      echo "Failed to stop recording forcefully. You may need to manually terminate the process."
    fi
  fi
}


stop_startcam() {
  if [ -e "$lock_file" ]; then
    pid=$(cat "$lock_file")

    if [ -n "$pid" ]; then
      # Use pkill to attempt to stop the startcam.sh process
      pkill -f "usb_cam"
      
      # Check the exit status of pkill (0 means success, 1 means no process matched)
      if [ "$?" -eq "0" ]; then
        echo -e "${GREEN}Stopped $start_script (PID: $pid)${NC}"
      else
        echo -e "${RED}Failed to stop $start_script (PID: $pid)${NC}"
      fi

      rm -f "$lock_file"
    else
      echo -e "${RED}Error: Invalid PID found in the lock file.${NC}"
    fi
  else
    echo -e "${YELLOW}$start_script is not running.${NC}"
  fi
}

menu() {
  while true; do
    # Sourcing ROS2
    source /opt/ros/humble/setup.bash
    echo -e "Choose an option:"
    echo -e "1. ${GREEN}ENABLE Camera Node${NC}"
    echo -e "2. ${GREEN}DISABLE Camera Node${NC}"
    echo -e "3. ${YELLOW}START Recording${NC}"
    echo -e "4. ${YELLOW}STOP Recording${NC}"
    echo -e "5. ${RED}Quit${NC}"
    read -p "Enter your choice: " choice

    case $choice in
      1)
        start_startcam
        ;;
      2)
        stop_startcam
        ;;
      3)
        start_recordcamcompressed
        ;;
      4)
        stop_recordcamcompressed
        ;;
      5)
        echo "Goodbye!"
        # Remove the lock file and return to the terminal prompt
        rm -f "$lock_file"
        return
        ;;
      *)
        echo -e "${RED}Invalid choice${NC}"
        ;;
    esac
  done
}

menu

