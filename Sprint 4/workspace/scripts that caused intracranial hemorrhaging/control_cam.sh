#!/bin/bash

# Define color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

start_script="startcam.sh"
lock_file=".startcam_lock"

start_script() {
  if [ -e "$lock_file" ]; then
    echo "Another instance of $start_script is already running."
  else
    ./$start_script &
    pid=$!

    # Store the main process PID (usb_cam) in the lock file
    echo "$pid" > "$lock_file"

    echo "Started $start_script (PID: $pid)"
  fi
}

stop_script() {
  if [ -e "$lock_file" ]; then
    pid=$(cat "$lock_file")

    if [ -n "$pid" ]; then
      # Use pkill to attempt to stop the usb_cam process
      pkill -f "usb_cam"
      
      # Check the exit status of pkill (0 means success, 1 means no process matched)
      if [ "$?" -eq "0" ]; then
        echo "Stopped $start_script (PID: $pid)"
      else
        echo "Failed to stop $start_script (PID: $pid)"
      fi

      rm -f "$lock_file"
    else
      echo "Error: Invalid PID found in the lock file."
    fi
  else
    echo "$start_script is not running"
  fi
}

menu() {
  while true; do
    echo -e "###########################################"
    echo -e "Choose an option:"
    echo -e "1. ${GREEN}Turn ON camera${NC}"
    echo -e "2. ${RED}Turn OFF camera${NC}"
    echo -e "3. ${YELLOW}Quit${NC}"
    read -p "Enter your choice: " choice    

    case $choice in
      1)
        start_script
        ;;
      2)
        stop_script
        ;;
      3)
        echo "Goodbye!"
        rm -f "$lock_file"
        # exit 0
        return
        ;;
      *)
        echo "Invalid choice"
        ;;
    esac
  done
}

menu

