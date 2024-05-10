#!/bin/bash

# Define variables
recording_pid=""
lock_file=".recording_lock"

# Function to start recording
start_recording() {
    if [ -n "$recording_pid" ] && ps -p "$recording_pid" &> /dev/null; then
        echo "Recording is already in progress (PID: $recording_pid)"
    else
        ros2 bag record /image_raw/compressed &
        recording_pid=$!
        echo "Started recording (PID: $recording_pid)"
    fi
}

# Function to stop recording
stop_recording() {
    if [ -n "$recording_pid" ] && ps -p "$recording_pid" &> /dev/null; then
        kill "$recording_pid"
        recording_pid=""
        echo "Stopped recording"
    else
        echo "No recording is currently active."
    fi
}

# Main menu
while true; do
    # Sourcing ROS2
    source /opt/ros/humble/setup.bash
    echo "Choose an option:"
    echo "1. Start Recording"
    echo "2. Stop Recording"
    echo "3. Quit"
    read -p "Enter your choice: " choice

    case $choice in
        1)
            start_recording
            ;;
        2)
            stop_recording
            ;;
        3)
            exit 0
            ;;
        *)
            echo "Invalid choice."
            ;;
    esac
done

