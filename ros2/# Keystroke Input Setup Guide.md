# Keystroke Input Setup Guide

This guide will help you set up and run the Keystroke Input project on Ubuntu and Linux. The project has been built using ROS 2 and colcon.

## Prerequisites

Before you begin, make sure you have the following prerequisites installed:

1. **Ubuntu or Linux Environment:** You should be running Ubuntu or a Linux distribution.

2. **ROS 2 Humble Hawksbill:** If you don't have ROS 2 installed, you can follow the installation instructions at [ROS 2 Installation](https://index.ros.org/doc/ros2/Installation/Humble/).

3. **Colcon:** You need colcon to build and manage ROS 2 packages. Install colcon using pip:

   ```
   pip install colcon-common-extensions
   ```

4. **VSCODE** If you are using VS Code like me go ahead and download the Microsoft ROS extension it covers both ROS and ROS2

## Running the Project

Assuming that your partner has already cloned the repository and built the project, here are the steps for running the Keystroke Input project:

1. **Source the ROS 2 Setup:**

   Before running ROS 2 commands, you need to source the ROS 2 setup file. This is typically done by sourcing the `install/setup.bash` file in your project directory:

   ```
   source /path/to/your/project/install/setup.bash
   ```

   Replace `/path/to/your/project` with the actual path to your project directory.

   **YOU WILL NEED TO BE IN ROS2 to run this, as I have already built it**
   **so when you are in ros2 ws, you will need to source/install.bash**
   **then you can run the ros2 nodes**

2. **Run Keystroke Publisher:**

   You can run the Keystroke Publisher with the following command:

   ```
   ros2 run keystroke_input keystroke_publisher
   ```

3. **Run Keystroke Listener:**

   You can run the Keystroke Listener with the following command:

   ```
   ros2 run keystroke_input keystroke_listener
   ```

## Troubleshooting

If you encounter issues or missing dependencies, ensure you have the following packages installed:

- `rclpy`
- `std_msgs`
- `pynput`

You can install these packages using ROS 2's package manager `rosdep` or using `pip`. For example:

```
pip install pynput keyboard
pip install 
```

