# To open the camera controller:
	# Open a terminal in the same folder as cam_controller.sh
	# Enter 'sudo bash cam_controller.sh'
	# In order to start recording, the camera node needs to be running first.  Enter '1' to enable the camera.
	# While the camera node is running, press 3 to start recording.  You should only be able to run one camera node and one node recording thread at a time.
	# To close cleanly, you should first stop recording, then turn off the camera node.
	# Enter '4' to stop recording and save the bag, then enter '2' to turn off the camera.
	# Enter '5' to quit.





---SHELL COMMANDS FOR EXPERT USERS---

## Killing nodes manually
	ps aux | grep <node_name>
	# in this case you would want to find 'usb_cam' nodes and kill them, if a zombie node occurs
	kill -9 <PPID>


## Launch camera
	# run the executable with default settings (without params file)
	ros2 run usb_cam usb_cam_node_exe

	# run the executable while passing in parameters via a yaml file
	# Launch the camera with our CURRENT CONFIG
	ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /opt/ros/humble/share/usb_cam/config/params_1.yaml
	
	# launch the usb_cam executable that loads parameters from the same `usb_cam/config/params.yaml` file as above along with an additional image viewer node
	ros2 launch usb_cam camera.launch.py


## Bags
	# recording bags
	ros2 topic list
	ros2 bag record <topic name>

	# recording a bag with multiple topics at once
	ros2 bag record -o <chosen name> <topic 1 name> <topic 2 name> <topic ...>

	# how to play back a bag
	ros2 bag play <bag path>

	# bag info
	ros2 bag info <bag dir>
