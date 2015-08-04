source /opt/ros/indigo/setup.bash
source /elab_dev/usc-experience-lab/data_capture/ros/devel/setup.bash
pushd .
roscd player_eye_tracking
cd third_party/TobiiGazeSdk-CApi-4.1.0.806-linux64/Samples/wxWidgetsCalibrationSample
./calibrationsample
popd
