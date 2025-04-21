#!/bin/bash

##################################################################################################
# Author: Soumyadip Banerjee                                                                     #
# Website: https://www.philosopherscode.de                                                       #
# github:  https://github.com/Nova200019                                                         #
# Disclaimer: This code is for educational purposes and provided "as-is" without any warranties. #
##################################################################################################

# Setup ROS environment
source /opt/ros/melodic/setup.bash # adjust the file path
source /home/sb/catkin_ws/devel/setup.bash # adjust the file path

# Print out ROS setup
echo "ROS Setup:"

# Start roscore in the background
roscore &
ROSCORE_PID=$!
sleep 5

# Start USB Camera Node
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _image_width:=1280 _image_height:=720 _pixel_format:=yuyv _camera_frame_id:=usb_cam _framerate:=30 _io_method:=mmap _camera_info_url:=file:////home/sb/lica/usb_cam.yml & # adjust the file path
USB_CAM_PID=$!
sleep 5

# Start Livox LiDAR Publisher
roslaunch livox_ros_driver livox_lidar.launch &
LIVOX_PID=$!
sleep 5

# Start Image Processing Node for mono camera
rosrun image_proc image_proc __ns:=/usb_cam _image_raw:=/usb_cam/image_raw _camera_info:=/usb_cam/camera_info _image_mono:=/usb_cam/image_mono _image_color:=/usb_cam/image_color _image_rect:=/usb_cam/image_rect &
IMAGE_PROC_PID=$!
sleep 5


# Verify nodes and topics
echo "Running ROS Nodes:"
rosnode list

echo "Active ROS Topics:"
rostopic list

# Publish static transform between LiDAR and Camera
rosrun tf static_transform_publisher 0 0 0 0 0 0 usb_cam livox_frame 100 &
TF_STATIC_PID=$!
sleep 5

# Start RViz
echo "Starting RViz..."
rosrun rviz rviz 
RVIZ_PID=$!
sleep 5
export LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libgomp.so.1 /home/sb/.local/lib/python3.8/site-packages/scikit_learn.libs/libgomp-d22c30c5.so.1.0.0" # adjust the file path

source ~/.bashrc
 
# Keep the container running
wait $XVFB_PID $X11VNC_PID $USB_CAM_PID $LIVOX_PID $IMAGE_PROC_PID $TF_STATIC_PID $RVIZ_PID $PYTHON_PID $CALIBRATION_PID $ROSCORE_PID

