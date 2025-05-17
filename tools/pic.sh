#!/bin/bash
##################################################################################################
# Author: Soumyadip Banerjee                                                                     #
# Website: https://www.philosopherscode.de                                                       #
# github:  https://github.com/Nova200019                                                         #
# Disclaimer: This code is for educational purposes and provided "as-is" without any warranties. #
##################################################################################################
# Directory to save captured images
output_dir="./captured_images"
mkdir -p $output_dir

# Device name (adjust according to your system, e.g., /dev/video0 for Linux)
device="/dev/video0"

# Frame rate (frames per second)
frame_rate=1

# Duration to capture images (in seconds)
duration=10

# Capture 10 images
ffmpeg -y -f video4linux2 -framerate $frame_rate -video_size 640x480 -i $device -vf fps=1 $output_dir/image_%03d.jpg

echo "Captured 10 images and saved to $output_dir"

