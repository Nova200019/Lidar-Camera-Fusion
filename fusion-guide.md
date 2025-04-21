# Dimos-LicaFuse - Initial Setup Guide

## Overview

This project implements object detection and tracking using early fusion of LiDAR and camera data. The project uses the YOLOv4 deep learning model for object detection. It can process data both in real-time using ROS (Robot Operating System) and from pre-recorded datasets stored as files. Below are the instructions and details for both approaches. Finally, the obtained data can be transmitted via MQTT.
### Useful tools:
1. **Navigate to the tools directory**
   ```sh
    cd tools
    ```
3. **bintopcd.py** is used to convert bin point-clouds to pcd format.
4. **calib-with-check.py** is used to calibrate lidar camera with a checkerboard.
5. **calibrate-with-matlab.m** is used to calibrate lidar camera with a checkerboard in matlab.
6. **pic.sh** is used to capture images from Adrucam using FFMPEG.
7. **calibrate-lidar-camera.py** is used to calibrate lidar camera without a checkerboard.


## Simplified General Architecture
**Here is a simplified general architechture used for early fusion for realtime data processing:**
![Alt text](./assets/dimos_arc.jpg?raw=true "")

## Getting started

### Step 1: Create Project Directory
- create a folder where you want to download and install Lica-Fuse.
- open command prompt and type:
    ```sh
    cd path/to/your/folder
    ```
### Step 2: Add remote
```sh
git config --global user.name "youserid1234"
git config --global user.email "youremail@example.de"
```
### Step 3: Install ROS Melodic (Ubuntu Bionic)
1. **For the purposes of installing ROS, we need to enable the Ubuntu Universe, Restricted, and Multiverse repositories:**
    ```sh
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo add-apt-repository restricted
    sudo add-apt-repository multiverse
    ```

2. **The ROS repository server details have to be fed into source.list, which is in /etc/apt/.**:
    ```sh
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```
3. **Set up Keys**:
    ```sh
   sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    ```
4. **Installing ROS Melodic**:
    ```sh
   sudo apt-get update
   sudo apt-get install ros-melodic-desktop-full
    ```
5. **Setting up the ROS environment**:
    ```sh
   echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc 
   source ~/.bashrc
    ```
or

[Follow this Link to ROS Installation](https://wiki.ros.org/melodic/Installation/Ubuntu)

### Step 4: Clone the Repository
1. **While being in same directory, type:**
    ```sh
    git clone https://git-ce.th-rosenheim.de/pse-24/Dimos-LiCaFuse.git
    ```
    ```sh
    cd Dimos-LiCaFuse
    ```
2. **Install Python Packages**:
    ```sh
    pip install -r requirements.txt
    ```
3. Follow the steps in **How to run**.

### Step 5: Import YOLO weights file in project
The fusion uses YOLOv4 for object detection. In order to run the model, you need a weights file which includes pre-trained weights. This file was too big to upload in this GitLab repo and was instead uploaded to [nextcloud](https://nextcloud.dimos-ops.com/s/SmR8jpLwBzWcYkw?path=%2FModel%20Files). 

Download the yolov4.weights file and import it manually in the [lica-fuse-without-ros/yolov4](./lica-fuse-without-ros/yolov4?ref_type=heads) directory.

### Step 6: Import .pcd files in project
As with the YOLO weight files, the .pcd files for the KITTI dataset were too big to be uploaded in this project and were uploaded the nextcloud instead. 

Follow the same link as above and import the .pcd files in [lica-fuse-without-ros/data/velodyne](.lica-fuse-without-ros/data/velodyne?ref_type=heads) directory.

## Early Fusion with ROS
The ROS-based implementation captures live data from a LiDAR sensor and a camera, processes the data to detect objects, and publishes the results. This approach is suitable for real-time applications where data is streamed from sensors mounted on a robot or a vehicle.

**Key Components:**
- **ROS Nodes:** The script initializes a ROS node to subscribe to LiDAR and camera topics.
- **YOLOv4**: A deep learning model for real-time object detection.
- **LiDAR to Camera Projection**: Projects LiDAR points onto the camera image for fusion.
- **Detection Results Publishing**: Publishes the detected objects and the fused image to ROS topics for visualization via Rviz.

### Architechture with ROS:
**Here is the architectural diagram with ros components:**

![Alt text](./assets/Fusion/ros.png?raw=true "")

### How to Run:

1. **Setup ROS Environment**: Ensure that the ROS environment is correctly sourced and ROS Master URI is set.

2. **Go to the directory**:
    ```sh
    cd ros-lica-fuse
    ```

3. **Launch ROS Nodes**: Run the ROS master and launch any required sensor drivers (after running the script please wait for a while to be completely executed).

     ```sh
    ./ros-entrypoint.sh
    ```

4. **Run the Script**:
    ```sh
    python3 roslicafuse.py
    ```
5. **Visualization**: Use RViz or any ROS-compatible visualization tool to view the published topics.

`fused_image/`: This topic in RVIZ (As Image topic) will show you the fused result

6. **See Console for generated messages**.





## Early Fusion with pre-recorded KITTI dataset:
 We have implemented fusion logic using pre-recorded dataset from [KITTI](https://www.cvlibs.net/datasets/kitti/).
### Architechture with KITTI (without ROS):
**Here is the architectural diagram with KITTI dataset:**

![Alt text](./assets/Fusion/noros.png?raw=true "")

### How to Run:
1. **Go to the directory**
     ```sh
    cd lica-fuse-without-ros
    ```
2. **Prepare Dataset**: Organize your dataset with images, point clouds, and calibration files.
    - `data/img/`: Directory containing image files.
    - `data/velodyne/`: Directory containing LiDAR point cloud files.
    - `data/calib/`: Directory containing calibration files.
4. **Run the Script**:
    ```sh
    python3 licafuse.py
    ```
5. **View Results**: The script will process each frame in the dataset and display the results.
7. **See Console for generated messages**.


## Output results:
1. **Output Image (with all lidar points including noise):**

![Alt text](./assets/Fusion/Fusion Result_screenshot_29.06.2024.png?raw=true "")
2. **Output Image (after filtering all  noise):**

![Alt text](./assets/Fusion/Fusionresult.png?raw=true "")

3. **Messege Generation for MQTT:**
   ```sh
   {
    "station_id": 1,
    "ts": "2024-06-29 21:30:10",
    "class": "person",
    "size": {
        "height": 1.6090000569820404,
        "width": 10.825999736785889,
        "depth": 14.034000396728516
    },
    "position": {
        "x": 15.21500015258789,
        "y": -11.458999872207642,
        "z": -0.9365000277757645
    }

4. **You can record a video with output frames like this or stream it over network:**


[Watch the video](./assests/Fusion/output.mp4)



## MQTT Configuration Instructions

This project uses MQTT to handle communication for object detection and 3D point cloud data. Follow the instructions below to set up the MQTT broker details for both production and local development environments.

### Production Environment
For a production environment, configure the MQTT broker details in the mqtt.config.ini file. This file should contain the following settings:

 ```sh
 [MQTT]
broker_address = localhost
port = 1883
topic = object_detection/3d_point_cloud
use_tls = False
cafile = path/to/ca.crt
certfile = path/to/client.crt
keyfile = path/to/client.key
 ```
### Production Environment
For local development, you can leave the mqtt.config.ini file unchanged. Instead, follow these steps to install and run the Mosquitto MQTT broker:

**On Ubuntu:**

 ```sh
sudo apt-get update
sudo apt-get install mosquitto mosquitto-clients
```





## Special Thanks :)
Special thanks to [lavinama](https://github.com/lavinama) for the early fusion idea.

**Coded with love by the Lica-Fuse team of PSE-24 :)**