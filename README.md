# Dimos-LicaFuse - Landing Page

LiCa-Fuse is a student research project which aims at enabling the fusion of Camera and LiDAR data. 
The resulting model of this fusion detects objects as well as their position in a certain environment. 
This data can then be used for various applications, such as autonomous driving or surveillance systems.

## Repository structure

This repository is structured as follows: 

This main branch describes the implementation of the fusion of camera and lidar data. The corresponding guide can be found in the [fusion-guide](./fusion-guide.md).

Other parts of the project which contain some form of code can be found in the [tools directory](./tools). Each sub-directory consists of the necessary code and assets as well as a specific README.md file.

The tools directory includes 
- [Retrieval of pointcloud data and converting them from .lvx to .pcd files](./tools/Retrival_of%20_point_cloud_data?ref_type=heads)
- [Object detection using pointcloud clustering](./tools/object_detection_node?ref_type=heads)
- [Calibration of camera and LiDAR using Point To Plane method](./tools/point2planeCalibration?ref_type=heads)
- [Streaming of pointcloud data as an image to a webserver](./tools/streaming?ref_type=heads)

as well as some files not allocated to a specific sub-directory. Further information on the purpose of these files can be found under [Useful Tools](./fusion-guide.md#useful-tools) in the fusion guide.

Additionally, these sub-projects can also be found in their respective **feature branches**. This sturcture is aiming at providing a good base for further development on this project and to make the separate sub-projects easier editible for developers.

Parts of the project that do **not** contain any code, for example research results, can be found [in the Wiki](https://git-ce.th-rosenheim.de/pse-24/Dimos-LiCaFuse/-/wikis/home) under the "Product" index.

### Responsibilities

To gain an insight into which team member was responsible for which tasks, you can check [Responsibilities](https://git-ce.th-rosenheim.de/pse-24/Dimos-LiCaFuse/-/wikis/Project-Organization/Responsibilities?redirected_from=Project-Organization/Roles) in the Wiki.

## Contents of delivery

### Software

This repository provides detailed instructions on how to install and use the provided tools as well as insights into research results obtained during the course of this project. 

With this repository, you will be able to implement **early fusion of camera images with LiDAR point clouds**. This data can then be used to determine the position of an object as well as its classification. At the time of the end of this project, this process only works for data from the KITTI dataset. In order to use images and point clouds directly from your own camera/LiDAR sensor, calibration will still be necessary. The obtained object data is then transmitted to an **MQTT broker** through a JSON message. 

Alternatively, you can find preliminary results of using **clustering of pointclouds** in order to enable object detection. 

You will also find instructions on how to enable **streaming of point cloud data to a web server as an image**. These instructions are to be understood as base for further developments as the results are not quite what was expected (see feature branch or tools directory for further information).

One step during the project was to **collect point cloud data** in the format of .lvx files and distribute them as .pcd files. The documentation for this process can also be found in this repository. 

For research results on **YOLOv5/YOLOv8 for object recognition**, **Calibration** using a 3D checkerboard and the Point to Plane method and **Object Detection with clustering** refer to the wiki.  

### Hardware

During the hand-over, the customer will receive a **Jetson** device with all contents of this repository installed. The **ROS image** of the device will also be uploaded to Nextcloud.

Additionally, a **3D printed mount** for the LiDAR and camera will also be handed over. Its documentation can be found in the *assets* folder. 

## Technical Requirements

- Ubuntu 18.04.
- ROS Melodic
- Matlab
- Python 3.6+
- python libraries listed in requirements.txt
