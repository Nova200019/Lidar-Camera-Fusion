##################################################################################################
# Author: Soumyadip Banerjee                                                                     #
# Website: https://www.philosopherscode.de                                                       #
# github:  https://github.com/Nova200019                                                         #
# Disclaimer: This code is for educational purposes and provided "as-is" without any warranties. #
##################################################################################################
import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
import os
import logging

# Set up logging
logging.basicConfig(filename='lidar_camera_calibration.log', level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s')
os.system("source /opt/ros/melodic/setup.bash")
os.system("source ~/catkin_ws/devel/setup.bash")
def calibrate_lidar_camera():
    # Initialize ROS
    rospy.init_node('lidar_camera_calibration', anonymous=True)
    logging.info('ROS node initialized.')

    # Create subscribers for LiDAR and camera data
    lidar_sub = rospy.Subscriber('/livox/lidar', PointCloud2, lidar_callback)
    camera_sub = rospy.Subscriber('/usb_cam/image_raw', Image, camera_callback)
    logging.info('Subscribers for LiDAR and camera data created.')

    # Wait for messages to be received
    logging.info('Waiting for LiDAR and camera data...')
    rospy.spin()

def lidar_callback(lidar_msg):
    global lidar_data_received
    lidar_data_received = pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)
    logging.info('LiDAR data received.')
    if camera_image_received is not None:
        perform_calibration()

def camera_callback(camera_msg):
    global camera_image_received
    bridge = CvBridge()
    camera_image_received = bridge.imgmsg_to_cv2(camera_msg, desired_encoding='bgr8')
    logging.info('Camera data received.')
    if lidar_data_received is not None:
        perform_calibration()

def perform_calibration():
    global lidar_data_received, camera_image_received

    # Convert LiDAR data to numpy array
    lidar_data = np.array(list(lidar_data_received))
    logging.debug(f'LiDAR data converted to numpy array with shape: {lidar_data.shape}')

    # Perform calibration
    rotation_matrix, translation_vector = extrinsic_calibration(lidar_data, camera_image_received)

    # Save the extrinsic parameters
    np.savez('extrinsic_parameters.npz', rotation_matrix=rotation_matrix, translation_vector=translation_vector)
    logging.info('Extrinsic parameters saved to extrinsic_parameters.npz')

    # Output the transformation as an environment variable
    transform_str = f'[{",".join(map(str, rotation_matrix.flatten()))},{",".join(map(str, translation_vector.flatten()))}]'
    with open('lidar_camera_transform.env', 'w') as f:
        f.write(f'LIDAR_CAMERA_TRANSFORM={transform_str}\n')
    logging.info('Created lidar_camera_transform.env')

    # Set environment variable
    os.environ['LIDAR_CAMERA_TRANSFORM'] = transform_str
    logging.info('Environment variable LIDAR_CAMERA_TRANSFORM set.')

def extrinsic_calibration(lidar_points, camera_image):
    # Feature detection and matching
    orb = cv2.ORB_create()
    kp_camera, des_camera = orb.detectAndCompute(camera_image, None)
    logging.debug(f'Detected {len(kp_camera)} features in camera image.')

    # Select keypoints in LiDAR point cloud
    lidar_keypoints = select_keypoints_in_point_cloud(lidar_points, 500)
    logging.debug(f'Selected {lidar_keypoints.shape[0]} keypoints in LiDAR point cloud.')

    # Match features between LiDAR and camera
    matched_lidar_idx, matched_camera_idx = match_features(lidar_keypoints, des_camera, kp_camera)
    logging.debug(f'Matched {len(matched_lidar_idx)} features between LiDAR and camera.')

    # Extract matched points
    matched_lidar_points = lidar_keypoints[matched_lidar_idx]
    matched_camera_points = np.array([kp_camera[idx].pt for idx in matched_camera_idx])

    # Estimate transformation using matched points
    rotation_matrix, translation_vector = estimate_transformation(matched_lidar_points, matched_camera_points)
    logging.debug('Estimated rotation matrix and translation vector.')
    return rotation_matrix, translation_vector

def select_keypoints_in_point_cloud(lidar_points, num_points):
    indices = np.random.choice(lidar_points.shape[0], num_points, replace=False)
    keypoints = lidar_points[indices]
    logging.debug(f'Selected keypoints from LiDAR point cloud.')
    return keypoints

def match_features(lidar_keypoints, camera_features, camera_keypoints):
    # Placeholder function for matching features between LiDAR and camera data
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(camera_features, lidar_keypoints)
    matches = sorted(matches, key=lambda x: x.distance)

    matched_camera_idx = np.array([m.queryIdx for m in matches])
    matched_lidar_idx = np.array([m.trainIdx for m in matches])
    logging.debug('Matched features between LiDAR and camera data.')
    return matched_lidar_idx, matched_camera_idx

def estimate_transformation(lidar_points, camera_points):
    centroid_lidar = np.mean(lidar_points, axis=0)
    centroid_camera = np.mean(camera_points, axis=0)

    lidar_centered = lidar_points - centroid_lidar
    camera_centered = camera_points - centroid_camera

    H = np.dot(lidar_centered.T, camera_centered)

    U, S, Vt = np.linalg.svd(H)
    rotation_matrix = np.dot(Vt.T, U.T)

    if np.linalg.det(rotation_matrix) < 0:
        Vt[-1, :] *= -1
        rotation_matrix = np.dot(Vt.T, U.T)

    translation_vector = centroid_camera.T - np.dot(rotation_matrix, centroid_lidar.T)
    logging.debug('Computed transformation from LiDAR to camera points.')
    return rotation_matrix, translation_vector

if __name__ == '__main__':
    lidar_data_received = None
    camera_image_received = None
    try:
        calibrate_lidar_camera()
    except rospy.ROSInterruptException:
        logging.error('ROS Interrupt Exception occurred.')
