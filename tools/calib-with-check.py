##################################################################################################
# Author: Soumyadip Banerjee                                                                     #
# Website: https://www.philosopherscode.de                                                       #
# github:  https://github.com/Nova200019                                                         #
# Disclaimer: This code is for educational purposes and provided "as-is" without any warranties. #
##################################################################################################
import os
import logging
import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
import numpy as np
from sklearn.neighbors import NearestNeighbors

# Logging configuration
logging.basicConfig(filename='lidar_camera_calibration.log', level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s')

logging.info('Script started')

# Ensure ROS environment is sourced
os.system("source /opt/ros/melodic/setup.bash")
os.system("source ~/catkin_ws/devel/setup.bash")

# Ensure ROS Master URI is set
if 'ROS_MASTER_URI' not in os.environ:
    os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'

logging.info('ROS environment sourced')

# Global variables to store received data
lidar_data_received = None
camera_image_received = None

def calibrate_lidar_camera():
    global lidar_data_received, camera_image_received
    
    # Initialize ROS
    rospy.init_node('lidar_camera_calibration', anonymous=True)
    logging.info('ROS node initialized.')
    print('ROS node initialized.')

    # Create subscribers for LiDAR and camera data
    rospy.Subscriber('/livox/lidar', PointCloud2, lidar_callback)
    rospy.Subscriber('/usb_cam/image_mono', Image, camera_callback)
    logging.info('Subscribers for LiDAR and camera data created.')
    print('Subscribers for LiDAR and camera data created.')

    # Wait for messages to be received
    logging.info('Waiting for LiDAR and camera data...')
    print('Waiting for LiDAR and camera data...')
    rospy.spin()

def lidar_callback(lidar_msg):
    global lidar_data_received
    logging.info('LiDAR callback triggered.')
    print('LiDAR callback triggered.')
    lidar_data_received = list(pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True))
    logging.info('LiDAR data received.')
    print('LiDAR data received.')
    if camera_image_received is not None:
        logging.info('Both LiDAR and camera data received. Performing calibration.')
        print('Both LiDAR and camera data received. Performing calibration.')
        perform_calibration()

def camera_callback(camera_msg):
    global camera_image_received
    logging.info('Camera callback triggered.')
    print('Camera callback triggered.')
    bridge = CvBridge()
    camera_image_received = bridge.imgmsg_to_cv2(camera_msg, desired_encoding='mono8')
    logging.info('Camera data received.')
    print('Camera data received.')
    if lidar_data_received is not None:
        logging.info('Both LiDAR and camera data received. Performing calibration.')
        print('Both LiDAR and camera data received. Performing calibration.')
        perform_calibration()

def perform_calibration():
    global lidar_data_received, camera_image_received
    logging.info('Starting calibration process.')
    print('Starting calibration process.')

    # Convert LiDAR data to numpy array
    lidar_data = np.array(lidar_data_received)

    # Detect checkerboard in camera image
    ret_camera, corners_camera = detect_checkerboard(camera_image_received, (7, 7))
    if not ret_camera:
        logging.error('Checkerboard not detected in camera image')
        print('Checkerboard not detected in camera image')
        return

    # Draw detected corners on the camera image
    camera_image_with_corners = cv2.drawChessboardCorners(camera_image_received, (7, 7), corners_camera, ret_camera)
    cv2.imwrite('camera_image_with_corners.png', camera_image_with_corners)

    # Detect checkerboard in LiDAR point cloud
    ret_lidar, corners_lidar = detect_checkerboard_in_point_cloud(lidar_data, (7, 7), 0.03)
    if not ret_lidar:
        logging.error('Checkerboard not detected in LiDAR point cloud')
        print('Checkerboard not detected in LiDAR point cloud')
        return

    # Perform calibration using detected corners
    rotation_matrix, translation_vector = estimate_transformation(corners_lidar, corners_camera)

    # Convert rotation matrix to Euler angles
    roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)

    # Extract translation components
    x, y, z = translation_vector.flatten()

    # Ensure values are floats
    x, y, z = float(x), float(y), float(z)
    roll, pitch, yaw = float(roll), float(pitch), float(yaw)

    # Display results
    print('X: {:.2f}, Y: {:.2f}, Z: {:.2f}, Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}'.format(x, y, z, roll, pitch, yaw))
    logging.info('X: {:.2f}, Y: {:.2f}, Z: {:.2f}, Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}'.format(x, y, z, roll, pitch, yaw))

    # Save the extrinsic parameters
    np.savez('extrinsic_parameters.npz', rotation_matrix=rotation_matrix, translation_vector=translation_vector)
    logging.info('Extrinsic parameters saved to extrinsic_parameters.npz')
    print('Extrinsic parameters saved to extrinsic_parameters.npz')

    # Output the transformation as an environment variable
    transform_str = '[{0},{1}]'.format(
        ",".join(map(str, rotation_matrix.flatten())),
        ",".join(map(str, translation_vector.flatten()))
    )
    with open('lidar_camera_transform.env', 'w') as f:
        f.write('LIDAR_CAMERA_TRANSFORM={}\n'.format(transform_str))
    logging.info('Created lidar_camera_transform.env')
    print('Created lidar_camera_transform.env')

    # Set environment variable
    os.environ['LIDAR_CAMERA_TRANSFORM'] = transform_str
    logging.info('Environment variable LIDAR_CAMERA_TRANSFORM set.')
    print('Environment variable LIDAR_CAMERA_TRANSFORM set.')

    # Generate the calibration file
    generate_calibration_file(rotation_matrix, translation_vector)
    
    rospy.signal_shutdown('Calibration complete: {}'.format(transform_str))

def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.degrees(x), np.degrees(y), np.degrees(z)

def detect_checkerboard(image, pattern_size=(7, 7)):
    # Adjust brightness and contrast
    alpha = 1.5  # Contrast control (1.0-3.0)
    beta = 0    # Brightness control (0-100)
    adjusted = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    cv2.imwrite('adjusted_image.png', adjusted)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(adjusted, (5, 5), 0)
    cv2.imwrite('blurred_image.png', blurred)

    # Detect checkerboard
    ret, corners = cv2.findChessboardCorners(blurred, pattern_size, None)
    if ret:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(blurred, corners, (11, 11), (-1, -1), criteria)
        logging.info('Checkerboard detected in camera image')
        print('Checkerboard detected in camera image')
    else:
        # Log image for debugging
        timestamp = rospy.Time.now().to_nsec()
        logging.debug('Checkerboard detection failed. Saving image for analysis.')
        cv2.imwrite('checkerboard_detection_failed_{}.png'.format(timestamp), image)
        
    return ret, corners

def detect_checkerboard_in_point_cloud(lidar_points, pattern_size=(7, 7), square_size=0.03):
    # Convert LiDAR points to 2D plane by projecting onto a plane
    xy_points = lidar_points[:, :2]

    # Create a grid of points corresponding to the checkerboard pattern
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Use nearest neighbors to find the corresponding points in the point cloud
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(xy_points)
    distances, indices = nbrs.kneighbors(objp[:, :2])

    if np.mean(distances) > 0.05:  # Adjusted this threshold to a larger value for robustness
        return False, None

    corners_lidar = lidar_points[indices.flatten()]
    return True, corners_lidar

def estimate_transformation(lidar_points, camera_points):
    logging.info('Estimating transformation between LiDAR and camera points.')
    print('Estimating transformation between LiDAR and camera points.')
    
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

    return rotation_matrix, translation_vector

def generate_calibration_file(rotation_matrix, translation_vector):
    # Intrinsic parameters from the example
    P2 = np.array([1042.6311, 0.0, 956.52848, 0.0,
                   0.0, 1041.80847, 632.79006, 0.0,
                   0.0, 0.0, 1.0, 0.0])
    
    R0_rect = np.eye(3).flatten()
    
    # Extrinsic parameters
    Tr_velo_to_cam = np.hstack((rotation_matrix, translation_vector.reshape(-1, 1))).flatten()

    # Write to calib.txt
    with open('calib.txt', 'w') as f:
        f.write('P2: {}\n'.format(" ".join(map(str, P2))))
        f.write('R0_rect: {}\n'.format(" ".join(map(str, R0_rect))))
        f.write('Tr_velo_to_cam: {}\n'.format(" ".join(map(str, Tr_velo_to_cam))))

    logging.info('Calibration parameters written to calib.txt')
    print('Calibration parameters written to calib.txt')

if __name__ == '__main__':
    try:
        logging.info('Starting LiDAR-Camera calibration script.')
        print('Starting LiDAR-Camera calibration script.')
        calibrate_lidar_camera()
    except rospy.ROSInterruptException:
        logging.error('ROS Interrupt Exception occurred.')
        print('ROS Interrupt Exception occurred.')
    except Exception as e:
        logging.error('An unexpected error occurred: {}'.format(e))
        print('An unexpected error occurred: {}'.format(e))

