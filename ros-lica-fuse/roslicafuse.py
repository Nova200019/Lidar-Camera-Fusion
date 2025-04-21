#!/usr/bin/env python3

##################################################################################################
# Author: Soumyadip Banerjee                                                                     #
# Website: https://www.philosopherscode.de                                                       #
# GitHub:  https://github.com/Nova200019                                                         #
# Disclaimer: This code is for educational purposes and provided "as-is" without any warranties. #
# Special Thanks: https://github.com/lavinama                                                    #
##################################################################################################

# Imports
import os
import logging
from pathlib import Path

import rospy
import sys
import json
from datetime import datetime
import cv2
import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import MarkerArray
import sensor_msgs.point_cloud2 as pc2
import threading
import tensorflow as tf
from yolov4.tf import YOLOv4
from queue import Queue
from mqtt.listener import DirectoryWatcher
from mqtt.mqtt_client_pub import MQTTClient
from mqtt.settings import CONFIG_INI, path_to_watch

# Set the environment variable to preload the libgomp library
os.environ['LD_PRELOAD'] = '/usr/lib/aarch64-linux-gnu/libgomp.so.1'

# Verify if the path is added correctly
print("Python path:", sys.path)

try:
    import open3d as o3d
except ImportError as e:
    print("Failed to import Open3D: ", e)
    sys.exit(1)

# Logging configuration
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logging.info('Script started')

# Ensure ROS Master URI is set
if 'ROS_MASTER_URI' not in os.environ:
    os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'
logging.info('ROS environment sourced')

# Global variables to store received data
lidar_data_queue = Queue(maxsize=10)
camera_image_queue = Queue(maxsize=10)
detected_objects = []

# Initialize ROS node
rospy.init_node('object_detection_and_tracking', anonymous=True)


# YOLO setup using YOLOv4
def load_yolo():
    logging.info("Loading YOLO model")
    yolo = YOLOv4(tiny=True)  # Use YOLOv4-tiny for faster performance
    yolo.classes = os.getcwd() + "/yolov4/coco.names"  # Path to class names
    yolo.make_model()
    yolo.load_weights(os.getcwd() + "./yolov4/yolov4-tiny.weights", weights_type="yolo")  # Path to weights
    logging.info("YOLO model loaded successfully")
    return yolo


yolo = load_yolo()

# TensorFlow GPU settings
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
        logical_gpus = tf.config.experimental.list_logical_devices('GPU')
        logging.info(f'{len(gpus)} Physical GPUs, {len(logical_gpus)} Logical GPUs')
    except RuntimeError as e:
        logging.error(e)

# Publishers for RViz
pcd_pub = rospy.Publisher('/fused_pcd', PointCloud2, queue_size=10)
image_pub = rospy.Publisher('/fused_image', Image, queue_size=10)
marker_pub = rospy.Publisher('/detection_markers', MarkerArray, queue_size=10)


# Convert ROS Image message to OpenCV format
def imgmsg_to_cv2(img_msg):
    if img_msg.encoding == "yuyv":
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv_yuyv = np.ndarray(shape=(img_msg.height, img_msg.width, 2), dtype=dtype, buffer=img_msg.data)
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv_yuyv = image_opencv_yuyv.byteswap().newbyteorder()
        image_opencv_bgr = cv2.cvtColor(image_opencv_yuyv, cv2.COLOR_YUV2BGR_YUYV)
        return image_opencv_bgr
    elif img_msg.encoding == "bgr8":
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv
    elif img_msg.encoding == "rgb8":
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
        return image_opencv
    elif img_msg.encoding == "mono8":
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width), dtype=dtype, buffer=img_msg.data)
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
    else:
        rospy.logerr(f"Unsupported encoding: {img_msg.encoding}")
        return None


# Convert OpenCV image to ROS Image message
def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg


class LiDAR2Camera:
    def __init__(self, calib_file):
        """Initialize the LiDAR to Camera projection using calibration data"""
        calibs = self.read_calib_file(calib_file)
        self.calibs = calibs
        self.P = self.calibs["P2"].reshape(3, 4)
        self.V2C = self.calibs["Tr_velo_to_cam"].reshape(3, 4)
        self.R0 = self.calibs["R0_rect"].reshape(3, 3)

    def read_calib_file(self, filepath):
        """Read the calibration file and return a dictionary of calibration matrices"""
        data = {}
        with open(filepath, "r") as f:
            for line in f.readlines():
                line = line.rstrip()
                if len(line) == 0:
                    continue
                key, value = line.split(":", 1)
                try:
                    data[key] = np.array([float(x) for x in value.split()])
                except ValueError:
                    pass
        return data

    def cart2hom(self, pts_3d):
        """Convert Cartesian coordinates to homogeneous coordinates"""
        n = pts_3d.shape[0]
        pts_3d_hom = np.hstack((pts_3d, np.ones((n, 1))))
        return pts_3d_hom

    def project_velo_to_ref(self, pts_3d_velo):
        """Project LiDAR points to reference frame using calibration matrices"""
        pts_3d_velo = self.cart2hom(pts_3d_velo)  # nx4
        return np.dot(pts_3d_velo, np.transpose(self.V2C))

    def project_velo_to_image(self, pts_3d_velo):
        """Project LiDAR points to the image plane using calibration matrices"""
        pts_3d_hom = self.cart2hom(pts_3d_velo)
        pt_3d = pts_3d_hom[0]

        m1 = self.V2C.shape[1]
        extra_matrix = np.zeros((1, m1))
        extra_matrix[:, -1] = 1
        R_t_homo = np.vstack((self.V2C, extra_matrix))

        R_t_X = np.matmul(R_t_homo, pts_3d_hom.transpose())

        m1 = self.R0.shape[1]
        extra_matrix = np.zeros((1, m1))
        R0_hom = np.vstack((self.R0, extra_matrix))
        m2 = m1 + 1
        extra_matrix = np.zeros((m2, 1))
        extra_matrix[-1, :] = 1
        R0_hom = np.hstack((R0_hom, extra_matrix))

        R0_R_t_X = np.matmul(R0_hom, R_t_X)
        homo_result = np.matmul(self.P, R0_R_t_X)

        pts_2d = homo_result.transpose()
        pts_2d[:, 0] /= pts_2d[:, 2]
        pts_2d[:, 1] /= pts_2d[:, 2]
        return pts_2d[:, :-1]

    def get_lidar_in_image_fov(self, pc_velo, xmin, ymin, xmax, ymax, return_more=False, clip_distance=2.0):
        """Filter LiDAR points to keep those within the image field of view"""
        pts_2d = self.project_velo_to_image(pc_velo)
        a = [i for i, x in enumerate(pts_2d[:, 0] >= xmin) if x]
        b = [i for i, x in enumerate(pts_2d[:, 0] < xmax) if x]
        c = [i for i, y in enumerate(pts_2d[:, 1] >= ymin) if y]
        d = [i for i, y in enumerate(pts_2d[:, 1] < ymax) if y]
        e = [i for i, z in enumerate(pc_velo[:, 0] > clip_distance) if z]
        fov_inds = list(set(a) & set(b) & set(c) & set(d) & set(e))

        self.imgfov_pc_velo = pc_velo[fov_inds, :]
        if return_more:
            return self.imgfov_pc_velo, pts_2d, fov_inds
        else:
            return self.imgfov_pc_velo

    def show_lidar_on_image(self, pc_velo, img, debug=False):
        """Overlay LiDAR points on the image"""
        imgfov_pc_velo, pts_2d, fov_inds = self.get_lidar_in_image_fov(pc_velo, 0, 0, img.shape[1], img.shape[0], True)

        logging.info("3D PC Velo " + str(imgfov_pc_velo))
        logging.info("2D PIXEL: " + str(pts_2d))
        logging.info("FOV : " + str(fov_inds))
        self.imgfov_pts_2d = pts_2d[fov_inds, :]

        cmap = plt.cm.get_cmap("hsv", 256)
        cmap = np.array([cmap(i) for i in range(256)])[:, :3] * 255
        self.imgfov_pc_velo = imgfov_pc_velo

        img = np.array(img, copy=True)

        for i in range(self.imgfov_pts_2d.shape[0]):
            x = round(self.imgfov_pts_2d[i][0])
            y = round(self.imgfov_pts_2d[i][1])

            depth = imgfov_pc_velo[i][0]
            r = 2
            colour = cmap[int(510 / depth), :]
            thickness = -1
            img = cv2.circle(img, (x, y), radius=r, color=tuple(colour), thickness=-1)

        return img


def run_obstacle_detection(image):
    """Run YOLOv4 for obstacle detection on the image"""
    height, width = image.shape[:2]
    resized_image = yolo.resize_image(image)
    resized_image = resized_image / 255.
    input_data = resized_image[np.newaxis, ...].astype(np.float32)

    candidates = yolo.model.predict(input_data)
    _candidates = []
    for candidate in candidates:
        batch_size = candidate.shape[0]
        grid_size = candidate.shape[1]
        _candidates.append(tf.reshape(candidate, shape=(1, grid_size * grid_size * 3, -1)))

    candidates = np.concatenate(_candidates, axis=1)
    pred_bboxes = yolo.candidates_to_pred_bboxes(candidates[0], iou_threshold=0.35, score_threshold=0.40)
    pred_bboxes = pred_bboxes[~(pred_bboxes == 0).all(1)]
    pred_bboxes = yolo.fit_pred_bboxes_to_original(pred_bboxes, image.shape)

    result = yolo.draw_bboxes(image, pred_bboxes)
    result = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)

    detected_objects = []
    for bbox in pred_bboxes:
        detected_objects.append({
            "class": yolo.classes[int(bbox[5])],
            "size": [bbox[2], bbox[3], 1],
            "position": [bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2, 0]
        })

    return result, detected_objects


def publish_detection_results(detected_objects):
    """Publish detected objects to ROS and print them as JSON"""
    if len(detected_objects) == 0:
        return
    Path.mkdir(path_to_watch, exist_ok=True)
    for obj in detected_objects:
        detection_result = {
            "station_id": 1,
            "ts": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "class": obj['class'],
            "size": {
                "height": obj['size'][2],
                "width": obj['size'][0],
                "depth": obj['size'][1]
            },
            "position": {
                "x": obj['position'][0],
                "y": obj['position'][1],
                "z": obj['position'][2]
            }
        }
        print(json.dumps(detection_result))
        # dump json file into watched directory
        result_file = Path(path_to_watch).joinpath(f"{detection_result.get('ts')}.json")
        with open(result_file, 'w') as f:
            json.dump(detection_result, f)


def lidar_callback(data):
    """Callback function to process received LiDAR data"""
    try:
        lidar_data = np.array(list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)))
        if lidar_data_queue.full():
            lidar_data_queue.get()
        lidar_data_queue.put(lidar_data)
    except Exception as e:
        rospy.logerr(f"Failed to process LiDAR data: {e}")


def camera_callback(data):
    """Callback function to process received camera images"""
    try:
        camera_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        if camera_image_queue.full():
            camera_image_queue.get()
        camera_image_queue.put(camera_image)
    except Exception as e:
        rospy.logerr(f"Failed to process camera image: {e}")


def process_data():
    """Process LiDAR and camera data for object detection and tracking"""
    while not rospy.is_shutdown():
        if not lidar_data_queue.empty() and not camera_image_queue.empty():
            try:
                lidar_data = lidar_data_queue.get()
                camera_image = camera_image_queue.get()

                # Resize camera image to reduce processing time
                camera_image = cv2.resize(camera_image, (640, 360))

                # Project LiDAR points onto the camera image
                image_with_lidar = lidar2cam.show_lidar_on_image(lidar_data, camera_image)

                # Perform YOLO object detection
                result_image, detected_objects = run_obstacle_detection(image_with_lidar)

                # Publish detection results
                publish_detection_results(detected_objects)

                # Publish the fused image to ROS
                image_msg = cv2_to_imgmsg(result_image)
                image_msg.header.stamp = rospy.Time.now()
                image_pub.publish(image_msg)
            except Exception as e:
                rospy.logerr(f"Error in processing data: {e}")


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('object_detection_and_tracking', anonymous=True)

    # Create an instance of LiDAR2Camera
    lidar2cam = LiDAR2Camera("/home/sb/lica/calib.txt")

    # initialize mqtt client
    mqtt_client = MQTTClient(CONFIG_INI)
    mqtt_client.connect()
    watcher = DirectoryWatcher(path_to_watch, mqtt_client)
    watcher.run()

    # Subscribe to LiDAR and camera topics
    rospy.Subscriber("/livox/lidar", PointCloud2, lidar_callback)
    rospy.Subscriber("/usb_cam/image_raw", Image, camera_callback)

    # Start the processing thread
    processing_thread = threading.Thread(target=process_data)
    processing_thread.start()

    # Spin ROS node
    rospy.spin()
