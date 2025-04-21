#!/usr/bin/env python3

##################################################################################################
# Author: Soumyadip Banerjee                                                                     #
# Website: https://www.philosopherscode.de                                                       #
# github:  https://github.com/Nova200019                                                         #
# Disclaimer: This code is for educational purposes and provided "as-is" without any warranties. #
# Special Thanks:https://github.com/lavinama                                                     #
##################################################################################################


#Imports#
import logging
import os
import glob
import numpy as np
import cv2
import open3d as o3d
import matplotlib.pyplot as plt
from yolov4.tf import YOLOv4
import tensorflow as tf
import time
import json



# Set up logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Load the files
work_dir = os.getcwd()
image_files = sorted(glob.glob(os.path.join(work_dir, "data/img/*.png")))
point_files = sorted(glob.glob(os.path.join(work_dir, "data/velodyne/*.pcd")))
calib_files = sorted(glob.glob(os.path.join(work_dir, "data/calib/*.txt")))

# Initialize YOLOv4
yolo = YOLOv4(tiny=False)# Use YOLOv4-tiny for faster performance
yolo.classes = work_dir + "/yolov4/coco.names"
yolo.make_model()
yolo.load_weights(work_dir + "/yolov4/yolov4.weights", weights_type="yolo")

class LiDAR2Camera(object):
    def __init__(self, calib_file):
        """ Initialize the LiDAR to Camera projection using calibration data """
        calibs = self.read_calib_file(calib_file)
        self.calibs = calibs
        self.P = self.calibs["P2"].reshape(3, 4)
        self.V2C = self.calibs["Tr_velo_to_cam"].reshape(3, 4)
        self.R0 = self.calibs["R0_rect"].reshape(3, 3)
        logging.info(f"Calibration data read from {calib_file}.")

    def read_calib_file(self, filepath):
        """ Read the calibration file and return the data as a dictionary """
        data = {}
        with open(filepath, "r") as f:
            for line in f.readlines():
                line = line.rstrip()
                if len(line) == 0:
                    continue
                key, value = line.split(":", 1)
                data[key] = np.array([float(x) for x in value.split()])
        return data

    def cart2hom(self, pts_3d):
        """ Convert Cartesian coordinates to homogeneous coordinates """
        n = pts_3d.shape[0]
        pts_3d_hom = np.hstack((pts_3d, np.ones((n, 1))))
        logging.info("Converted Cartesian coordinates to homogeneous coordinates.")
        return pts_3d_hom

    def project_velo_to_image(self, pts_3d_velo):
        """ Project LiDAR points to the image plane """
        pts_3d_hom = self.cart2hom(pts_3d_velo)

        # Convert V2C to 4x4
        V2C_4x4 = np.vstack((self.V2C, np.array([0, 0, 0, 1])))

        # Convert R0 to 4x4
        R0_4x4 = np.eye(4)
        R0_4x4[:3, :3] = self.R0

        # Apply transformation: P * R0_rect * Tr_velo_to_cam * X
        pts_3d_cam = pts_3d_hom @ V2C_4x4.T
        pts_3d_cam = pts_3d_cam @ R0_4x4.T
        pts_2d_hom = pts_3d_cam @ self.P.T

        # Normalize homogeneous coordinates
        pts_2d = pts_2d_hom[:, :2] / pts_2d_hom[:, 2, np.newaxis]
        logging.info("Projected 3D points to image plane.")
        return pts_2d

    def get_lidar_in_image_fov(self, pc_velo, xmin, ymin, xmax, ymax, return_more=False, clip_distance=2.0):
        """ Filter LiDAR points to keep those within the image field of view """
        pts_2d = self.project_velo_to_image(pc_velo)
        a = [i for i, x in enumerate(pts_2d[:, 0] >= xmin) if x]
        b = [i for i, x in enumerate(pts_2d[:, 0] < xmax) if x]
        c = [i for i, y in enumerate(pts_2d[:, 1] >= ymin) if y]
        d = [i for i, y in enumerate(pts_2d[:, 1] < ymax) if y]
        e = [i for i, z in enumerate(pc_velo[:, 0] > clip_distance) if z]
        fov_inds = list(set(a) & set(b) & set(c) & set(d) & set(e))
        self.imgfov_pc_velo = pc_velo[fov_inds, :]
        if return_more:
            logging.info("Filtered LiDAR points to keep those within the image field of view.")
            return self.imgfov_pc_velo, pts_2d, fov_inds
        else:
            logging.info("Filtered LiDAR points to keep those within the image field of view.")
            return self.imgfov_pc_velo

    def show_lidar_on_image(self, pc_velo, img, pred_bboxes, debug=False):
        """ Project LiDAR points onto the image and filter points based on bounding boxes """
        imgfov_pc_velo, pts_2d, fov_inds = self.get_lidar_in_image_fov(
            pc_velo, 0, 0, img.shape[1], img.shape[0], True
        )
        if debug:
            logging.debug(f"3D PC Velo: {imgfov_pc_velo}")
            logging.debug(f"2D PIXEL: {pts_2d}")
            logging.debug(f"FOV: {fov_inds}")
        self.imgfov_pts_2d = pts_2d[fov_inds, :]
        cmap = plt.get_cmap("hsv", 256)
        cmap = np.array([cmap(i) for i in range(256)])[:, :3] * 255
        self.imgfov_pc_velo = imgfov_pc_velo

        for i in range(self.imgfov_pts_2d.shape[0]):
            x = round(self.imgfov_pts_2d[i][0])
            y = round(self.imgfov_pts_2d[i][1])
            depth = imgfov_pc_velo[i][0]
            r = 2
            color = tuple(map(int, cmap[int(510 / depth), :3]))
            belongs_to_box = False
            for box in pred_bboxes:
                if rectContains(box, self.imgfov_pts_2d[i], img.shape[1], img.shape[0], shrink_factor=0.2):
                    belongs_to_box = True
                    break
            if belongs_to_box:
                img = cv2.circle(img, (x, y), radius=r, color=color, thickness=-1)
        logging.info("Projected LiDAR points onto the image.")
        return img

    def calculate_size_and_position(self, pred_bboxes, image):
        """ Calculate the size and position of detected objects based on LiDAR points """
        sizes = []
        positions = []
        for box in pred_bboxes:
            points_in_box = []
            for i in range(self.imgfov_pts_2d.shape[0]):
                if rectContains(box, self.imgfov_pts_2d[i], image.shape[1], image.shape[0], shrink_factor=0.2):
                    points_in_box.append(self.imgfov_pc_velo[i])

            if points_in_box:
                points_in_box = np.array(points_in_box)
                min_point = np.min(points_in_box, axis=0)
                max_point = np.max(points_in_box, axis=0)
                size = max_point - min_point
                position = (min_point + max_point) / 2
                sizes.append(size)
                positions.append(position)
                json_message = {
                    "station_id": 1,
                    "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "class": yolo.classes[int(box[4])],
                    "size": {"height": size[2], "width": size[1], "depth": size[0]},
                    "position": {"x": position[0], "y": position[1], "z": position[2]}
                }
                print(json.dumps(json_message, indent=4))
        return sizes, positions

    def lidar_camera_fusion(self, pred_bboxes, image, dist_technique="average"):
        """ Fuse LiDAR points with detected obstacles in the image and calculate distances """
        img_bis = image.copy()
        cmap = plt.get_cmap("hsv", 256)
        cmap = np.array([cmap(i) for i in range(256)])[:, :3] * 255
        distances_obj = []

        for box in pred_bboxes:
            distances = []
            for i in range(self.imgfov_pts_2d.shape[0]):
                depth = self.imgfov_pc_velo[i][0]
                if rectContains(box, self.imgfov_pts_2d[i], image.shape[1], image.shape[0], shrink_factor=0.2):
                    distances.append(depth)
                    x = round(self.imgfov_pts_2d[i][0])
                    y = round(self.imgfov_pts_2d[i][1])
                    r = 2
                    color = tuple(map(int, cmap[int(510 / depth), :3]))
                    img_bis = cv2.circle(img_bis, (x, y), radius=r, color=color, thickness=-1)

            if distances:
                distances = filter_outliers(distances)
                dist = get_best_distance(distances, dist_technique)
                distances_obj.append(dist)
                cv2.putText(img_bis, '{:.2f} m'.format(dist), (int(box[0] * image.shape[1]), int(box[1] * image.shape[0]) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        sizes, positions = self.calculate_size_and_position(pred_bboxes, image)
        logging.info("Fused LiDAR points with detected obstacles in the image.")
        return img_bis, distances_obj, sizes, positions

def run_obstacle_detection(img):
    """ Run obstacle detection using YOLOv4 """
    logging.info("starting obstacle detection")
    start_time = time.time()
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    resized_image = yolo.resize_image(img)
    resized_image = resized_image / 255.0
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
    pred_bboxes = yolo.fit_pred_bboxes_to_original(pred_bboxes, img.shape)
    exec_time = time.time() - start_time
    logging.info("time: {:.2f} ms".format(exec_time * 1000))
    logging.info(pred_bboxes)
    logging.info("Obstacle detection completed.")
    return pred_bboxes

def rectContains(rect, pt, w, h, shrink_factor=0):
    """ Check if a point is within a given rectangle """
    max_x = int(rect[0] * w + rect[2] * (1 - shrink_factor) * (w / 2))
    min_x = int(rect[0] * w - rect[2] * (1 - shrink_factor) * (w / 2))
    max_y = int(rect[1] * h + rect[3] * (1 - shrink_factor) * (h / 2))
    min_y = int(rect[1] * h - rect[3] * (1 - shrink_factor) * (h / 2))
    return min_x <= pt[0] <= max_x and min_y <= pt[1] <= max_y

def filter_outliers(distances):
    """ Filter out outlier distances """
    mean = np.mean(distances)
    std = np.std(distances)
    return np.array(list({x for i, x in enumerate(distances) if x < mean + std} &
                         {x for i, x in enumerate(distances) if x > mean - std}))

def get_best_distance(distances, technique="closest"):
    """ Get the best distance from a list of distances using the specified technique """
    if technique == "closest":
        return np.amin(distances)
    if technique == "average":
        return np.mean(distances)
    if technique == "median":
        return np.median(distances)
    if technique == "farthest":
        return np.amax(distances)
    else:
        return np.median(distances)

def main():
    """ Main function to process images and point clouds """
    num_calib_files = len(calib_files)
    for index, (image_file, point_file) in enumerate(zip(image_files, point_files)):
        logging.info(f"Processing image and point cloud pair {index + 1}/{len(image_files)}.")
        calib_file = calib_files[index % num_calib_files]  # Use calibration files cyclically
        lidar2cam = LiDAR2Camera(calib_file)
        image = cv2.imread(image_file)
        cloud = o3d.io.read_point_cloud(point_file)
        points = np.asarray(cloud.points)
        pred_bboxes = run_obstacle_detection(image.copy())
        lidar_image = lidar2cam.show_lidar_on_image(points, image, pred_bboxes)
        fused_image, distances_obj, sizes, positions = lidar2cam.lidar_camera_fusion(pred_bboxes, lidar_image)
        fused_image = yolo.draw_bboxes(fused_image, pred_bboxes)  # Draw bounding boxes again
        cv2.imshow('Fusion Result', fused_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

