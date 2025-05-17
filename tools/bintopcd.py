##################################################################################################
# Author: Soumyadip Banerjee                                                                     #
# Website: https://www.philosopherscode.de                                                       #
# github:  https://github.com/Nova200019                                                         #
# Disclaimer: This code is for educational purposes and provided "as-is" without any warranties. #
##################################################################################################
import open3d as o3d
import numpy as np
import struct
import glob
size_float = 4

file_to_open = sorted(glob.glob("C:\\Users\\soumy\\PycharmProjects\\pythonProject1\\data\\velodyne\\*.bin"))##Configure your file path

for fi in file_to_open:
	list_pcd = []
	with open (fi, "rb") as f:
	    byte = f.read(size_float*4)
	    while byte:
	        x,y,z,intensity = struct.unpack("ffff", byte)
	        list_pcd.append([x, y, z])
	        byte = f.read(size_float*4)
	np_pcd = np.asarray(list_pcd)
	pcd = o3d.geometry.PointCloud()
	v3d = o3d.utility.Vector3dVector
	pcd.points = v3d(np_pcd)
	o3d.io.write_point_cloud(fi[:-3]+"pcd", pcd)