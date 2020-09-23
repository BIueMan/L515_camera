## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
from pyntcloud import PyntCloud           # open source library for 3D pointcloud visualisation
print("Environment Ready")

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
print("start stream")

try:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    """ get 2d plot of color and depth """
    # Convert images to numpy arrays, same size
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    color_image = cv2.resize(color_image, (640, 480), cv2.INTER_CUBIC)
    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    # Stack both images horizontally
    images = np.hstack((color_image, depth_colormap))
    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    cv2.waitKey(1)

    """ get 3D pointcloud plot """
    # get raw vectors of points
    pc = rs.pointcloud()
    pc.map_to(color_frame)
    points = pc.calculate(depth_frame)
    vtx = np.asanyarray(points.get_vertices())
    tex = np.asanyarray(points.get_texture_coordinates())

    # plot using SDK system to plot, see link to how to use the plot() function
    # https://pyntcloud.readthedocs.io/en/latest/_modules/pyntcloud/core_class.html#PyntCloud.plot
    pc = rs.pointcloud()
    pc.map_to(color_frame)
    pointcloud = pc.calculate(depth_frame)
    pointcloud.export_to_ply("1.ply", color_frame)
    cloud = PyntCloud.from_file("1.ply")
    cloud.plot(beckend="threejs")


finally:
    # Stop streaming
    print("end stream")
    pipeline.stop()