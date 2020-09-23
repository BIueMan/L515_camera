# First import the library
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt

##########################
# code 1:
# this code will get an image and extract the raw data from the depth sensor
# and imshow it
##########################

X = 640
Y = 480

""" start the comera stream"""
# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
pipeline.start()

try:
    # Create a pipeline object. This object configures the streaming camera and owns it's handle
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()

    # get image dist
    if depth:
        dist = np.zeros((X, Y))
        for x in range(0, X):
            for y in range(0, Y):
                dist[x][y] = depth.get_distance(x, y)


finally:
    pipeline.stop()

    # print matrix
    dist = dist.T
    plt.imshow(dist)
    plt.show()

##############################
# code 2:
# do the same. but using depth.as_frame().get_data() to get the data
#
##############################
# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
pipeline.start()

try:
    # Create a pipeline object. This object configures the streaming camera and owns it's handle
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()

    depth_data = depth.as_frame().get_data()
    np_image = np.asanyarray(depth_data)

finally:
    pipeline.stop()

    plt.imshow(np_image)
    plt.show()