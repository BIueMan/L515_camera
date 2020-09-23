################################################################
##          see lines 18-39, 72-89 for change params          ##
##  use help(rs.option) to see list of parames to can change  ##
################################################################

import pyrealsense2 as rs
import numpy as np
import cv2
import keyboard

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)


# Start streaming
pipeline_profile = pipeline.start(config)
print("start stream")
# get device to change param
device = pipeline_profile.get_device()
# get depth sensor
depth_sensor = device.query_sensors()[0]
print("laser power is = ", depth_sensor.get_option(rs.option.laser_power))
# see the limit for the laser
laser_range = depth_sensor.get_option_range(rs.option.laser_power)
print("laser power range = " , laser_range.min , "~", laser_range.max)
# change the laser param
depth_sensor.set_option(rs.option.laser_power, 0)
print("laser power change to = ", depth_sensor.get_option(rs.option.laser_power))
print('')

""" same with receiver gain """
print("receiver gain is = ", depth_sensor.get_option(rs.option.receiver_gain))
receiver_range = depth_sensor.get_option_range(rs.option.receiver_gain)
print("receiver gain range = " , receiver_range.min , "~", receiver_range.max)
depth_sensor.set_option(rs.option.receiver_gain, 16)
print("receiver gain change to = ", depth_sensor.get_option(rs.option.receiver_gain))

print('')
try:
    print("- press Esc to end Stream -")
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
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

        if keyboard.is_pressed('Esc'):
            # print(" Esc as press")
            break
        if keyboard.is_pressed('0'):
            depth_sensor.set_option(rs.option.laser_power, 0)
        elif keyboard.is_pressed('9'):
            depth_sensor.set_option(rs.option.laser_power, 100)
        if keyboard.is_pressed('7'):
            depth_sensor.set_option(rs.option.receiver_gain, 8)
        elif keyboard.is_pressed('8'):
            depth_sensor.set_option(rs.option.receiver_gain, 16)

finally:
    # Stop streaming
    print("end stream")
    pipeline.stop()