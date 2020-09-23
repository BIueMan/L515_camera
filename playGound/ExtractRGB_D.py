""" We are planning to use the following packages,
    see that all the import load correctly"""
import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
print("Environment Ready")

"""we open a stream using a record file (.bag)"""
# Setup: open stream
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("../tiger_box.bag")
profile = pipe.start(cfg)

# Skip 5 first frames to give the Auto-Exposure time to adjust
for x in range(5):
    pipe.wait_for_frames()

# Store this frameset for later processing:
frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
depth_frame = frameset.get_depth_frame()

# Cleanup: close stream
pipe.stop()
print("Frames Captured")

""" plot the color image """
color = np.asanyarray(color_frame.get_data())
# color = color[:,:,0] # for one of the color in the image
plt.rcParams["axes.grid"] = False
plt.rcParams['figure.figsize'] = [12, 6]
plt.imshow(color)
plt.show()

""" now plot the depth frame """
colorizer = rs.colorizer()
colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
plt.imshow(colorized_depth)
plt.show()

""" Stream Alignment
Upon closer inspection you can notice that the two frames are not captured from the same physical viewport.
To combine them into a single RGBD image, let's align depth data to color viewport: """
# Create alignment primitive with color as its target stream:
align = rs.align(rs.stream.color)
frameset = align.process(frameset)

# Update color and depth frames:
aligned_depth_frame = frameset.get_depth_frame()
colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

# Show the two frames together:
images = np.hstack((color, colorized_depth))
plt.imshow(images)
plt.show()