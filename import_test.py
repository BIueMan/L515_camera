'''
 there as a couple of laborers the code used,
 this scrip will make sure there are all installed
 make sure you using python 3.7, because "pyrealsense2" library don't work on newer version

 run this code, it will fall on library that was not imported successfully
'''

"""
# numpy, matplotlib and image prossing library. to install run the line in the terminal 
# make sure the terminal set to the directory of (/python37/Scripts). or where your "pip.exe" is install
# 
# pip install numpy
# pip install matplotlib
# pip install opencv-python
"""
import numpy as np
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import cv2                                # state of the art computer vision algorithms library

"""
# pyrealsense2 is the camera SDK.
# make sure you run python 3.7 or older, as for now this library don't supported the newer version 
# 
# pip install pyrealsense2
"""
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API

""" extra laborers
# pip install keyboard
"""
import keyboard

"""
# show the RGB_D image as pointcloud visualisation. an optional library
# pip install pyntcloud
"""
from pyntcloud import PyntCloud           # open source library for 3D pointcloud visualisation

print(" all the needed libraries were installed successfully")