"""
this code will use same of intel build in filter
see all filters at
https://github.com/IntelRealSense/librealsense/blob/jupyter/notebooks/depth_filters.ipynb
"""
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
print("Environment Ready")

""" load stairs depth image """
# Setup:
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("../stairs.bag")
profile = pipe.start(cfg)

# Skip 5 first frames to give the Auto-Exposure time to adjust
for x in range(5):
    pipe.wait_for_frames()

# Store next frameset for later processing:
frameset = pipe.wait_for_frames()
depth_frame = frameset.get_depth_frame()

# Cleanup:
pipe.stop()
print("Frames Captured")

"""*************"""
""" the filters """
"""*************"""
# note:
# depth_frame - is the original depth image (before converting to numpy matrix)
# colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
def NoFilter(depth_frame):
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

    plt.rcParams["axes.grid"] = False
    plt.rcParams['figure.figsize'] = [8, 4]
    plt.imshow(colorized_depth)
    plt.show()

def DecimationFilter(depth_frame):
    """ Decimation filter """
    """ 
    Decimation
    When using Depth-from-Stereo solution, z-accuracy is related to original spacial resolution.
    If you are satisfied with lower spatial resolution, the Decimation Filter will reduce spatial resolution preserving z-accuracy and performing some rudamentary hole-filling.
    """
    decimation = rs.decimation_filter()
    decimated_depth = decimation.process(depth_frame)
    colorized_depth = np.asanyarray(colorizer.colorize(decimated_depth).get_data())
    plt.imshow(colorized_depth)
    plt.show()
    """ 
    You can control the amount of decimation (liniar scale factor) via filter_magnitude option.
    """
    decimation.set_option(rs.option.filter_magnitude, 4)
    decimated_depth = decimation.process(depth_frame)
    colorized_depth = np.asanyarray(colorizer.colorize(decimated_depth).get_data())
    plt.imshow(colorized_depth)
    plt.show()

if __name__ == '__main__':
    colorizer = rs.colorizer()
    NoFilter(depth_frame)
    DecimationFilter(depth_frame)