# L515_camera
In sort, this code is a interface with the L515 Camera, meant to be use by students.
The interface give you a basic control over the setings that L515 could be config.
After configuration, you could take picters and videos with the camera. and save them as either PNG images or video format, or keep them as numpy matrices.
if you are a student, feel free to use my code :)

# HOW TO START:
1) make sure the comera conected to USB3 port
2) at first, try to use the "Intel.RealSense.Viewer16" file to see what the comera can do. (downloaded from the SDK link)
3) next in python, see the code for "Installation" of the camera driver and the libraries the code used. the camera driver only sopurt python 3.7 or lower!
4) when you call the interface, you first start the steam that conect to the camera, then you can use it to take picters and videro.
   code code to start will be:
        camera = L515_basic_interface()  # call the interface
        camera.startStream()
        camera.liveVideo()
        camera.stopStream()
5) if you want to know how the SDK of the camera work, there are same sample code from intel, and same i write. you may skip this part if you only interested is the interface:
   SAMPLE code:
    * try to learn from the playGound code in the order I write them
      1) GetRawDepth - extract the depth image from a frame
      2) ExtractRGB_D - extract RGB and depth images from the frame
      3) LiveVideo - live video useing the comera. all images convert to NP matrix for easy use
      4) Filters - base SDK filters, see more at https://github.com/IntelRealSense/librealsense/blob/jupyter/notebooks/depth_filters.ipynb
      5) LiveAlign - Align the RGB and depth images useing SDK build in fanction
      6) GetSetOption - the link of "optimizing use" tell as that by changing (laser power) and (receiver gain),
         we can change the range of the camera depth to work with near or far objects.
6) for more see the function that was deffinde in the interface

# NOTES:
  how the camera works:
    * the camera shots frames all the time. when you ask for one it will give you the last frame that it took.
    * from a fream you could extract matrices of: color-8bit image, grayscle-16bit image. one for the RGB camera, and the second for the depth sensor.
    * the code works with these matrices
  saved images:
    * PNG is a 3 channel 8-bit format and could easly save a color image. but the grayscle image a 1 channel 16-bit, and cant be save on one channel of a PNG format.
      so when the depth image is saved, it split the data betwen the first 2 channels. and when load a grascle image, it will morge the data beck to 16-bit.
      so dont get confused if a depth PNG image dont make sense.
  video:
    * as for now, i did't found a video format that does't compres the data. so there are 2 ways to shot a video
    1) normal with format with compression.
    2) it will shot and save a group of pictures, every image come from the next frame.
    
    
# EXTRA NOTES:
* pyrealsense2.config() can set variables for the camera before it start to stream. 
	- depth and color image can be set to the size that "Intel.RealSense.Viewer16" can be set them.
    	  also, they dont have resolution at the same size, so use color_image = cv2.resize(color_image, (640, 480), cv2.INTER_CUBIC) to reshape the image after convort to numpy matrix. see "LiveVideo.py" for used
	  or use "align" to align the image for prosess. see "LiveAlign" for used

* rs.align(align_to) - change the color and depth of a frame to to the same size, so they could be align.
* SDK-filter - there is a filter labrery in the camera SDK, see "Filter.py" for same used.
	- also visit the link to see all the SDK filter: https://github.com/IntelRealSense/librealsense/blob/jupyter/notebooks/depth_filters.ipynb
* for easy calibration of the camera, there is a default min,max option (option.max_distance, option.min_distance), see SAMPLE code 6 to implement options.


# LINKS:
SDK - https://github.com/IntelRealSense/librealsense/releases/tag/v2.36.0
python - https://github.com/IntelRealSense/librealsense/tree/development/wrappers/python
optimizing use - https://www.intelrealsense.com/optimizing-the-lidar-camera-l515-range/

documentation:
the official document of intel is from the semple code, but there is non-official document.
private_non_official_document - https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.html
python_semple_code - https://github.com/IntelRealSense/librealsense/tree/development/wrappers/python/examples
