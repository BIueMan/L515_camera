"""
this small library was build in order to help student interface with L515 depth camera, for deep learning purposes.
### it could: ###
1. take a frame and return the color and depth images as numpy matrices:
    # in addition: #
    * it can take a frame and save them as uncompress PNG image
    * and open the PNG images to numpy matrices
2. film a video with the camera, and save them either by using:
    * .avi file (will compress the images)
    * folder full with PNG images (uncompress)

see "L515_basic_interface()" class for more details:
"""

# TODO: make a readme file for github, create dir if dir is not there

import pyrealsense2 as rs
import numpy as np
import cv2
import keyboard
import os
import datetime

# default parameters, it set to be the max size of an image
DEPTH_SIZE_DEFAULT = (640, 480)
COLOR_SIZE_DEFAULT = (1280, 720)
LASER_POWER_DEFAULT = 80.0
RECEIVER_GAIN_DEFAULT = 9.0
FPS = 30

'''
this class use to convert numpy images into PNG file and vice versa.
make use:
    * name is a legal name for a file
    * image is a [x,y,3]-RGB shape matrix
    * dir is the full location of where you want to work on
        - use "self.main_directory" as root, and add path to the working location
'''
class PNG_converter():
    def __init__(self):
        self.main_directory = os.getcwd()
        self.image_dir = '\\output\\images'
        self.video_dir = '\\output\\videos'
        self.default_dir = '\\output\\default'

    ''' function to save an image, will not compress the data
            * image - numpy matrix shape as RGB image
            * name - legal name for the PNG file (no need to add .png at the end)
            * dir - where you want to save the image     
     '''
    def imsave(self, image, name, dir = None):
        if dir is None:
            path_dir = self.default_dir
        else:
            path_dir = dir
        path = self.main_directory + path_dir
        os.chdir(path) # change dir to where we want to save the image

        if not cv2.imwrite(name + ".png", image, [cv2.IMWRITE_PNG_COMPRESSION, 0]):
            print("fail to save image")
            # fail here probably caused do to using illegal characters for the file name
            # or image matrix is illegal size, not (x,y,3)/RGB size

        os.chdir(self.main_directory) # change beck to main dir

    ''' function to load PNG image as numpy matrix
            * path - where the image is saved
            * name - full name including .png at the end  
    '''
    def imload(self, path, name):
        os.chdir(path)  # change dir to where we want to save the image
        image = cv2.imread(name)
        os.chdir(self.main_directory)  # change beck to root dir
        return image


''' this is the interface class with the camera
    * class should be called without any parameters. use the parameters only if you want to change the default once
    * the class give you control over the laser_power and receiver_gain, there are the must important changes you could
      do to the camera inorder to work with it from close of from afar. (read more at the README file)
      ### initialize: ###
        1. "startStream()" should be called first to connect to the camera
        2. you can use all the function to take images and videos
        3. "stopStream()" should be call after you finish working with the camera, to close the interface with it
    
'''
class L515_basic_interface():
    def __init__(self, image_size_depth = DEPTH_SIZE_DEFAULT, image_size_color = COLOR_SIZE_DEFAULT,
                 laser_power = LASER_POWER_DEFAULT, receiver_gain = RECEIVER_GAIN_DEFAULT):
        self.image_size_d = image_size_depth
        self.image_size_c = image_size_color
        self.laser_power = laser_power
        self.receiver_gain = receiver_gain
        self.depth_sensor = None

        # config object
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.image_size_d[0], self.image_size_d[1], rs.format.z16, FPS)
        self.config.enable_stream(rs.stream.color, self.image_size_c[0], self.image_size_c[1], rs.format.bgr8, FPS)

        self.pipeline = rs.pipeline()

        # for video record
        self.main_directory = os.getcwd()
        self.video_dir = '\\output\\videos'

    ''' use to connect to the camera
        will return "True" if he succeeded, else "False"
    '''
    def startStream(self):
        try:
            pipeline_profile = self.pipeline.start(self.config)
            print("start stream")
            # get device
            device = pipeline_profile.get_device()
            # get depth sensor
            self.depth_sensor = device.query_sensors()[0]
            # set laser and receiver params
            self.depth_sensor.set_option(rs.option.laser_power, self.laser_power)
            self.depth_sensor.set_option(rs.option.receiver_gain, self.receiver_gain)
            success = True
        except:
            print("fail to start the stream")
            self.pipeline.stop()
            success = False
        return success

    ''' close stream, use it when you finish with the camera '''
    def stopStream(self):
        self.pipeline.stop()
        print("close stream")

    ''' this function will take a single frame of both color and depth.
            * you can input a frame of your own from the camera, by default it will take a frame on it own  
        it will return them as numpy matrices
            * color_image - is an 8bit-RGB matrix
            * depth_image - is an 16bit-grayscale matrix
            # if fail, function will return a signal "None"
    '''
    def takePicture(self, frame = None):
        # if there is no input frame
        if not frame:
            try:
                frames = self.pipeline.wait_for_frames()
            except:
                return None # over time, fail to capture a frame
        else:
            frames = frame

        # get color and depth
        try:
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame: # if frame is empty
                return None
        except:
            return None # fail to load from the frame

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return color_image, depth_image

    ''' this function will save a single frame of both color and depth as PNG file.
            * you can input a color/depth numpy matrices of your own, by default it will capture a frame on it own  
        PNG file are 8bit-RBG images, and as you know:
            * color_image - is an 8bit-RGB matrix
            * depth_image - is an 16bit-grayscale matrix
            so we have a problem saving the depth images. in order to counter it we will split the data to RG when
            we save the depth_image. just remember to load and merge the data afterwards. see help fanction at the end
        all data will be save at the output folder, under images
    '''
    def savePicture(self, color_np=None, depth_np=None):
        if color_np is None or depth_np is None:
            color, depth = self.takePicture()
        else:
            color, depth = color_np, depth_np

        # split 16 bit grayscale into two 8 bit channel in 8bit image
        depth_image = bit_16_into_two_8(depth)

        # get names for files
        time = get_file_time()
        color_file_name = "".join(("color_", time))
        depth_file_name = "".join(("depth_", time))

        # save images
        converter = PNG_converter()
        converter.imsave(color, color_file_name, "".join((converter.image_dir, "\\color")))
        converter.imsave(depth_image, depth_file_name, "".join((converter.image_dir, "\\depth")))

    ''' this function will let you record a .avi file
        press 'R' in order to start and stop recording.
        press 'Esc' to exit the function.
        all data will be save at the output folder, under videos
    '''
    def liveVideo(self):
        print("- press Esc to end Stream -")
        print("- press R to start and stop the record -")
        is_recording = False
        color_out = None
        depth_out = None
        path = "".join((self.main_directory, self.video_dir))
        os.chdir(path)  # change dir to where we want to save the image
        try:
            keyboard_pressed_pass = {'r':False}
            while True:
                frame = self.pipeline.wait_for_frames()
                color_image, depth_image = self.takePicture(frame)

                ''' show live video '''
                # resize image, so images will be in same size
                color_image_resize = cv2.resize(color_image, (640, 480), cv2.INTER_CUBIC)
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                # Stack both images horizontally
                images = np.hstack((color_image_resize, depth_colormap))
                # Show images
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                cv2.waitKey(1)

                ''' record '''
                if is_recording and not (color_out is None or depth_out is None):
                    depth = bit_16_into_two_8(depth_image) # split data to RG channels
                    color_out.write(color_image)
                    """ change to what depth video you want to save"""
                    # depth_out.write(depth) # save a splited data
                    depth_out.write(depth_colormap) # save a colormap of the depth

                if keyboard.is_pressed('Esc'):
                    # print(" Esc as press")
                    break

                ''' user control'''
                r_is_pressed = keyboard.is_pressed('r')
                if r_is_pressed and not keyboard_pressed_pass['r']:
                    if not is_recording:
                        print(" start recording ")
                        is_recording = True
                        # create video files
                        time = get_file_time()
                        color_name = "color_" + time + ".avi"
                        color_out = cv2.VideoWriter(color_name, cv2.VideoWriter_fourcc(*'DIVX'), FPS, tuple(COLOR_SIZE_DEFAULT))
                        depth_name = "depth_" + time + ".avi"
                        depth_out = cv2.VideoWriter(depth_name, cv2.VideoWriter_fourcc(*'DIVX'), FPS, tuple(DEPTH_SIZE_DEFAULT))
                    else:
                        print(" stop recording ")
                        color_out.release()
                        depth_out.release()
                        print(" videos were saved ")
                        is_recording = False

                keyboard_pressed_pass['r'] = r_is_pressed

        except:
            print("fail to load frame or use frame")
        finally:
            if is_recording:
                print(" stop recording ")
                color_out.release()
                depth_out.release()
                print(" videos were saved ")
            os.chdir(self.main_directory)  # change beck to main dir

    ''' this function will let you record a video, and will save if as pictures.
            press 'R' in order to start and stop recording.
            press 'P' in order to save a single frame.
            press 'Esc' to exit the function.
            all data will be save at the output folder, under videos in a file with date
    '''
    def livePicture(self):
        print("- press Esc to end Stream -")
        print("- press R to start and stop the record -")
        print("- press P to take a single picture -")
        is_recording = False
        color_out = None
        depth_out = None
        prime_counter = 0
        try:
            keyboard_pressed_pass = {'r':False,
                                     'p':False}
            while True:
                frame = self.pipeline.wait_for_frames()
                color_image, depth_image = self.takePicture(frame)

                ''' show live video '''
                # resize image, so images will be in same size
                color_image_resize = cv2.resize(color_image, (640, 480), cv2.INTER_CUBIC)
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                # Stack both images horizontally
                images = np.hstack((color_image_resize, depth_colormap))
                # Show images
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                cv2.waitKey(1)

                ''' record '''
                if is_recording and not (color_out is None or depth_out is None):
                    depth = bit_16_into_two_8(depth_image) # split data to RG channels
                    # get names for files
                    name = str(prime_counter)
                    # save images
                    converter = PNG_converter()
                    converter.imsave(color_image, name, converter.video_dir + color_out)
                    converter.imsave(depth, name, converter.video_dir + depth_out)

                    prime_counter += 1

                if keyboard.is_pressed('Esc'):
                    # print(" Esc as press")
                    break

                ''' user control'''
                r_is_pressed = keyboard.is_pressed('r')
                if r_is_pressed and not keyboard_pressed_pass['r']:
                    if not is_recording:
                        print(" start recording ")
                        is_recording = True
                        # create dir for pictures
                        time = get_file_time()
                        color_out = "//color//" + time
                        os.mkdir(self.main_directory + self.video_dir + color_out)
                        depth_out = "//depth//" + time
                        os.mkdir(self.main_directory + self.video_dir + depth_out)
                    else:
                        print(" stop recording ")
                        color_out, depth_out = None, None
                        prime_counter = 0
                        is_recording = False
                keyboard_pressed_pass['r'] = r_is_pressed

                p_is_pressed = keyboard.is_pressed('p')
                if p_is_pressed and not keyboard_pressed_pass['p']:
                    # get names for files
                    time = get_file_time()
                    color_file_name = "".join(("color_", time))
                    depth_file_name = "".join(("depth_", time))
                    # split depth data
                    depth = bit_16_into_two_8(depth_image)  # split data to RG channels
                    # save images
                    converter = PNG_converter()
                    converter.imsave(color_image, color_file_name, "".join((converter.image_dir, "\\color")))
                    converter.imsave(depth, depth_file_name, "".join((converter.image_dir, "\\depth")))
                    print("picture was taken")
                keyboard_pressed_pass['p'] = p_is_pressed
        except:
            print("fail to load frame or use frame")
        finally:
            if is_recording:
                print(" stop recording ")

    ''' you can use this function inorder to set the "laser_power" and "receiver_gain" params
        make sure all the params are flute type. and in the range of the camera
    '''
    def changeParam(self, set_laser, set_receiver):
        try:
            depth_sensor = self.depth_sensor
            print("laser power was = ", depth_sensor.get_option(rs.option.laser_power))
            # see the limit for the laser
            laser_range = depth_sensor.get_option_range(rs.option.laser_power)
            print("laser power range = ", laser_range.min, "~", laser_range.max)
            # change the laser param
            depth_sensor.set_option(rs.option.laser_power, set_laser)
            print("laser power change to = ", depth_sensor.get_option(rs.option.laser_power))
            print('')
            print("receiver gain was = ", depth_sensor.get_option(rs.option.receiver_gain))
            receiver_range = depth_sensor.get_option_range(rs.option.receiver_gain)
            print("receiver gain range = ", receiver_range.min, "~", receiver_range.max)
            depth_sensor.set_option(rs.option.receiver_gain, set_receiver)
            print("receiver gain change to = ", depth_sensor.get_option(rs.option.receiver_gain))
        except:
            print("--- invalid params ---")

""" from here we have same help fanciton, the code is useing """

# return a date as a legal name for file name
def get_file_time():
    date = datetime.datetime.now()
    time = date.strftime("%x_%X")
    time = time.replace(':', ';')
    time = time.replace('/', ',')
    return time

# input, a 16-bit-grayscale image
# output, 8bit-RGB-image at the size of the matrix
#   the 16 bits was split into the first 2 color channels of the image
#   the R channel is the high 8-bit value of the 16-bit, and G channel are the lower once
def bit_16_into_two_8(matrix_16):
    high = (matrix_16 >> 8) & 0xff
    high = np.array(high, dtype=np.uint8)

    low = matrix_16 & 0xff
    low = np.array(low, dtype=np.uint8)

    zeros = np.zeros(matrix_16.shape, dtype=np.uint8)

    return np.dstack((high, low, zeros))

# marge a 8bit-RGB-image into 16 grayscale image, using the R channel as the high value, and G as the low
def bit_two_8_into_16(image):
    high = image[:,:,0]
    high = np.array(high, dtype=np.int16)
    high = (high << 8)

    low = image[:,:,1]
    low = np.array(low, dtype=np.int16)

    return high + low

