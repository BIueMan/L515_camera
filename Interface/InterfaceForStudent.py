import pyrealsense2 as rs
import numpy as np
import cv2
import keyboard
import os
import datetime


# default set to be max size of image
DEPTH_SIZE_DEFAULT = [640, 480]
COLOR_SIZE_DEFAULT = [1280, 720]
LASER_POWER_DEFAULT = 80.0
RECEIVER_GAIN_DEFAULT = 9.0


class L515_basic_interface():
    def __init__(self, image_size_d = DEPTH_SIZE_DEFAULT, image_size_c = COLOR_SIZE_DEFAULT,
                 laser_power = LASER_POWER_DEFAULT, receiver_gain = RECEIVER_GAIN_DEFAULT):
        self.image_size_d = image_size_d
        self.image_size_c = image_size_c
        self.laser_power = laser_power
        self.receiver_gain = receiver_gain
        self.depth_sensor = None

        # config object
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.image_size_d[0], self.image_size_d[1], rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.image_size_c[0], self.image_size_c[1], rs.format.bgr8, 30)

        self.pipeline = rs.pipeline()

    def startStream(self):
        success = False
        try:
            # Start streaming
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
            print("fail to start stream")
            self.pipeline.stop()
            success = False
        return success

    def stopStream(self):
        self.pipeline.stop()
        print("close stream")

    def takePicture(self, frame = None):
        # if there is no imput frame
        if not frame:
            try:
                frames = self.pipeline.wait_for_frames()
            except:
                return None
        else:
            frames = frame

        # get color and depth
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame: # if frame is empty
            return None

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return color_image, depth_image

    # TODO finish save picture
    def savePicture(self):
        color, depth = self.takePicture()
        date = datetime.now()
        time = date.strftime("%x-%X")


    # will return list that contain lists numpy matrixs. every one of them count as video
    def liveVideo(self):
        print("- press Esc to end Stream -")
        print("- press R to to start and stop the record -")
        return_list = []
        video_list = []
        record_bool = False
        try:
            while True:
                frame = self.pipeline.wait_for_frames()
                color_image, depth_image = self.takePiction(frame) # when the function fail, it will return None, so this line will fail to

                ''' record '''
                if record_bool:
                    video_list.append([color_image, depth_image])

                ''' show images '''
                # resize image
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

                if keyboard.is_pressed('r'):
                    if record_bool:
                        print(" start recording ")
                        record_bool = True
                    else:
                        print(" stop recording, save video ")
                        return_list.append(video_list)
                        video_list = []
                        record_bool = False

        except:
            print("fail to load frame")
        finally:
            if video_list:
                return_list.append(video_list)
            return return_list

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

    # TODO save images (colora and depth) with date name

class JPG_PNG_converter():
    def __init__(self):
        self.main_directory = '/root'
        self.image_dir = '/root/output/images'
        self.video_dir = '/root/output/video'

    def imsave(self, image, name):
        os.chdir(self.image_dir)
        cv2.imwrite(name, image)

    def imload(self, image_path):
        return cv2.imread(image_path)

    def get_file_location(self, file):
        return os.path.dirname(os.path.realpath(file))
