import numpy as np
import cv2
import os
import datetime

'''
this class use to convert numpy images into PNG file and vice versa.
make use:
    * name is a legal name for a file
    * image is a [x,y,3]-RGB shape matrix
    * dir is the full location of where you want to work on
        - use "self.main_directory" as the root, and add path to the working location
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
        if not os.path.exists(path):
            os.makedirs(path)
            os.makedirs(path + "//color")
            os.makedirs(path + "//depth")
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



""" from here we have same help funciton, the code is useing """

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

