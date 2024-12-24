#!/usr/bin/env python

import rospy                                       #importing rospy library
from sensor_msgs.msg import Image                  #importing Image type from package sensor_msgs.msg, used for handling images in ROS
from cv_bridge import CvBridge, CvBridgeError      #importing CvBrige, CvBridgeError types from cv_bridge package,to convert between ROS Image messages and OpenCV images.
import cv2                                         #importing cv library
import numpy as np                                 #Import the numpy library which will help with some matrix operations

class ShowingImage(object):                        #calling the class showing_Image

    def __init__(self):                            #define init method, Initializes the class
        
        
        image = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_2/Course_images/Filtering.png')

        #I resized the image so it can be easier to work with
        image = cv2.resize(image,(300,300))

        #Once we read the image we need to change the color space to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #Hsv limits are defined
        #here is where you define the range of the color you´re looking for
        #each value of the vector corresponds to the H,S & V values respectively
        min_green = np.array([50,220,220])
        max_green = np.array([60,255,255])

        min_red = np.array([170,220,220])
        max_red = np.array([180,255,255])

        min_blue = np.array([110,220,220])
        max_blue = np.array([120,255,255])

        #This is the actual color detection 
        #Here we will create a mask that contains only the colors defined in your limits
        #This mask has only one dimention, so its black and white }
        mask_g = cv2.inRange(hsv, min_green, max_green)
        mask_r = cv2.inRange(hsv, min_red, max_red)
        mask_b = cv2.inRange(hsv, min_blue, max_blue)

        #We use the mask with the original image to get the colored post-processed image
        res_b = cv2.bitwise_and(image, image, mask= mask_b)
        res_g = cv2.bitwise_and(image,image, mask= mask_g)
        res_r = cv2.bitwise_and(image,image, mask= mask_r)

        cv2.imshow('Green',res_g)
        cv2.imshow('Blue',res_b)
        cv2.imshow('Red',res_r)        
        cv2.imshow('image',image)
        cv2.waitKey(0)

def main():                                             #main function
    showing_image_object = ShowingImage()               #Creates an instance of ShowingImage, which starts the ROS subscriber
    rospy.init_node('load_image_node', anonymous=True)  #Initializes a ROS node named 'load_image_node'.
       
    try:
        rospy.spin()                                    #Keeps the program running, waiting for callbacks (incoming images).
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()                         

if __name__ == '__main__':
    main()