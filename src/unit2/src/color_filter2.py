#!/usr/bin/env python

import rospy                                       #importing rospy library
from sensor_msgs.msg import Image                  #importing Image type from package sensor_msgs.msg, used for handling images in ROS
from cv_bridge import CvBridge, CvBridgeError      #importing CvBrige, CvBridgeError types from cv_bridge package,to convert between ROS Image messages and OpenCV images.
import cv2                                         #importing cv library
import numpy as np                                 #Import the numpy library which will help with some matrix operations

class ShowingImage(object):                        #calling the class showing_Image

    def __init__(self):                            #define init method, Initializes the class
    

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback) #Creates a ROS subscriber that listens to the /camera/rgb/image_raw topic. When a new message arrives, it calls the camera_callback method.
        self.bridge_object = CvBridge()            #Initializes a CvBridge object to convert between ROS and OpenCV image formats.

    def camera_callback(self,data):                #define camera_callback method , This is called whenever a new image is received from the subscribed topic.
        try:
            # We select bgr8 because its the OpenCV encoding by default
            image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") #convert the incoming ROS image message (data) into an OpenCV image using the imgmsg_to_cv2 method with BGR encoding.
        except CvBridgeError as e:                 #if there’s an error during conversion , it catches the CvBridgeError and prints it.
            print(e)

        #I resized the image so it can be easier to work with
        #image = cv2.resize(image,(300,300))

        #Once we read the image we need to change the color space to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #Hsv limits are defined
        #here is where you define the range of the color you´re looking for
        #each value of the vector corresponds to the H,S & V values respectively
        min_green = np.array([40,50,50])
        max_green = np.array([80,255,255])

        min_red = np.array([0,50,50])
        max_red = np.array([10,255,255])

        min_blue = np.array([100,50,50])
        max_blue = np.array([140,255,255])

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
        #cv2.waitKey(1)
        cv2.imshow('Blue',res_b)
        #cv2.waitKey(1)
        cv2.imshow('Red',res_r) 
        #cv2.waitKey(1)       
        cv2.imshow('image',image)
        cv2.waitKey(1)

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