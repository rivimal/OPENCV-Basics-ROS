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
            img = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") #convert the incoming ROS image message (data) into an OpenCV image using the imgmsg_to_cv2 method with BGR encoding.
        except CvBridgeError as e:                 #if there’s an error during conversion , it catches the CvBridgeError and prints it.
            print(e)  

        #Convert the image to gray scale so the gradient is better visible
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.resize(img,(450,350))

        #Apply the horizontal sobel operator with a kernel size of 3
        sobelx = cv2.Sobel(img,cv2.CV_64F,1,0,ksize=3)

        #Apply the vertical sobel operator with a kernel size of 3
        sobely = cv2.Sobel(img,cv2.CV_64F,0,1,ksize=3)

        cv2.imshow('sobel',img)  
        cv2.imshow('sobelx',sobelx) 
        cv2.imshow('sobely',sobely) 
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