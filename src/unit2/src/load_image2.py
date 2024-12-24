#!/usr/bin/env python

import rospy                                       #importing rospy library
from sensor_msgs.msg import Image                  #importing Image type from package sensor_msgs.msg, used for handling images in ROS
from cv_bridge import CvBridge, CvBridgeError      #importing CvBrige, CvBridgeError types from cv_bridge package,to convert between ROS Image messages and OpenCV images.
import cv2                                         #importing cv library


class ShowingImage(object):                        #calling the class showing_Image

    def __init__(self):                            #define init method, Initializes the class
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback) #Creates a ROS subscriber that listens to the /camera/rgb/image_raw topic. When a new message arrives, it calls the camera_callback method.
        self.bridge_object = CvBridge()            #Initializes a CvBridge object to convert between ROS and OpenCV image formats.

    def camera_callback(self,data):                #define camera_callback method , This is called whenever a new image is received from the subscribed topic.
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") #convert the incoming ROS image message (data) into an OpenCV image using the imgmsg_to_cv2 method with BGR encoding.
        except CvBridgeError as e:                 #if there’s an error during conversion , it catches the CvBridgeError and prints it.
            print(e)
        
        cv2.imshow('image_drone',cv_image)         #Displays the converted image in a window titled 'image_drone'
        cv2.waitKey(2)                             #Waits for 2 milliseconds  

        cv2.imwrite('/home/user/catkin_ws/src/unit2/drone_image.jpg', cv_image) #Saves the displayed image to the specified path as 'drone_image.jpg'.

        

def main():                                        #main function
    showing_image_object = ShowingImage()          #Creates an instance of ShowingImage, which starts the ROS subscriber
    rospy.init_node('load_image_node', anonymous=True) #Initializes a ROS node named 'load_image_node'.
    img = cv2.imread('/home/user/catkin_ws/src/unit2/drone_image.jpg') #Reads a test image from a specified path using OpenCV
    cv2.imshow('image', img)                        #Displays the test image in a window titled 'image'.
    cv2.waitKey(0)
    
    try:
        rospy.spin()                                #Keeps the program running, waiting for callbacks (incoming images).
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()                         

if __name__ == '__main__':
    main()