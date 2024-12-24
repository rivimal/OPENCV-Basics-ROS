#!/usr/bin/env python

import rospy                                       #importing rospy library
from sensor_msgs.msg import Image                  #importing Image type from package sensor_msgs.msg, used for handling images in ROS
from cv_bridge import CvBridge, CvBridgeError      #importing CvBrige, CvBridgeError types from cv_bridge package,to convert between ROS Image messages and OpenCV images.
import cv2   
from cv2 import aruco                                      


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
        
        cv2.imshow('AR_Tag',cv_image)         #Displays the converted image in a window titled 'image_drone'
        cv2.waitKey(2)                             #Waits for 2 milliseconds  

        cv2.imwrite('/home/user/catkin_ws/src/unit5/AR_Tag.jpg', cv_image) #Saves the displayed image to the specified path as 'drone_image.jpg'.
        
        image = cv_image
        h,w = image.shape[:2]

        image = cv2.resize(image,(int(w*0.7), int(h*0.7)))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        #Initialize the aruco Dictionary and its parameters 
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()

        #Detect the corners and id's in the examples 
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        #First we need to detect the markers itself, so we can later work with the coordinates we have for each.
        frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)

        #Show the markers detected
        cv2.imshow('markers',frame_markers)
        cv2.waitKey(0)
        cv2.destroyAllWindows()



def main():                                        #main function
    showing_image_object = ShowingImage()          #Creates an instance of ShowingImage, which starts the ROS subscriber
    rospy.init_node('load_image_node', anonymous=True) #Initializes a ROS node named 'load_image_node'.
    
    try:
        rospy.spin()                                #Keeps the program running, waiting for callbacks (incoming images).
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()                         

if __name__ == '__main__':
    main()