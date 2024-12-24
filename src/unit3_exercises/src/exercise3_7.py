#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image                  #importing Image type from package sensor_msgs.msg, used for handling images in ROS
from cv_bridge import CvBridge, CvBridgeError 

class ShowingImage(object):                        #calling the class showing_Image

    def __init__(self):                            #define init method, Initializes the class
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback) #Creates a ROS subscriber that listens to the /camera/rgb/image_raw topic. When a new message arrives, it calls the camera_callback method.
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        self.bridge_object = CvBridge()   
        self.rgb_image = None
        self.detected_size = None

    def camera_callback(self,data):                #define camera_callback method , This is called whenever a new image is received from the subscribed topic.

        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.rgb_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") #convert the incoming ROS image message (data) into an OpenCV image using the imgmsg_to_cv2 method with BGR encoding.
        except CvBridgeError as e:                 #if there’s an error during conversion , it catches the CvBridgeError and prints it.
            print(e)

    def depth_callback(self, data):
        try:
            depth_image = self.bridge_object.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:                 #if there’s an error during conversion , it catches the CvBridgeError and prints it.
            print(e)

        height_from_ground = depth_image[depth_image.shape[0]//2, depth_image.shape[1]//2]

        if height_from_ground > 1.96:
            detectPeople(self, self.rgb_image)
            # showPeoplePosition(self.rgb_image)                            

def detectPeople(self, img):

    # Lets initialize the HOG descriptor
    hog = cv2.HOGDescriptor()

        #We set the hog descriptor as a People detector
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    #The image is pretty big so we will gibve it a resize
    imX = 700
    imY = 500
    img = cv2.resize(img,(imX,imY))

    #We will define de 8x8 blocks in the winStride
    boxes, weights = hog.detectMultiScale(img, winStride=(8,8))
    boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

    for (xA, yA, xB, yB) in boxes:
        
        showPeoplePosition(self, xA, xB) 
        #Center in X 
        medX = xB - xA 
        xC = int(xA+(medX/2)) 

        #Center in Y
        medY = yB - yA 
        yC = int(yA+(medY/2)) 

        #Draw a circle in the center of the box 
        cv2.circle(img,(xC,yC), 1, (0,255,255), -1)

        # display the detected boxes in the original picture
        cv2.rectangle(img, (xA, yA), (xB, yB),
                            (255, 255, 0), 2)   

    cv2.imshow('frame_2',img)
    cv2.waitKey(1)

def showPeoplePosition(self, x, w):
    
        center_x = x + w // 2

        # Get the width of the image
        image_width = self.rgb_image.shape[1]

        # Determine if the person is on the left or right
        if center_x < image_width // 3:  # Left side (you can adjust the threshold as needed)
            current_side = "left"

        elif center_x > (image_width * 2) // 3:  # Right side
            current_side = "right"

        else:
            current_side = "center"

        if current_side and current_side != self.detected_size:
            self.detected_size = current_side
            print(f"Person detected on the {current_side} side.")
            return

def main():                                             #main function
    ShowingImage()              
    rospy.init_node('load_image_node', anonymous=True)  #Initializes a ROS node named 'load_image_node'.
       
    try:
        rospy.spin()                                    #Keeps the program running, waiting for callbacks (incoming images).
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()                         

if __name__ == '__main__':
    main()