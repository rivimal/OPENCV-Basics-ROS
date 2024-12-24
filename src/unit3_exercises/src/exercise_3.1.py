#!/usr/bin/env python

import numpy as np
import cv2
import rospy

def detectFace():  
    face_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/unit3_exercises/haar_cascades/frontalface.xml')
    eye_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/unit3_exercises/haar_cascades/eye.xml')

    original = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/Course_images/chris.jpg')
    img = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/Course_images/chris.jpg')
    
    original = cv2.resize(original,(500,300))
    img = cv2.resize(img,(500,300))

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    ScaleFactor = 1.2

    minNeighbors = 3
 
    faces = face_cascade.detectMultiScale(gray, ScaleFactor, minNeighbors)
    eyes = eye_cascade.detectMultiScale(gray, ScaleFactor, minNeighbors)


    for (x,y,w,h) in faces:
    
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)  
        roi = img[y:y+h, x:x+w] 
    
    for (x,y,w,h) in eyes:
    
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)  
        roi = img[y:y+h, x:x+w] 

    cv2.imshow('Face',img)
    cv2.imshow('Original', original)
    cv2.waitKey(0)

def main():                                             #main function
    detectFace()              
    rospy.init_node('load_image_node', anonymous=True)  #Initializes a ROS node named 'load_image_node'.
       
    try:
        rospy.spin()                                    #Keeps the program running, waiting for callbacks (incoming images).
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()                         

if __name__ == '__main__':
    main()