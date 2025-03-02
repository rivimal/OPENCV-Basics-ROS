#!/usr/bin/env python

import cv2
import numpy as np

image = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_4/Course_images/corner_test_2.png')

gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

fast = cv2.FastFeatureDetector_create() 

#Keypoints using non Max Supression
Keypoints_1 = fast.detect(gray, None)

#Set non Max Supression disabled 
fast.setNonmaxSuppression(False)

#Keypoints without non max Suppression
Keypoints_2 = fast.detect(gray, None)

#Create  instance of the original image

image_without_nonmax = np.copy(image)

# Draw keypoints on top of the input image

cv2.drawKeypoints(image, Keypoints_2, image_without_nonmax, color=(0,35,250))


cv2.imshow('Without non max Supression',image_without_nonmax)
cv2.waitKey(0)
cv2.destroyAllWindows()