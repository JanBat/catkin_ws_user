#!/usr/bin/env python

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt


# from __future__ import print_function




##### makes gray pictures at the moment #####





edgemask = (-1, -1, -1,
            -1,  9, -1,
            -1, -1, -1,
            1) #last parameter: multiplier at the end

blurmask = ( 1,  1,  1,
             1,  1,  1,
             1,  1,  1,
             0.1111111)



def apply_mask(img, mask):
  #make it gray
  b,g,r = cv2.split(img)
  b2, g2, r2 = cv2.split(img)
  #(make duplicate rbgs to save stuff in, so mask only uses original data as input)  

  for i in range(img.shape[0]):
    for j in range(img.shape[1]):  
      intensity = (r[i,j]+ b[i,j]+ g[i,j])/3    #intensity = average of r/b/g
      r[i,j] = intensity 
      b[i,j] = intensity 
      g[i,j] = intensity

 
  #now, apply mask
  for i in range(img.shape[0]):
    for j in range(img.shape[1]):  
      if i == 0 or i >= img.shape[0]-1 or j == 0 or j >= img.shape[1]-1:    #basically if coordinate is on edge. TODO: optimize, this shouldnt be called every single time
        r2[i,j] = 0
        b2[i,j] = 0
        g2[i,j] = 0
      else:
        value = mask[0]*r[i-1,j-1]+ mask[1]*r[i,j-1]+ mask[2]*r[i+1,j-1]+mask[3]*r[i-1,j]+mask[4]*r[i,j]+mask[5]*r[i+1,j]+mask[6]*r[i-1,j+1]+ mask[7]*r[i,j+1]+ mask[8]*r[i+1,j+1]
        value *=mask[9]
        r2[i,j] = value
        b2[i,j] = value
        g2[i,j] = value
  return cv2.merge((b2,g2,r2))
###MAIN###


img = cv2.imread('masked.png')
img = apply_mask(img, edgemask)
cv2.imwrite('masked.png', img)

