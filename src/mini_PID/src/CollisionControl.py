#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys

import time

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt



class CollisionControl:
  
  def __init__(self):
    self.speed_pub = rospy.Publisher("/JaRen/speed",Int16, queue_size=1)
    self.lane_pub = rospy.Publisher("/JaRen/matrix", Int16, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/JaRen/app/camera/depth/image_raw",Image,self.callback, queue_size=1)
    self.lane=1
    self.lastcollision = int(round(time.time() * 1000))

  def callback(self,data):
   #timediff = int(round(time.time() * 1000))-self.lastcollision
   #if timediff > 2000:
    
    #test
    #millis = int(round(time.time() * 1000))
    #print millis



    #print ("#0:")
    #print (data)
    try:
      depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
    except CvBridgeError as e:
      print(e) 
    
    small_image = cv2.resize(depth_image, (0,0), fx=0.02, fy=0.02)    
    print(small_image)




    total_x = small_image.shape[0]
    total_y = small_image.shape[1]
    #just check the innermost 1/9 of the image)
    x = total_x/3-1    #
    y = total_y/3-1   #
    close_pixels = 0
    pixel_proximity = 550
    minimum_amount_to_react = 5
    for xx in range(total_x/3):       #
      x +=1
      y = total_y/3-1   #
      for yy in range(total_y/3): #
        y +=1
        value = small_image[x,y]
        if value != 0 and value < pixel_proximity:
          close_pixels +=1
    #print("DEBUG1: ",pixels_closer_than_600)
    if close_pixels >= minimum_amount_to_react: 
       #to just stop:
       #self.speed_pub.publish(0)

    
       #to switch lane. only do this once every 2 seconds (?)  (super simple naive implementation)   
       self.lastcollision = int(round(time.time() * 1000))
       print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
       print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
       print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
       print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
       print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
       if self.lane == 1:
         self.lane_pub.publish(2) 
         self.lane = 2
         #rospy.sleep(2)
       else:
         self.lane_pub.publish(1) 
         self.lane = 1
         #rospy.sleep(2)
    else: 

       print("...................................................")
       print("...................................................")
       print("...................................................")
       print("...................................................")
       print("...................................................")

def main(args):
  rospy.init_node('CollisionControl', anonymous=True)
  cc = CollisionControl()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
