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
    self.lane_pub = rospy.Publisher("/JaRen/RC/lane", Int16, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/JaRen/app/camera/depth/image_raw",Image,self.callback, queue_size=1)
    self.lane=1
    self.lastcollision = int(round(time.time() * 1000))
    self.panic_sub = rospy.Subscriber("JaRen/LidarCollisionDetection", Int16, self.paniccallback, queue_size=1)

    #channels for rc control:
    self.sub_min_x = rospy.Subscriber(
          "JaRen/RC/minx", Float32, self.minxcallback, queue_size=1)
    self.sub_max_x = rospy.Subscriber(
          "JaRen/RC/maxx", Float32, self.maxxcallback, queue_size=1)
    self.sub_min_y = rospy.Subscriber(
          "JaRen/RC/miny", Float32, self.minycallback, queue_size=1)
    self.sub_max_y = rospy.Subscriber(
          "JaRen/RC/maxy", Float32, self.maxycallback, queue_size=1)
    self.sub_col_dist = rospy.Subscriber(
          "JaRen/RC/coldist", Float32, self.coldistcallback, queue_size=1)
    self.sub_col_num = rospy.Subscriber(
          "JaRen/RC/colnum", Float32, self.colnumcallback, queue_size=1)
    self.sub_col_mode = rospy.Subscriber(
          "JaRen/RC/colmode", Float32, self.colmodecallback, queue_size=1)
    self.sub_time_between_collisions = rospy.Subscriber(
          "JaRen/RC/timebtwcol", Float32, self.timebtwcolcallback, queue_size=1)
    self.sub_camera_on_off = rospy.Subscriber(
          "JaRen/RC/camera_on_off", Float32, self.cameraonoffcallback, queue_size=1)


    #justier-variablen:
    self.min_x = 10
    self.max_x = 20
    self.min_y = 10
    self.max_y = 20
    self.col_dist = 600
    self.col_num = 5
    self.col_mode = 0   #0 = stop || 1 = change lane || 2 = ignore
    self.time_between_collisions = 2000 #(ms)
    self.camera_on = 1  #1 = on || 0 = off

  def paniccallback(self, data):
     #if data.data ==1:
       timediff = int(round(time.time() * 1000))-self.lastcollision
       if timediff > self.time_between_collisions:
        self.lastcollision = int(round(time.time() * 1000))    
        if self.lane == 1:
         self.lane_pub.publish(2) 
         self.lane = 2
         #rospy.sleep(2)
        else:
         self.lane_pub.publish(1) 
         self.lane = 1
         #rospy.sleep(2)


  def cameraonoffcallback(self, data):
     self.camera_on = int(data.data)
     print ("Collision Control Values")
     print("X:", self.min_x," - ", self.max_x)
     print("Y:", self.min_y," - ", self.max_y)
     print("Distance: ", self.col_dist, " Amount: ", self.col_num)
     print("Mode: ", self.col_mode, " time between coll.:", self.time_between_collisions)


  def minxcallback(self, data):
     self.min_x = int(data.data)
     print ("Collision Control Values")
     print("X:", self.min_x," - ", self.max_x)
     print("Y:", self.min_y," - ", self.max_y)
     print("Distance: ", self.col_dist, " Amount: ", self.col_num)
     print("Mode: ", self.col_mode, " time between coll.:", self.time_between_collisions)

  def maxxcallback(self, data):
     self.max_x = int(data.data)
     print ("Collision Control Values")
     print("X:", self.min_x," - ", self.max_x)
     print("Y:", self.min_y," - ", self.max_y)
     print("Distance: ", self.col_dist, " Amount: ", self.col_num)
     print("Mode: ", self.col_mode, " time between coll.:", self.time_between_collisions)


  def minycallback(self, data):
     self.min_y = int(data.data)
     print ("Collision Control Values")
     print("X:", self.min_x," - ", self.max_x)
     print("Y:", self.min_y," - ", self.max_y)
     print("Distance: ", self.col_dist, " Amount: ", self.col_num)
     print("Mode: ", self.col_mode, " time between coll.:", self.time_between_collisions)


  def maxycallback(self, data):
     self.max_y = int(data.data)
     print ("Collision Control Values")
     print("X:", self.min_x," - ", self.max_x)
     print("Y:", self.min_y," - ", self.max_y)
     print("Distance: ", self.col_dist, " Amount: ", self.col_num)
     print("Mode: ", self.col_mode, " time between coll.:", self.time_between_collisions)


  def coldistcallback(self, data):
     self.col_dist = int(data.data)
     print ("Collision Control Values")
     print("X:", self.min_x," - ", self.max_x)
     print("Y:", self.min_y," - ", self.max_y)
     print("Distance: ", self.col_dist, " Amount: ", self.col_num)
     print("Mode: ", self.col_mode, " time between coll.:", self.time_between_collisions)


  def colnumcallback(self, data):
     self.col_num = int(data.data)
     print ("Collision Control Values")
     print("X:", self.min_x," - ", self.max_x)
     print("Y:", self.min_y," - ", self.max_y)
     print("Distance: ", self.col_dist, " Amount: ", self.col_num)
     print("Mode: ", self.col_mode, " time between coll.:", self.time_between_collisions)


  def colmodecallback(self, data):
     self.col_mode = int(data.data)
     print ("Collision Control Values")
     print("X:", self.min_x," - ", self.max_x)
     print("Y:", self.min_y," - ", self.max_y)
     print("Distance: ", self.col_dist, " Amount: ", self.col_num)
     print("Mode: ", self.col_mode, " time between coll.:", self.time_between_collisions)


  def timebtwcolcallback(self, data):
     self.time_between_collisions = int(data.data)
     print ("Collision Control Values")
     print("X:", self.min_x," - ", self.max_x)
     print("Y:", self.min_y," - ", self.max_y)
     print("Distance: ", self.col_dist, " Amount: ", self.col_num)
     print("Mode: ", self.col_mode, " time between coll.:", self.time_between_collisions)






  def callback(self,data):
    timediff = int(round(time.time() * 1000))-self.lastcollision
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
    if self.camera_on == 1:
      print(small_image)


    

    total_x = small_image.shape[0]
    total_y = small_image.shape[1]
    

    #just check the innermost 1/9 of the image)
    x = self.min_x #total_x/3-1    #
    y = self.min_y #total_y/3-1   #
    close_pixels = 0
    pixel_proximity = self.col_dist #550
    minimum_amount_to_react = self.col_num #5
    for xx in range(self.max_x - self.min_x):#total_x/3):       #
      x +=1
      y = self.min_y #total_y/3-1   #
      for yy in range(self.max_y - self.min_y): #total_y/3): #
        y +=1
        value = small_image[x,y]
        if value != 0 and value < pixel_proximity:
          close_pixels +=1
    #print("DEBUG1: ",pixels_closer_than_600)
    if close_pixels >= minimum_amount_to_react: 
     if timediff > self.time_between_collisions:    
      self.lastcollision = int(round(time.time() * 1000))
      if self.col_mode == 0:   
       #to just stop:
       self.speed_pub.publish(0)
      elif self.col_mode ==1:
       #to switch lane. only do this once every 2 seconds (?)  (super simple naive implementation)   
       
       if self.camera_on == 1:
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
      #else: NOP
    else: 
       if self.camera_on == 1:
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
