#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys

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
  

  def callback(self,data):
    
    #print ("#0:")
    #print (data)
    try:
      depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
    except CvBridgeError as e:
      print(e) 
    
    small_image = cv2.resize(depth_image, (0,0), fx=0.02, fy=0.02)    


#    print ("##################################################")
#    print ("##################################################")
#    print ("##################################################")
#    print (small_image)
##    print ("##################################################")
#    print ("##################################################")
#    print ("##################################################")
    #print("#1:")
    #print(depth_image)
    #depth_array = np.array(depth_image, dtype=np.uint16)
    #print("#2:")
    #print(depth_array)
    #print("#2.plot:")
    #print(plt.plot(depth_array))
    #image_np = cv2.imdecode(depth_array, cv2.IMREAD_COLOR)
    #print("#3:")
    #print(image_np)	

    #...xxx...
    #...xxx...
    #.........



    total_x = small_image.shape[0]
    total_y = small_image.shape[1]
    #just check the innermost 1/9 of the image)
    #x = total_x    #
    #y = total_y    #
    close_pixels = 0
    pixel_proximity = 450
    minimum_amount_to_react = 3
    for xx in range(total_x):       #
      #x +=1
      #y = total_y   #
      for yy in range(total_y): #
        #y +=1
        value = small_image[xx,yy]
        if value != 0 and value < pixel_proximity:
          close_pixels +=1
    #print("DEBUG1: ",pixels_closer_than_600)
    if close_pixels >= minimum_amount_to_react: 
#   switch lane. only do this once every 2 seconds (?)  (super simple naive implementation)   

       if self.lane == 1:
         self.lane_pub.publish(2) 
         self.lane = 2
         rospy.sleep(2000)
       else:
         self.lane_pub.publish(1) 
         self.lane = 1
         rospy.sleep(2000)

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
