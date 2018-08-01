#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
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
    self.speed_pub = rospy.Publisher("/JaRen/speed",Float32, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/JaRen/app/camera/depth/image_raw",Image,self.callback, queue_size=1)

  

  def callback(self,data):
    try:
      depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
    except CvBridgeError as e:
      print(e) 
    print("#1:")
    print(depth_image)
    depth_array = np.array(depth_image, dtype=np.uint16)
    print("#2:")
    print(depth_array)
    image_np = cv2.imdecode(depth_array, cv2.IMREAD_COLOR)
    print("#3:")
    print(image_np)

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
