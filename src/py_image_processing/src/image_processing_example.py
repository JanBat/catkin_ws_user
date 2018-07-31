#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
from blobs import identify_blobs       #to use
from ransac import ransac
from cv_extensions import print_lines_onto_cv
from cv_extensions import make_top_half_all_black
from cv_extensions import remove_all_non_grayscale
from cv_extensions import threshold

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
sys.setrecursionlimit(1500)

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)

  

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #let's start by ignoring the entire top half =) #bandaidfixes
    cv_image = cv2.resize(cv_image, (0,0), fx=0.2, fy=0.2)     
    cv_image = make_top_half_all_black(cv_image)
    cv_image = threshold(cv_image)
    

    #debug:
#    try:
#      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#    except CvBridgeError as e:
#      print(e)
    ####end debug


    #make it gray
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 230
    ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);


    # identify blobs!

    rgb_for_blob_identification = cv2.cvtColor(thresh1, cv2.COLOR_GRAY2RGB)


    
    color_blobs = identify_blobs(rgb_for_blob_identification)


###copypaste

    img = color_blobs[0]    #[0] to get the image created by identify_blobs, not the coords. should be pretty colorful.


    #test ransac:
    coords_for_ransac = []
    for sublist in color_blobs[1]:
      for coord in sublist:
        coords_for_ransac.append(coord)

    rrr = ransac(coords_for_ransac, 3)
    image = color_blobs[0]
    print_lines_onto_cv(rrr, image)
    ###copypaste ende

#commented during debug:
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
