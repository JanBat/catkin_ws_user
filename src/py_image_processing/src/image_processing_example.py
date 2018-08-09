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
debugcounter = 0
class image_converter:
  
  def __init__(self):
    self.image_pub_bnw = rospy.Publisher("/image_processing/bnw",Image, queue_size=1)
    self.image_pub_warped = rospy.Publisher("/image_processing/warped",Image, queue_size=1)
    self.image_pub_ransacced = rospy.Publisher("/image_processing/ransacced",Image, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("JaRen/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)
    self.debugcounter = 0
#    from_p = np.array([
#         [56,144],
#         [102,81],
#         [63,389],
#         [197,532]], dtype = "float32")
#    to_p = np.array([
#         [100,100],
#         [400,100],
#         [100,500],
#         [700,500]], dtype = "float32")

    from_p = np.array([
         [153,294],
         [347,290],
         [49,382],
         [440,365]], dtype = "float32")
    to_p = np.array([
         [50,150],
         [225,150],
         [50,200],
         [225,200]], dtype = "float32")

    self.warpMatrix = cv2.getPerspectiveTransform(from_p, to_p)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    except CvBridgeError as e:
      print(e)

    #let's start by ignoring the entire top half =) #bandaidfixes
    #cv_image_small = cv2.resize(cv_image, (0,0), fx=0.2, fy=0.2)     
    #cv_image_half = make_top_half_all_black(cv_image_small)
    #cv_image_ts = threshold(cv_image_half)
    w = cv_image.shape[0]
    h = cv_image.shape[1]
    #crop image:
    cv_image = cv_image[50:400, 50:590]

    #debug:
#    try:ridge.cv2_to_imgmsg(cv_image, "bgr8"))
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


    
#    color_blobs = identify_blobs(rgb_for_blob_identification, 6)
    
    #single red pixel in the center of each blob:
    #for blob_middle_point in color_blobs[2]:
    #  rgb_for_blob_identification[blob_middle_point[0], blob_middle_point[1]] = [0,0,255]
    #print(color_blobs[2])

###copypaste

    #img = color_blobs[0]    #[0] to get the image created by identify_blobs, not the coords. should be pretty colorful.


    #test ransac:
#    coords_for_ransac = []
#    for sublist in color_blobs[1]:
#      for coord in sublist:
#        coords_for_ransac.append(coord)

#    rrr = ransac(coords_for_ransac, 2)
#    image = color_blobs[0]
#    print_lines_onto_cv(rrr, color_blobs[0])
    ####copypaste ende

    # try applying warp matrix:
    warped = cv2.warpPerspective(rgb_for_blob_identification, self.warpMatrix, (600,450))
    try:
      self.image_pub_warped.publish(self.bridge.cv2_to_imgmsg(warped, "bgr8"))
    except CvBridgeError as e:
      print(e)

    try:
      self.image_pub_bnw.publish(self.bridge.cv2_to_imgmsg(rgb_for_blob_identification, "bgr8"))
    except CvBridgeError as e:
      print(e)

#    try:
#      self.image_pub_ransacced.publish(self.bridge.cv2_to_imgmsg(color_blobs[0], "bgr8"))
#    except CvBridgeError as e:
#      print(e)

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
