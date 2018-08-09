#laser localization script

#shall do the following:

#subscribe to /scan

#calculate coordinates relative to car position based on scan ranges

#(publish created map as image

#build internal map and localize itself in it (???????)


#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import math
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry

#####Hilfsfunktionen :




#scale makes the laser coord component of the picture greater (zooms in)
def make_cv_picture_from_laser_space_coordinates(coords, image_width, image_height, scale):
    blank_img =np.zeros((image_height,image_width,3), np.uint8)    
    ### etc. etc.
    #print(coords)
    for c in range(len(coords)):

      coord = coord_to_picture_space(coords[c], image_height, image_width, scale)
      #just paint it white for now:
      if (blank_img.shape[0] > coord[0] and blank_img.shape[1] > coord[1]):
        blank_img[coord[0], coord[1]] = (255,255,255)
    return blank_img

def coord_to_picture_space(coord, img_height, img_width, scale):

    #place origin at image center
    origin = [img_width/2, img_height/2]
    coord_x = origin[0]+coord[0]*scale
    #we subtract y-values because y goes down in the cv picture
    coord_y = origin[1]-coord[1]*scale
    outputcoord = [coord_x, coord_y]
    return outputcoord
    


def get_laser_space_coordinates_from_full_scan(full_scan, self):
    angle_min = full_scan.angle_min
    angle_max = full_scan.angle_max
    angle_increment = full_scan.angle_increment
    
    output_array = []

    #we start iterating at angle_min:
    iteration_angle = angle_min
    i = 0
    while (iteration_angle < angle_max):
      if not np.isinf(full_scan.ranges[i]):
          #coord = get_laser_space_coordinate_from_angle_and_distance(iteration_angle, full_scan.ranges[i], self)
          coord = get_laser_space_coordinate_from_angle_and_distance_modified_by_odom(iteration_angle, full_scan.ranges[i], self)
          output_array.append(coord)
      #...do something smart with coord, e.g. save it or plot it on a map
      
      i += 1
      iteration_angle +=angle_increment
    return (output_array)

def get_laser_space_coordinate_from_angle_and_distance(angle, distance, self):
    #a +/- error snuck in somewhere :> (TODO)  (laser rotates clockwise but starts at -PI)
    x = math.cos(angle)*distance
    y = -math.sin(angle)*distance
    return [x,y]

#returns coordinates that conform with the map
def get_laser_space_coordinate_from_angle_and_distance_modified_by_odom(angle, distance, self):
   
   #bereinigen um eigenwinkel
   angle = angle - self.angle
   PI = 3.1415926

   while angle > PI:
     angle = angle - 2*PI
   while angle < -PI:
     angle = angle + 2*PI
   x = math.cos(angle)*distance
   y = -math.sin(angle)*distance
   
   x += self.position[0]
   y += self.position[1]
   #bereinigen um eigenposition
  
   return [x,y]

def publish_coords_as_pic(coords, self):
    #let's try a scale of 30:
    pic = make_cv_picture_from_laser_space_coordinates(coords, 100, 100, 20)
    
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(pic, "bgr8"))
    except CvBridgeError as e:
      print(e)    

def publish_coords_as_array(coords, self):
    self.array_pub.publish(coords)


######################





class laser_localization:
  
  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)
    self.array_pub = rospy.Publisher("JaRen/LidarArray", Float32, queue_size=1)
    self.panic_pub = rospy.Publisher("JaRen/LidarCollisionDetection", Int16, queue_size=1)
    self.bridge = CvBridge()
    self.lidar_sub = rospy.Subscriber("/JaRen/scan",LaserScan,self.lidar_callback, queue_size=1)
    self.odom_sub = rospy.Subscriber("/localization/odom/3", Odometry, self.odom_callback, queue_size=1)
    self.angle = 0
    self.position = [0,0]
  def odom_callback(self, data):
    p = data.pose.pose.position
    o = data.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
    self.position = [p.x, p.y]
    self.angle = euler
  def lidar_callback(self,data):

    #"proper":    
    coords = get_laser_space_coordinates_from_full_scan(data, self)


    publish_coords_as_array(coords, self)
    publish_coords_as_pic(coords, self)

    
    #makeshift collision detection:
    #detection_at_proximity = 0.7 #meters
    #if data.ranges[-1] < detection_at_proximity:
    #  print("!!!!!!!!!!!!!!!!!!!!!", data.ranges[-1], "!!!!!!!!!!!!!!!!!!!!!")
    #  self.panic_pub.publish(1)
    

def main(args):
  rospy.init_node('laser_localization', anonymous=True)
  ll = laser_localization()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

