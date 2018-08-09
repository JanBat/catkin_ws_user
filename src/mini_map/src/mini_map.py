

#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import math
import rospy
import cv2
import numpy as np
import os
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
####Specs.:

#build map from .txt files (lanes etc.)

#keep track of information gathered by odometry, lidar, cameras

#periodically print map to screen, enriched by additional information

#(?)send navigation recommendations to appropriate nodes (?)




########### copypaste functions:



def read_points(map_file, offset_x=0.0, offset_y=0.0):
    """
    Reads a file with the map data in the RNDF Format
    :return: generator of x, y position tuples
    """

    # detect waypoints x.y.z and store latitude / longitude as x / y
    with open(map_file) as m_file:
        for line in m_file:
            if line.startswith('1.1.'):
                x, y = line.split('\t')[1:3]
                yield [float(x) + offset_x, float(y) + offset_y]
    with open(map_file) as m_file:
        for line in m_file:
            if line.startswith('1.2.'):
                x, y = line.split('\t')[1:3]
                yield [float(x) + offset_x, float(y) + offset_y]


def relative_filename(name):
  return os.path.join(os.path.dirname(__file__), name)


#####Teilfunktionen :


#collects information about lanes
def build_base_map(self):
  
  points_lane_1 = read_points(relative_filename('new_map_loop1.txt'))
  points_lane_2 = read_points(relative_filename('new_map_loop2.txt'))
  for point in points_lane_1: 
    self.lane1.append(point)
  for point in points_lane_2: 
    self.lane2.append(point)



#creates a 80*80 grid, initialized with ' '
#looks at self.lane1 etc. and overwrites ' ' where reasonable
#prints resulting map to screen
def print_map(self):
  #canvas:
  canvas = []
  for x in range(60):
    column = []
    canvas.append(column)
    for y in range(40):
      cell = " "
      column.append(cell)


  #add lane information:
  for coord in self.lane1:
    x = int(coord[0]*10)
    y = int(coord[1]*10)
    canvas[x][y] = "1"


  for coord in self.lane2:
    x = int(coord[0]*10)
    y = int(coord[1]*10)
    canvas[x][y] = "2"


  #add laser information:
  for coord in self.laser_array:
    x = int(coord[0]*10)
    y = int(coord[1]*10)
    canvas[x][y] = "#"
     


  #add car position:
  canvas[int(self.car_position[0])][int(self.car_position[1])] = 'O'





#do the actual printing:
  for y in range(len(canvas[0])):
    line = ""
    for x in range(len(canvas)):
      line += canvas[x][y]
    print (line)

######################


#map matrices:
#[10cm steps][10cm steps]

class mini_map:
  
  def __init__(self):
    #self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)
    #self.bridge = CvBridge()
    self.odom_sub = rospy.Subscriber("/localization/odom/3", Odometry, self.odom_callback, queue_size=1)
    self.lidar_sub = rospy.Subscriber("/JaRen/LidarArray", Float32, self.lidar_callback, queue_size=1)
    self.lane1 = []
    self.lane2 = []
    self.laser_array = []
    self.base_map = build_base_map(self)
    self.car_position = [20,20]
    #print_map(self)
  

  def lidar_callback(self,data):
    self.laser_array = data

  def odom_callback(self,data):
    #print ("odom callback!!!")
    self.car_position = [data.pose.pose.position.x*10, data.pose.pose.position.y*10]
    print_map(self)    
    
def main(args):
  rospy.init_node('mini_map', anonymous=True)
  mm = mini_map()
  #while (1):
  #  print_map(mm)
  #  rospy.sleep(1)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

