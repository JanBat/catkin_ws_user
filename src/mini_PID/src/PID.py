
#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import tf
import rospy

import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image


from math import sqrt
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
#from nav_msgs.msg import float64
from nav_msgs.msg import Odometry


from v2a import vector2angle

from matplotlib import pyplot as plt

class PID:
    
    def __init__(self):
      self.pub_stop_start = rospy.Publisher(
          "JaRen/stop_start",
          Int16,
          queue_size=100)
      self.pub_speed = rospy.Publisher("JaRen/speed", Int16, queue_size=100)
      self.pub_steering = rospy.Publisher(
          "JaRen/steering",
          UInt8,
          queue_size=100)
      self.sub_odom = rospy.Subscriber(
          "localization/odom/3", Odometry, self.callback, queue_size=10)
      self.sub_matrix = rospy.Subscriber(
          "JaRen/matrix", Int16, self.lanechange, queue_size=10)
      self.lane1 = np.load("matrix100cm_lane1.npy") 
      self.lane2 = np.load("matrix100cm_lane2.npy")
      self.dynamics = self.lane1 #initially, use lane1
  
    def lanechange(self, data):
      print (data)
      if data.data == 1:
        self.dynamics = self.lane1
        print ("Switching to Lane 1")
      elif data.data == 2:
        self.dynamics = self.lane2
        print ("Switching to Lane 2")
  
    def callback(self,data):

      #print("WHEEEEEE")
      #make it move:    
      
      #self.pub_speed.publish(0)
      #self.pub_stop_start.publish(150)
      #rospy.sleep(1)
      #print data
      o = data.pose.pose.orientation
      l = data.pose.pose.position
      x= l.x
      y=l.y
      #print("#####")
      #print("DEBUG")
      #print (x,y)
      #print("data:",data)
      
      #quaternion = data[].x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
      euler = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
      yaw = euler[2]
      
      PI = 3.1415926
      #permanent direction:
      #desiredAngle = PI   #we want go go straight north (absolute direction)

      #desiredAngle based on quadrant (go around clockwise)
      #if y > 2 and x>2:
      #  desiredAngle = PI
      #elif x<=2 and y > 3:
      #  desiredAngle = -0.5*PI
      #elif y<=3 and x <=4:
      #  desiredAngle = 0
      #elif x> 4:
      #  desiredAngle = 0.5*PI
      #else:
      #  desiredAngle = yaw
      
      #desired angle based on force matrix:


      #dynamics = np.load("matrix100cm_lane1.npy")

      #Car position
    
      #Vector for car position
      map_coord_x = self.dynamics[x*10][y*10][0]
      map_coord_y = self.dynamics[x*10][y*10][1]
      #print("map_coord_x/y:", map_coord_x, map_coord_y)

      #Calclating angle of car relativ to vector 
     
      #car_coord_x = np.cos(yaw)*map_coord_x+np.sin(yaw)*map_coord_y
      #car_coord_y = -np.sin(yaw)*map_coord_x+np.cos(yaw)*map_coord_y

      #print(car_coord_x, car_coord_y)
      
      #steering = np.arctan(car_coord_y/(car_coord_x))
      #print(steering)
      desiredAngle = vector2angle([map_coord_x, map_coord_y])
      #print("after v2a:")
      #print("desired yaw:", desiredAngle)
      #print("current yaw:", yaw)
      #(haphazard semi-copypaste)
      #end forcefield stuff


      directionAngle =  desiredAngle - yaw    # direction correction (relative direction)
      while (directionAngle > PI):
        directionAngle -= 2*PI
      while (directionAngle < -PI):
        directionAngle += 2*PI
      
      #directionAngle is now -PI......+PI .. we want 180.....0 instead (?) 
      #not proportionally (TODO)

      directionAngle *= 200 # 280 #was 130 too    #was 280
      directionAngle += 90
      #make it way bigger and afterwards clamp: (thats why *280)
      dir = directionAngle
      if dir > 180:
        dir = 180
      elif dir < 0:
        dir = 0  
      #(directionAngle 0 is mapped to steering 90)
      #-PI is mapped to roughly 87 (good enough) (+PI conversely)


      
      #print("yaw:", yaw, "steering:", dir)
      #print("#####")
      self.pub_steering.publish(int(dir))    
      #self.pub_steering.publish(90)
#      self.pub_steering.publish(90)   #HARD LEFT
#      self.pub_steering.publish(0) #HARD RIGHT
      self.pub_stop_start.publish(0)
      #rospy.sleep(1)
      #self.pub_speed.publish(270)
      #rospy.sleep(1)

  

def main(args):
  rospy.init_node('mini_PID', anonymous=True)
  mini_PID = PID()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
 


if __name__ == '__main__':
    main(sys.argv)
