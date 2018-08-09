
#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import tf
import rospy
import time

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

#initialize lastTime

lastTime = int(round(time.time() * 1000))
errsum = 0
err_ring_puffer = [0,0,0,0,0,0,0,0,0,0]
err_ring_sum = 0
lasterr = 0
kp = 170
ki = 0.000000001
kd = 160


def PID_correction(error):
        now = int(round(time.time() * 1000))
        timechange = now - lastTime
        
        #global errsum
        global lasterr
        global lasttime    
        global err_ring_puffer
        global err_ring_sum

        #errsum += error*timechange
        err_ring_sum -= err_ring_puffer[0]
        del err_ring_puffer[0]
        err_ring_puffer.append(error)
        err_ring_sum += error
        #print (err_ring_puffer, "sum:", err_ring_sum)
        derr = (error - lasterr) / timechange

        lasterr = error
        lasttime = now




        #error: current direction vs desired direction

        #errsum: in which direction do we overshoot more?
  
        #derr: how much did error change since last turn?

        output = kp* error + ki * err_ring_sum + kd *derr

        #to make it fit the 0..180 value field of the actual steering:
        output +=90   #to make 90 the middle value
        if output > 180:
          output = 180
        elif output < 0:
          output = 0

        return output
 
    



 

class PID:
    
    def __init__(self):
      self.pub_stop_start = rospy.Publisher(
          "JaRen/stop_start",
          Int16,
          queue_size=100)
      self.pub_speed = rospy.Publisher("JaRen/speed", Int16, queue_size=1)
      self.pub_steering = rospy.Publisher(
          "JaRen/steering",
          UInt8,
          queue_size=1)
      self.sub_odom = rospy.Subscriber(
          "localization/odom/3", Odometry, self.callback, queue_size=1)
      self.sub_matrix = rospy.Subscriber(
          "JaRen/RC/lane", Int16, self.lanechange, queue_size=1)

      self.sub_kp = rospy.Subscriber(
          "JaRen/RC/kp", Float32, self.kpcallback, queue_size=1)
      self.sub_ki = rospy.Subscriber(
          "JaRen/RC/ki", Float32, self.kicallback, queue_size=1)
      self.sub_kd = rospy.Subscriber(
          "JaRen/RC/kd", Float32, self.kdcallback, queue_size=1)



      #self.RCspeed = rospy.Subscriber("JaRen/RC/
      #    "

      self.lane1 = np.load("matrix100cm_lane1.npy") 
      self.lane2 = np.load("matrix100cm_lane2.npy")
      self.dynamics = self.lane1 #initially, use lane1
      lastTime = int(round(time.time() * 1000))

     


    def kpcallback(self, data):
      global kp
      global ki
      global kd  
      kp = data.data
      print ("kp, ki, kd:", kp, ki, kd)

    def kicallback(self, data):
      global kp
      global ki
      global kd
      ki = data.data
      print ("kp, ki, kd:", kp, ki, kd)

    def kdcallback(self, data):
      global kp
      global ki
      global kd
      kd = data.data
      print ("kp, ki, kd:", kp, ki, kd)
 

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
      xx =int(x*10)
      yy =int(y*10)
      
      if xx<self.dynamics.shape[0] and yy<self.dynamics.shape[1]:

      
        map_coord_x = self.dynamics[xx][yy][0]
        map_coord_y = self.dynamics[xx][yy][1]
      else:
        map_coord_x = self.dynamics[0][0][0]
        map_coord_y = self.dynamics[0][0][1]
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

#############pseudo-PID below

#      directionAngle *= kp # 280 #was 130 too    #was 280
#      directionAngle += 90
#      #make it way bigger and afterwards clamp: (thats why *280)
#      steering = directionAngle
#      if steering > 180:
#        steering = 180
#      elif steering < 0:
#        steering = 0  
#      #(directionAngle 0 is mapped to steering 90)
#      #-PI is mapped to roughly 87 (good enough) (+PI conversely)

#############pseudo-PID above
 
#"directionAngle" is the desired change of angle. (at 0, don't steer)

########"proper" PID:
#      print("####################")
#      print (directionAngle)
      steering =PID_correction(directionAngle)
#      print(steering)
#      print("####################")
##################
      
      #print("yaw:", yaw, "steering:", dir)
      #print("#####")
      self.pub_steering.publish(int(steering))    
      #self.pub_steering.publish(90)
#      self.pub_steering.publish(90)   #HARD LEFT
#      self.pub_steering.publish(0) #HARD RIGHT
      self.pub_stop_start.publish(0)
      #rospy.sleep(1)
      #self.pub_speed.publish(270)
      #rospy.sleep(1)

      #millis = int(round(time.time() * 1000))
      #print millis


#somewhat copypasty (simple PID):


def main(args):
  rospy.init_node('mini_PID', anonymous=True)
  mini_PID = PID()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
 


if __name__ == '__main__':
    main(sys.argv)
