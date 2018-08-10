
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




from matplotlib import pyplot as plt



class RemoteControl:
    
    def __init__(self):
      self.stop_start = rospy.Publisher(
          "JaRen/RC/stop_start",
          Int16,
          queue_size=100)
      self.speed = rospy.Publisher("JaRen/speed", Int16, queue_size=1)
      self.steering = rospy.Publisher(
          "JaRen/RC/steering",
          UInt8,
          queue_size=1)
      self.lane = rospy.Publisher("JaRen/RC/lane", Int16, queue_size=1)
      self.kp = rospy.Publisher("JaRen/RC/kp", Float32, queue_size=1)
      self.ki = rospy.Publisher("JaRen/RC/ki", Float32, queue_size=1)
      self.kd = rospy.Publisher("JaRen/RC/kd", Float32, queue_size=1)
       
      #for collisioncontrol:

      self.pub_min_x = rospy.Publisher(
          "JaRen/RC/minx", Float32, queue_size=1)
      self.pub_max_x = rospy.Publisher(
          "JaRen/RC/maxx", Float32, queue_size=1)
      self.pub_min_y = rospy.Publisher(
          "JaRen/RC/miny", Float32, queue_size=1)
      self.pub_max_y = rospy.Publisher(
          "JaRen/RC/maxy", Float32, queue_size=1)
      self.pub_col_dist = rospy.Publisher(
          "JaRen/RC/coldist", Float32, queue_size=1)
      self.pub_col_num = rospy.Publisher(
          "JaRen/RC/colnum", Float32, queue_size=1)
      self.pub_col_mode = rospy.Publisher(
          "JaRen/RC/colmode", Float32, queue_size=1)
      self.pub_time_between_collisions = rospy.Publisher(
          "JaRen/RC/timebtwcol", Float32, queue_size=1)

      self.pub_camera_on_off = rospy.Subscriber(
          "JaRen/RC/camera_on_off", Float32, queue_size=1)

      
 
def main(args):
  rospy.init_node('RemoteControl', anonymous=True)
  RC = RemoteControl()


  def publisher_menu(publisher, welcome_message):
    print (welcome_message)
    #value = ''
    value = raw_input("...")
    while value != "q":
      if i == "s":
        RC.speed.publish(0)       
      else:
        publisher.publish(float(value))
      value = raw_input("...")




  #main loop:
  input = ''

  i = ''
  while(i != 'q'):

    print("Press # + Enter for the following:")
    print("1) change speed")
    print("2) change lane")
    print("3) change Kp")
    print("4) change Ki")
    print("5) change Kd")
    print("++++++++++++")
    print("6) change min_x")
    print("7) change max_x")
    print("8) change min_y")
    print("9) change max_y")
    print("10) change col_dist")
    print("11) change col_num")
    print("12) change col_mode")
    print("13) change time_between_collisions")
    print("14) collision camera on/off")    

    i = raw_input("...")
    if i == '1':
      publisher_menu(RC.speed, "Number + Enter to change speed. q to quit")
    elif i == '2':
      publisher_menu(RC.lane, "Number + Enter to change lane. q to quit")
    elif i == '3':
      publisher_menu(RC.kp, "Number + Enter to change kp. q to quit")

    elif i == '4':
      publisher_menu(RC.ki, "Number + Enter to change ki. q to quit")

    elif i == '5':
      publisher_menu(RC.kd, "Number + Enter to change kd. q to quit")

    elif i == '6':
      publisher_menu(RC.pub_min_x, "Number + Enter to change min_x. q to quit")

    elif i == '7':
      publisher_menu(RC.pub_max_x, "Number + Enter to change max_x. q to quit")

    elif i == '8':
      publisher_menu(RC.pub_min_y, "Number + Enter to change min_y. q to quit")

    elif i == '9':
      publisher_menu(RC.pub_max_y, "Number + Enter to change max_y. q to quit")

    elif i == '10':
      publisher_menu(RC.pub_col_dist, "Number + Enter to change col_dist. q to quit")

    elif i == '11':
      publisher_menu(RC.pub_col_num, "Number + Enter to change col_num. q to quit")

    elif i == '12':
      publisher_menu(RC.pub_col_mode, "Number + Enter to change col_mode. q to quit")

    elif i == '13':
      publisher_menu(RC.pub_time_between_collisions, "Number + Enter to change time btw. collisions. q to quit")
    elif i == '14':
      publisher_menu(RC.pub_camera_on_off, "1 to turn camera on, 0 to turn off, q to quit")

    elif i == "s":
      RC.speed.publish(0) 
  #end main loop
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
 


if __name__ == '__main__':
    main(sys.argv)


