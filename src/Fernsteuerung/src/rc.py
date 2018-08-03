
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
    elif i == "s":
      RC.speed.publish(0) 
  #end main loop
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
 


if __name__ == '__main__':
    main(sys.argv)


