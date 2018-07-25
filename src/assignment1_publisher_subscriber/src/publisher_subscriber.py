#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from std_msgs.msg import Float32




def callback(raw_msg):
	yaw_msg = "I heard: " + str(raw_msg)
	publisher.publish(yaw_msg)


#Initialize node
rospy.init_node("model_car_yaw_node_test_12345")

#run subscriber
rospy.Subscriber("/yaw", Float32, callback)
publisher = rospy.Publisher("/assignment1_publisher_subscriber12345", String, queue_size=10)

#idle
rospy.spin()



#####comments


#source devel/setup.bash to synchronize stuff
