#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.std_srvs
from geometry_msgs.msg import Twist

def my_callback(request):
    rospy.loginfo("The Service move_bb8_in_circle has been called")
    move_circle.linear.x  = 0.2
    move_circle.angular.z = 0.2
    my_pub.publish(move_circle)
    rospy.loginfo("Finished service move_bb8_in_circle")
    return EmptyResponse() # the service Response class, in this case EmptyResponse

rospy.init_node('service_move_bb8_in_circle_server')
my_service = rospy.Service('/move_bb8_in_circle',Empty,my_callback) # create the service called move_bb8_in_circle with the defined callback
my_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
move_circle = Twist()
rospy.loginfo("Service /move_bb8_in_circle Ready")
rospy.spin() # maintain the service open.