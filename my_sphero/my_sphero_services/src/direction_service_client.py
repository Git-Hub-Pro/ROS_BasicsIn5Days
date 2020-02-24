#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest
import sys 

rospy.init_node('crash_direction_service_client') # initialise a ROS node with the name service_client
service_name = "/crash_direction_service"
rospy.wait_for_service(service_name) # wait for the service client /gazebo/delete_model to be running
direction_service = rospy.ServiceProxy(service_name, Trigger) # create the connection to the service
request_object = TriggerRequest()

rate = rospy.Rate(5)

ctrl_c = False
def shutdownhook():
    # works better than the rospy.is_shut_down()
    global ctrl_c
    print "shutdown time!"
    ctrl_c = True

rospy.on_shutdown(shutdownhook)

while not ctrl_c:
    result = direction_service(request_object) # send through the connection the request
    """
    ---                             
    bool success   # indicate succes
    string message # informational, 
    """
    if result.success:
        rospy.logwarn("Success =="+str(result.success)) # print the result given by the service called
        rospy.logwarn("Direction To Go=="+str(result.message)) # print the result given by the service called
    else:
        rospy.loginfo("Success =="+str(result.success)) # print the result given by the service called
        rospy.loginfo("Direction To Go=="+str(result.message)) # print the result given by the service called
    rate.sleep()