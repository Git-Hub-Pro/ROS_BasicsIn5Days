#! /usr/bin/env python
import rospkg
import rospy
from std_srvs.srv import Empty, EmptyRequest

rospy.init_node('service_move_bb8_in_circle_client') # Initialise a ROS node with the name service_move_bb8_in_circle_client
rospy.wait_for_service('/move_bb8_in_circle') # wait for the service client /move_bb8_in_circle to be running
move_bb8_in_circle_service_client = rospy.ServiceProxy('/move_bb8_in_circle',Empty) # Create the connection to the service
move_bb8_in_circle_request_object = EmptyRequest() # create an object of type EmptyRequest

result = move_bb8_in_circle_service_client(move_bb8_in_circle_request_object) # Send Through the connection path to the trajectory file to be executed
print result # print the result given by the service called 
