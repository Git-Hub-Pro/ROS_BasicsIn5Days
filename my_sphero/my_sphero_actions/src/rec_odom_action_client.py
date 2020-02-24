#! /usr/bin/env python

import rospy
import time
import actionlib
from my_sphereo_actions.msg import record_odomGoal, record_odomFeedback, record_odomResult, record_odomAction
from nav_msgs.msg import Odometry


# definition of the feedback callback, This will be called with feedback
# is received from the action server
# it just prints a message indicating a new message has been received

def feedback_callback(feedback):
    rospy.loginfo("Rec Odom Feedback feedback ==> "+str(feedback))

def count_seconds(seconds):
    for i in range(seconds):
        rospy.loginfo("Seconds passed =>"+str(i))
        time.sleep(1)

# initializes the action client node
rospy.init_node('record_odom_action_client_node')

# create the connection to the action server
client = actionlib.SimpleActionClient('/rec_odom_as',record_odomAction)

rate = rospy.Rate(1)

# waits until the action server is up and running
rospy.loginfo('Waiting for action Server')
client.wait_for_server
rospy.loginfo('Action Server Found...')

# creates a goal to send the action server
goal = record_odomGoal()

# sends the goal to the action server, specifying which feedback function
# to call when feedback received
client.send_goal(goal,feedback_cb=feedback_callback)

# simple_state will be 1 if active, and 2 when finished. It's a variable, better use a function like get_state.
# state = client.simple_state
# state_result will give the FINAL STATE. Will be 1 when Active, and 2 if No Error, 3 If Any Warning, and 3 if Error
state_result = client.get_state()
"""
class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR = 4

"""

rospy.loginfo("state_result: "+str(state_result))

while state_result < 2:
    rospy.loginfo("Waiting to finish: ")
    rate.sleep()
    state_result = client.get_state
    rospy.loginfo("state_result:"+str(state_result))

state_result = client.get_state
rospy.loginfo("[Result] State: "+str(state_result))

if state_result==4:
    rospy.logerr("Something went wrong in the Server Side")
if state_result==3:
    rospy.logwarn("There is a warning in the Server Side")

rospy.loginfo("[Result] State:"+str(client.get_result()))