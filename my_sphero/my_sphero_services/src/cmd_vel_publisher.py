#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class CmdVelPub(object):
    def __init__(self):
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self._twist_object = Twist()
        self.linearspeed = 0.2
        self.angularspeed = 0.5
    
    def move_robot(self,direction):
        if direction == "forwards":
            self._twist_object.linear.x = self.linearspeed
            self._twist_object.angular.z = 0.0
        elif direction == "right":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = self.angularspeed
        elif direction == "left":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = -self.angularspeed
        elif direction == "backwards":
            self._twist_object.linear.x = -self.linearspeed
        elif direction == "stop":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = 0.0
        else:
            pass
        
        self._cmd_vel_pub.publish(self._twist_object)

if __name__ == "__main__":
    rospy.init_node('cmd_vel_publisher_node')
    cmd_publisher_object = CmdVelPub()

    rate = rospy.Rate(1)

    ctrl_c = False

    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        global twist_object
        global pub

        rospy.loginfo("shutdown time!")

        ctrl_c = True
        cmd_publisher_object.move_robot(direction="stop")
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        cmd_publisher_object.move_robot(direction="forwards")
        rate.sleep()