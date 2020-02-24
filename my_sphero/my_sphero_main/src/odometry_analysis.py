#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import math 


class OdometryAnalysis(object):
    def __init__(self):
        pass
    
    def get_distance_moved(self, odmetry_data_list):
        
        distance = None
        
        if len(odmetry_data_list) >= 2 :
            start_odom = odmetry_data_list[0]
            end_odom = odmetry_data_list[len(odmetry_data_list)-1]
            
            start_position = start_odom.pose.pose.position
            end_position = end_odom.pose.pose.position
            
            rospy.loginfo("start_position ==>"+str(start_position))
            rospy.loginfo("end_position ==>"+str(end_position))
            
            
            distance_vector = self.create_vector(start_position, end_position)
            rospy.loginfo("Distance Vector ==>"+str(distance_vector))
            
            distance = self.calculate_legth_vector(distance_vector)
            rospy.loginfo("Distance ==>"+str(distance))
        
        else:
            rospy.logerr("Odom array doesnt have the minimum number of elements = "+str(len(odmetry_data_list)))
        
        return distance
        
    def create_vector(self, p1, p2):
        
        distance_vector = Vector3()
        distance_vector.x = p2.x - p1.x
        distance_vector.y = p2.y - p1.y
        distance_vector.z = p2.z - p1.z
        
        return distance_vector
    
    def calculate_legth_vector(self,vector):
        
        length = math.sqrt(math.pow(vector.x,2)+math.pow(vector.y,2)+math.pow(vector.z,2))
        return length


def check_if_out_maze(goal_distance, odom_result_array):
    odom_analysis_object = OdometryAnalysis()
    distance = odom_analysis_object.get_distance_moved(odom_result_array)
    rospy.loginfo("Distance Moved="+str(distance))
    # To exit we consider that each square in the floor is around 0.5m, there fore to exit correctly
    # distance has to be sqrt (6*5 + 5*4) = 7.8
    return distance > goal_distance