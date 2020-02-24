#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu


class ImuTopicReader(object):
    def __init__(self, topic_name = '/sphero/imu/data3'):
        self._topic_name = topic_name
        self._sub = rospy.Subscriber(self._topic_name, Imu, self.topic_callback)
        self._imudata = Imu()
        self._threshhold = 7.00
    def topic_callback(self, msg):
        self._imudata = msg
        rospy.logdebug(self._imudata)
    
    def get_imudata(self):
        """
        Returns the newest imu data

        std_msgs/Header header                                                                                                          
          uint32 seq                                                                                                                    
          time stamp                                                                                                                    
          string frame_id                                                                                                               
        geometry_msgs/Quaternion orientation                                                                                            
          float64 x                                                                                                                     
          float64 y                                                                                                                     
          float64 z                                                                                                                     
          float64 w                                                                                                                     
        float64[9] orientation_covariance                                                                                               
        geometry_msgs/Vector3 angular_velocity                                                                                          
          float64 x                                                                                                                     
          float64 y                                                                                                                     
          float64 z                                                                                                                     
        float64[9] angular_velocity_covariance                                                                                          
        geometry_msgs/Vector3 linear_acceleration                                                                                       
          float64 x                                                                                                                     
          float64 y                                                                                                                     
          float64 z                                                                                                                     
        float64[9] linear_acceleration_covariance                                                                                                              
        
        """
        return self._imudata
    
    def four_sector_detection(self):
        """
        Detects in which four directions there is an obstacle that made the robot crash
        Based on the imu data
        Axis:
         ^y
         |
        zO-->x
        
        """
        x_accel = self._imudata.linear_acceleration.x
        y_accel = self._imudata.linear_acceleration.y
        z_accel = self._imudata.linear_acceleration.z
        
        
        axis_list = [x_accel, y_accel, z_accel]
        
        max_axis_index = axis_list.index(max(axis_list))
        positive = axis_list[max_axis_index] >= 0
        significative_value = abs(axis_list[max_axis_index]) > self._threshhold
        
        
        if significative_value:
            if max_axis_index == 0:
                # Winner is in the x axis, therefore its a side crash left/right
                rospy.logwarn("[X="+str(x_accel))
                rospy.loginfo("Y="+str(y_accel)+", Z="+str(z_accel)+"]")
                if positive:
                    message = "right"
                else:
                    message = "left"
            
            elif max_axis_index == 1:
                # Winner is the Y axis, therefore its a forn/back crash
                rospy.logwarn("[Y="+str(y_accel))
                rospy.loginfo("X="+str(x_accel)+", Z="+str(z_accel)+"]")
                if positive:
                    message = "front"
                else:
                    message = "back"
            elif max_axis_index == 2:
                # Z Axixs is the winner, therefore its a crash that made it jump
                rospy.logwarn("[Z="+str(z_accel))
                rospy.loginfo("X="+str(x_accel)+", Y="+str(y_accel)+"]")
                
                if positive:
                    message = "up"
                else:
                    message = "down"
            else:
                message = "unknown_direction"
        else:
            rospy.loginfo("X="+str(x_accel)+"Y="+str(y_accel)+", Z="+str(z_accel)+"]")
            message = "nothing"
        
        return self.convert_to_dict(message)
        
    def convert_to_dict(self, message):
        """
        Converts the fiven message to a dictionary telling in which direction there is a detection
        """
        detect_dict = {}
        # We consider that when there is a big Z axis component there has been a very big front crash
        detection_dict = {"front":(message=="front" or message=="up" or message=="down"),
                          "left":message=="left",
                          "right":message=="right",
                          "back":message=="back"}
        return detection_dict
        
    
if __name__ == "__main__":
    rospy.init_node('imu_topic_subscriber', log_level=rospy.INFO)
    imu_reader_object = ImuTopicReader()
    rospy.loginfo(imu_reader_object.get_imudata())
    rate = rospy.Rate(0.5)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        data = imu_reader_object.get_imudata()
        rospy.loginfo(data)
        rate.sleep()