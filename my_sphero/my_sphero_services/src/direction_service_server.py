#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from imu_topic_subscriber import ImuTopicReader
import time


class CrashDirectionService(object):
    def __init__(self, srv_name='/crash_direction_service'):
        self._srv_name = srv_name
        self._imu_reader_object = ImuTopicReader()
        self.detection_dict = {"front":False, "left":False, "right":False, "back":False}
        self._my_service = rospy.Service(self._srv_name, Trigger , self.srv_callback)

    def srv_callback(self, request):
        self.detection_dict = self._imu_reader_object.four_sector_detection()
        
        message = self.direction_to_move()
        
        rospy.logdebug("[LEFT="+str(self.detection_dict["left"])+", FRONT="+str(self.detection_dict["front"])+", RIGHT="+str(self.detection_dict["right"])+"]"+", BACK="+str(self.detection_dict["back"])+"]")
        rospy.logdebug("DIRECTION ==>"+message)
        
        response = TriggerResponse()
        """
        ---                                                                                                 
        bool success   # indicate if crashed                                       
        string message # Direction
        """
        response.success = self.has_crashed()
        response.message = message
        
        return response

    
    def has_crashed(self):
        for key, value in self.detection_dict.iteritems():
            if value:
                return True
        
        return False
    
    def direction_to_move(self):

        if not self.detection_dict["front"]:
            message = "forwards"
        
        else:
            if not self.detection_dict["back"]:
                    message = "backwards"
            else:
                if not self.detection_dict["left"]:
                    message = "left"
                else:
                    if not self.detection_dict["right"]:
                        message = "right"
                    else:
                        message = "un_stuck"

        
        return message

if __name__ == "__main__":
    rospy.init_node('crash_direction_service_server', log_level=rospy.INFO) 
    dir_serv_object = CrashDirectionService()
    rospy.spin() # mantain the service open.