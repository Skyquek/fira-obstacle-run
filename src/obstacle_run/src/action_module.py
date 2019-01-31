#! /usr/bin/env python

import json
import os
from time import sleep

import roslib.packages as rospkg
import rospy
from std_msgs.msg import Int32

from op3_action_module_msgs.srv import IsRunning

from op3_module import OP3Module

class ActionModule(OP3Module):

    def __init__(self):
        super(ActionModule, self).__init__('action_module')

        # Initialize OP3 ROS publishers and service clients
        self.motion_index_pub = rospy.Publisher('/robotis/action/page_num', Int32, queue_size=0)
        self.is_running_client = rospy.ServiceProxy('/robotis/action/is_running', IsRunning)

        # Load action configuration
        self.actions = None
        self.load_configurations()

    def load_configurations(self):
        config_file = os.path.join(rospkg.get_pkg_dir('obstacle_run'), 'config', 'action_configurations.json')
        
        with open(config_file, 'r') as file:
            self.actions = json.load(file)

    def play_action(self, action_index):
        if action_index in list(self.actions.values()):
            rospy.loginfo('Playing action {0}'.format(action_index))

            message = Int32()
            message.data = action_index

            self.motion_index_pub.publish(message)

            while self.__action_is_running():
                sleep(1)
        else:
            rospy.logerr('Skip playing action {0}.'.format(action_index))
            rospy.logwarn('Action index {0} not found in action_configurations.json.'.format(action_index))
            
    def play_crawl(self):
        self.play_action(151)
        [self.play_action(152) for _ in range(7)]
        self.play_action(153)
    
    def __action_is_running(self):
        service_name = '/robotis/action/is_running'

        rospy.wait_for_service(service_name)

        try:
            return self.is_running_client().is_running
        except rospy.ServiceException:
            rospy.logerr('Failed to call service: {0}'.format(service_name))
