#!/usr/bin/env python

import actionlib
import os.path
import json
from requests import ConnectionError
import rospy
from switchbot_ros.msg import SwitchBotStatusAction
from switchbot_ros.msg import SwitchBotStatusFeedback
from switchbot_ros.msg import SwitchBotStatusResult
from switchbot_ros.msg import Device
from switchbot_ros.msg import DeviceArray
from switchbot_ros.switchbot import SwitchBotAPIClient
from switchbot_ros.switchbot import DeviceError, SwitchBotAPIError

from switchbot_ros_server import SwitchBotAction


class SwitchBotDeviceStatusAction(SwitchBotAction):
    """
    Get your switchbot status with ROS and SwitchBot API
    """
    def __init__(self):
        # SwitchBot configs
        # '~token' can be file path or raw characters
        token = rospy.get_param('~token')
        if os.path.exists(token):
            with open(token) as f:
                self.token = f.read().replace('\n', '')
        else:
            self.token = token
        
        # Switchbot API v1.1 needs secret key
        secret = rospy.get_param('~secret', None )
        if secret is not None and os.path.exists(secret):
            with open(secret, 'r', encoding='utf-8') as f:
                self.secret = f.read().replace('\n', '')
        else:
            self.secret = secret
        
        # Initialize switchbot client
        self.bots = self.get_switchbot_client()
        self.print_apiversion()
        self.print_devices()
        self.print_scenes()
        # Actionlib
        self._as = actionlib.SimpleActionServer(
            '~switch', SwitchBotStatusAction,
            execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        # Topic
        self.pub = rospy.Publisher('~devices', DeviceArray, queue_size=1, latch=True)
        self.published = False
        rospy.loginfo('SwitchBot Status Action Server: Ready.')


    def execute_cb(self, goal):
        feedback = SwitchBotStatusFeedback()
        result   = SwitchBotStatusResult()
        r = rospy.Rate(1)
        success = True
        # start executing the action
        try:
            if not self.bots:
                self.bots = SwitchBotAPIClient(token=self.token, secret=self.secret)
            response = self.bots.device_status(device_name=goal.device_name)
            if type(response) == dict:
                feedback.status = json.dumps(response)
            else:
                feedback.status = str(response)
        except (DeviceError, SwitchBotAPIError, KeyError) as e:
            rospy.logerr(str(e))
            feedback.status = str(e)
            success = False
        finally:
            self._as.publish_feedback(feedback)
            r.sleep()
            result.done = success
            result.status_body = feedback.status
            self._as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('switchbot_ros_status')
    server = SwitchBotDeviceStatusAction()
    server.spin()
