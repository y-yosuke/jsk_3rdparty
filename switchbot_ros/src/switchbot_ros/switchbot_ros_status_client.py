import rospy
import actionlib
import json
from switchbot_ros.msg import SwitchBotStatusAction
from switchbot_ros.msg import SwitchBotStatusGoal
from switchbot_ros.msg import DeviceArray


class SwitchBotROSStatusClient(object):

    def __init__(self,
                 actionname='switchbot_ros_status/switch',
                 topicname='switchbot_ros_status/devices'):

        self.actionname = actionname
        self.topicname = topicname
        self.action_client = actionlib.SimpleActionClient(
                actionname,
                SwitchBotStatusAction
                )
        rospy.loginfo("Waiting for status action server to start.")
        self.action_client.wait_for_server()


    def get_devices(self, timeout=None):

        return rospy.wait_for_message(
                self.topicname,
                DeviceArray,
                timeout=timeout
                )


    def device_status(self,
                       device_name,
                       wait=True
                       ):
        
        goal = SwitchBotStatusGoal()
        goal.device_name = device_name
        goal.command = ''
        goal.parameter = ''
        goal.command_type = ''
        self.action_client.send_goal(goal)
        if wait:
            self.action_client.wait_for_result()
            result = self.action_client.get_result()
            try:
                status = json.loads(result.status_body)
            except json.JSONDecodeError as e:
                rospy.logwarn('Data: "' + result.status_body
                                + '" -> JSON Decode Error: ' + str(e))
                return False
            return status
