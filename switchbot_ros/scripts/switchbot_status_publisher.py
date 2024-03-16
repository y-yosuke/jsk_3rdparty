#!/usr/bin/env python

import os.path
from requests import ConnectionError

import rospy
from switchbot_ros.msg import Meter, PlugMini, Hub2, Bot, StripLight
from switchbot_ros.switchbot_ros_client import SwitchBotROSClient


class SwitchBotStatusPublisher:
    """
    Publish your switchbot status with ROS and SwitchBot API
    """
    def __init__(self):
        # Initialize SwitchBot ROS Client
        self.client = SwitchBotROSClient()
        
        # Get parameters for publishing
        self.rate = rospy.get_param('~rate', 0.1)
        rospy.loginfo('Rate: ' + str(self.rate))
        
        device_name = rospy.get_param('~device_name')
        if device_name:
            self.device_name = device_name
        else:
            rospy.logerr('No Device Name')
            return
        
        # Set device type
        self.device_type = None
        self.device_list = self.client.get_devices()
        for device in self.device_list.devices:
            device_name = device.name
            if self.device_name == device_name:
                self.device_type = device.type
        
        if self.device_type:
            rospy.loginfo('deviceName: ' + self.device_name + ' / deviceType: ' + self.device_type)
        else:
            rospy.logerr('Invalid Device Name: ' + self.device_name)
            return
        
        topic_name = '~' + self.device_name
        topic_name = topic_name.replace('-', '_')
        
        # Publisher Message Class for each device type
        if self.device_type == 'Remote':
            rospy.logerr('Device Type: "' + self.device_type + '" -> No status in SwitchBot API.')
            return
        else:
            if self.device_type == 'Meter':
                self.msg_class = Meter
            elif self.device_type == 'MeterPlus':
                self.msg_class = Meter
            elif self.device_type == 'WoIOSensor':
                self.msg_class = Meter
            elif self.device_type == 'Hub 2':
                self.msg_class = Hub2
            elif self.device_type == 'Plug Mini (JP)':
                self.msg_class = PlugMini
            elif self.device_type == 'Plug Mini (US)':
                self.msg_class = PlugMini
            elif self.device_type == 'Bot':
                self.msg_class = Bot
            elif self.device_type == 'Strip Light':
                self.msg_class = StripLight
            else:
                rospy.logerr('No publisher process for "' + self.device_type + '" in switchbot_state_publisher.py')
                return
            
            self.status_pub = rospy.Publisher(topic_name, self.msg_class, queue_size=1, latch=True)
        
        rospy.loginfo('Ready: SwitchBot Status Publisher for ' + self.device_name)


    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
            
            if self.device_type == 'Remote':
                return
            else:
                status = self.client.device_status(device_name=self.device_name)
                
                if status:
                    time = rospy.get_rostime()
                    if self.msg_class == Meter:
                        msg = Meter()
                        msg.header.stamp = time
                        msg.temperature  = status['temperature']
                        msg.humidity     = status['humidity']
                        msg.battery      = status['battery']
                    elif self.msg_class == Hub2:
                        msg = Hub2()
                        msg.header.stamp = time
                        msg.temperature  = status['temperature']
                        msg.humidity     = status['humidity']
                        msg.light_level  = status['lightLevel']
                    elif self.msg_class == PlugMini:
                        msg = PlugMini()
                        msg.header.stamp = time
                        msg.voltage      = status['voltage']
                        msg.weight       = status['weight']
                        msg.current      = status['electricCurrent']
                        msg.minutes_day  = status['electricityOfDay']
                    elif self.msg_class == Bot:
                        msg = Bot()
                        msg.header.stamp = time
                        msg.battery      = status['battery']
                        msg.power        = status['power']
                        msg.device_mode  = status['deviceMode']
                    elif self.msg_class == StripLight:
                        msg = StripLight()
                        msg.header.stamp = time
                        msg.power        = status['power']
                        msg.color        = status['color']
                        msg.brightness   = status['brightness']
                    
                    if msg:
                        self.status_pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('switchbot_status_publisher')
        ssp = SwitchBotStatusPublisher()
        ssp.spin()
    except rospy.ROSInterruptException:
        pass
