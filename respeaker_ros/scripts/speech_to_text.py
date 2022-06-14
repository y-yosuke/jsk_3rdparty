#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import division

import sys

import actionlib
import rospy
try:
    import speech_recognition as SR
except ImportError as e:
    raise ImportError(str(e) + '\nplease try "pip install speechrecognition"')

import numpy as np
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from audio_common_msgs.msg import AudioData
enable_audio_info = True
try:
    from audio_common_msgs.msg import AudioInfo
except Exception as e:
    rospy.logwarn('audio_common_msgs/AudioInfo message is not exists.'
                 ' AudioInfo message will not be published.')
    enable_audio_info = False
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class SpeechToText(object):
    def __init__(self):
        # format of input audio data
        audio_info_topic_name = rospy.get_param('~audio_info', '')
        if len(audio_info_topic_name) > 0:
            if enable_audio_info is False:
                rospy.logerr(
                    'audio_common_msgs/AudioInfo message is not exists.'
                    ' Giving ~audio_info is not valid in your environment.')
                sys.exit(1)
            rospy.loginfo('Extract audio info params from {}'.format(
                audio_info_topic_name))
            audio_info_msg = rospy.wait_for_message(
                audio_info_topic_name, AudioInfo)
            self.sample_rate = audio_info_msg.sample_rate
            self.sample_width = audio_info_msg.bitrate // self.sample_rate // 8
            self.channels = audio_info_msg.channels
        else:
            self.sample_rate = rospy.get_param("~sample_rate", 16000)
            self.sample_width = rospy.get_param("~sample_width", 2)
            self.channels = rospy.get_param("~channels", 1)
        if self.sample_width == 2:
            self.dtype = 'int16'
        elif self.sample_width == 4:
            self.dtype = 'int32'
        else:
            raise NotImplementedError('sample_width {} is not supported'
                                      .format(self.sample_width))
        self.target_channel = rospy.get_param("~target_channel", 0)
        # language of STT service
        self.language = rospy.get_param("~language", "ja-JP")
        # ignore voice input while the robot is speaking
        self.self_cancellation = rospy.get_param("~self_cancellation", True)
        # time to assume as SPEAKING after tts service is finished
        self.tts_tolerance = rospy.Duration.from_sec(
            rospy.get_param("~tts_tolerance", 1.0))
        tts_action_names = rospy.get_param(
            '~tts_action_names', ['sound_play'])

        self.recognizer = SR.Recognizer()

        self.tts_action = None
        self.last_tts = None
        self.is_canceling = False
        self.tts_actions = []
        if self.self_cancellation:
            for tts_action_name in tts_action_names:
                tts_action = actionlib.SimpleActionClient(
                    tts_action_name, SoundRequestAction)
                if tts_action.wait_for_server(rospy.Duration(5.0)):
                    self.tts_actions.append(tts_action)
                else:
                    rospy.logerr(
                        "action '{}' is not initialized."
                        .format(tts_action_name))
                self.tts_timer = rospy.Timer(rospy.Duration(0.1), self.tts_timer_cb)

        self.pub_speech = rospy.Publisher(
            "speech_to_text", SpeechRecognitionCandidates, queue_size=1)
        self.sub_audio = rospy.Subscriber("audio", AudioData, self.audio_cb)

    def tts_timer_cb(self, event):
        stamp = event.current_real
        active = False
        for tts_action in self.tts_actions:
            for st in tts_action.action_client.last_status_msg.status_list:
                if st.status == GoalStatus.ACTIVE:
                    active = True
                    break
            if active:
                break
        if active:
            if not self.is_canceling:
                rospy.logdebug("START CANCELLATION")
                self.is_canceling = True
                self.last_tts = None
        elif self.is_canceling:
            if self.last_tts is None:
                self.last_tts = stamp
            if stamp - self.last_tts > self.tts_tolerance:
                rospy.logdebug("END CANCELLATION")
                self.is_canceling = False

    def audio_cb(self, msg):
        if self.is_canceling:
            rospy.loginfo("Speech is cancelled")
            return

        data = SR.AudioData(
            np.frombuffer(msg.data, dtype=self.dtype)[
                self.target_channel::self.channels].tobytes(),
            self.sample_rate, self.sample_width)
        try:
            rospy.loginfo("Waiting for result %d" % len(data.get_raw_data()))
            result = self.recognizer.recognize_google(
                data, language=self.language)
            msg = SpeechRecognitionCandidates(
                transcript=[result],
                confidence=[1.0],
            )
            self.pub_speech.publish(msg)
        except SR.UnknownValueError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))
        except SR.RequestError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))


if __name__ == '__main__':
    rospy.init_node("speech_to_text")
    stt = SpeechToText()
    rospy.spin()
