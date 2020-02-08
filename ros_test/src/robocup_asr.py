#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ASRController():

	def __init__(self):
		rospy.init_node('asr_controller')

		rospy.Subscriber('/grammar_data', String, self.parse_speech)
		rospy.spin()

	def parse_speech(self, speech_data):
		rospy.loginfo(speech_data)

if __name__ == "__main__":
    ASRController()