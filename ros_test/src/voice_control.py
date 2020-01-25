#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class VoiceController():

    def __init__(self):

        #Create the node and set shutdown command
        rospy.init_node('voice_control')
        #rospy.on_shutdown(self.shutdown) Seems to cause some issues when trying to kill node

        #Initialze output msg
        self.msg = Twist()

        #Set speed
        self.speed_linear = 0.2
        self.speed_angular = 0.5

        #Initalize publisher
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)

        rospy.Subscriber("kws_data", String, self.parse_kws)
        rospy.spin()

    def parse_kws(self, kws_data):
        if kws_data.data.find('forward') > -1:
            self.msg.linear.x = self.speed_linear
            self.msg.angular.z = 0
        elif kws_data.data.find('back') > -1:
            self.msg.linear.x = -self.speed_linear
            self.msg.angular.z = 0
        elif kws_data.data.find('right') > -1:
            self.msg.linear.x = 0
            self.msg.angular.z = -self.speed_angular
        elif kws_data.data.find('left') > -1:
            self.msg.linear.x = 0
            self.msg.angular.z = self.speed_angular
        elif kws_data.data.find('stop') > -1:
            self.msg = Twist()
        elif kws_data.data.find('full') > -1 and kws_data.data.find('speed')> -1:
            self.speed_linear = 0.4
            self.speed_angular = 1
        elif kws_data.data.find('half') > -1 and kws_data.data.find('speed') > -1:
            self.speed_linear = 0.2
            self.speed_angular = 0.5

        self.pub.publish(self.msg)


    def shutdown(self):
        rospy.loginfo("Stopping VoiceController")
        self.pub.publish(Twist())#Publish empty twist message to stop
        rospy.sleep(1)

if __name__ == "__main__":
    VoiceController()