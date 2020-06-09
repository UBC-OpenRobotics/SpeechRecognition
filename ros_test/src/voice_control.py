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

        self.currentWord = ''
        self.previousWord = ''
        rospy.Subscriber("kws_data", String, self.processWords)
        #rospy.spin()

        self.parse_kws()

    def processWords(self, kws_data):
        
        self.previousWord = self.currentWord
        self.currentWord = kws_data
        rospy.loginfo('[DATA] current:%s \nprevious:%s'%(self.currentWord, self.previousWord))


    def parse_kws(self):
        while True and self.currentWord:
            if self.previousWord.data.find('move') > -1:
                if self.currentWord.data.find('forward') > -1:
                    self.msg.linear.x = self.speed_linear
                    self.msg.angular.z = 0
                elif self.currentWord.data.find('back') > -1:
                    self.msg.linear.x = -self.speed_linear
                    self.msg.angular.z = 0
            elif self.previousWord.data.find('turn') > -1:
                if self.currentWord.data.find('right') > -1:
                    self.msg.linear.x = 0
                    self.msg.angular.z = -self.speed_angular
                elif self.currentWord.data.find('left') > -1:
                    self.msg.linear.x = 0
                    self.msg.angular.z = self.speed_angular
            elif self.currentWord.data.find('stop') > -1:
                self.msg = Twist()
            elif self.previousWord.data.find('speed') >-1 or self.currentWord.data.find('speed') >-1:
                if self.previousWord.data.find('full') > -1 or self.previousWord.data.find('max') > -1:
                    self.speed_linear = 0.4
                    self.speed_angular = 1
                elif self.previousWord.data.find('half')>-1:
                    self.speed_linear = 0.2
                    self.speed_angular = 0.5
                elif self.previousWord.data.find('min')>-1:
                    self.speed_linear = 0.1
                    self.speed_angular = 0.1
                elif self.currentWord.data.find('down')>-1:
                    self.speed_linear = max(0.1, self.speed_linear-0.1)
                    self.speed_angular = max(0.1, self.speed_angular-0.2)
                elif self.currentWord.data.find('up')>-1:
                    self.speed_linear = min(0.4, self.speed_linear+0.1)
                    self.speed_angular = min(1, self.speed_angular+0.2)

            self.pub.publish(self.msg)


    def shutdown(self):
        rospy.loginfo("Stopping VoiceController")
        self.pub.publish(Twist())#Publish empty twist message to stop
        rospy.sleep(1)

if __name__ == "__main__":
    VoiceController()