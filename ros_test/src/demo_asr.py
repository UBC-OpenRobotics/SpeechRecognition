#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
from CorpusGenerator import CorpusGenerator

class DemoController():

    def __init__(self, corpGen):
        rospy.init_node('demo_controller')
        
        #Dataset
        self.corpGen = corpGen
        self.names = [name.upper() for name in corpGen.listNames()]
        self.categories = self.corpGen.generate_category_dict()
        self.drinks = [drink.upper() for drink in self.categories['drinks']]

        #Knowledge
        self.drink_dict={}
        self.current_name = ''

        #States
        self.ask_name = True
        self.wait_name = False
        self.ask_drink = False
        self.wait_drink = False
        self.ask_again = False

        #Speech synthesis bool
        self.talking = False

        self.timer = rospy.get_rostime().secs
        self.timeout = 10

        #Voice
        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.voice = 'voice_don_diphone'
        self.volume = 1.0
        self.parse_speech('')
        self.loop()


    def loop(self):
        rospy.Subscriber('/grammar_data', String, self.parse_speech)
        rospy.spin()


    def parse_speech(self, speech_data):

        if speech_data:
            if not self.talking:
                words = speech_data.data.strip()
            else:
                #Mute it
                words = ''

        if self.ask_name:
            self.talking = True
            rospy.loginfo("WHAT IS YOUR NAME")
            resp = 'WHAT IS YOUR NAME?'
            self.say(resp)
            
            #Set states
            self.ask_name=False
            self.wait_name=True

            #Set timer
            self.timer = rospy.get_rostime().secs
            rospy.sleep(0.5)

        elif self.wait_name:
            self.talking = False
            rospy.loginfo("WAITING FOR NAME")
            for name in self.names:
                if name in words:
                    self.talking = True
                    
                    rospy.loginfo(name)
                    self.drink_dict[name] = ''
                    self.current_name = name

                    self.wait_name=False
                    self.ask_drink=True
                    self.timer = rospy.get_rostime().secs

                    resp = 'NICE TO MEET YOU {}'.format(name)
                    self.say(resp)
                    rospy.loginfo("GOT NAME")

            if rospy.get_rostime().secs - self.timer > self.timeout:

                self.ask_again = True
                self.ask_name = True
                self.wait_name = False

        elif self.ask_drink:
            self.talking = True
            rospy.loginfo("WAITING FOR DRINK")
            resp = 'WHAT IS YOUR FAVOURITE DRINK?'
            self.say(resp)
            
            #Set states
            self.ask_drink=False
            self.wait_drink=True

            #Set timer
            self.timer = rospy.get_rostime().secs
            rospy.sleep(0.5)

        elif self.wait_drink:
            self.talking = False
            for drink in self.drinks:
                if drink in words:
                    self.talking = True

                    rospy.loginfo(drink)
                    self.drink_dict[self.current_name]= drink

                    self.wait_drink=False
                    self.timer = rospy.get_rostime().secs

                    resp = 'YOUR NAME IS {} AND YOUR FAVOURITE DRINK IS {}'.format(self.current_name, drink)
                    self.say(resp)
                    rospy.loginfo("GOT DRINK")

            if rospy.get_rostime().secs - self.timer > self.timeout:

                self.ask_again = True
                self.ask_drink = True
                self.wait_drink = False
        
        if self.ask_again:
            self.talking = True
            resp = 'I AM SORRY I DID NOT CATCH THAT'
            rospy.loginfo("DID NOT CATCH")
            self.say(resp)
            self.ask_again=False

    def say(self, speech):
        self.soundhandle.say(speech)

if __name__ == "__main__":
    namesFile = rospy.get_param("/demo_controller/namesFile")
    objectsFile = rospy.get_param("/demo_controller/objectsFile")
    locationsFile = rospy.get_param("/demo_controller/locationsFile")
    gesturesFile = rospy.get_param("/demo_controller/gesturesFile")
    questionsFile = rospy.get_param("/demo_controller/questionsFile")
    actionsFile = rospy.get_param("/demo_controller/actionsFile")
    
    corpGen = CorpusGenerator()
    corpGen.loadFiles(namesFile, objectsFile, locationsFile, gesturesFile, questionsFile, actionsFile)
    DemoController(corpGen)