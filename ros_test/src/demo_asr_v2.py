#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
from CorpusGenerator import CorpusGenerator
import json

class ReceptionistController():

    def __init__(self, corpGen):
        """
        Attributes:
        corpGen -- CorpusGenerator instance that holds all vocab knowledge
        """

        rospy.init_node('receptionist_controller')
        ### Dataset
        # Need information on known names, known drinks, known actions and gestures
        self.corpGen = corpGen
        self.names = [name.upper() for name in corpGen.listNames()]
        self.categories = self.corpGen.generate_category_dict()
        self.drinks = [drink.upper() for drink in self.categories['drinks']]
        self.actions = [action.upper() for action in self.corpGen.listActions()]

        ### Knowledge
        # drink_dict will store Names as keys and drinks as values
        self.drink_dict={}
        self.current_name = ''

        ### Publisher
        self.pub = rospy.Publisher("drink_data", String, queue_size=5)

        ### State Machine
        ## State Booleans
        # Two main states are talking and waitingForReply
        self.talking = True
        self.waitingForReply = False

        ## State information
        # Bundle contains what to say or what to wait for
        # Essentially, information passed between states
        self.bundle = ''

        ### Speech Recognition
        # words holds the latest data from the recognizer - may change under processes, may have to rethink
        self.words = ''

        ### Speech Synthesis
        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.voice = 'voice_don_diphone'
        self.volume = 1.0

        ### Time
        # timeout controls how long to wait for a reply without re-prompting
        self.timer = rospy.get_rostime().secs
        self.timeout = 10

        ### Execute
        self.startSubscriber()
        self.mainloop()

        # self.ask_name = True
        # self.wait_name = False
        # self.ask_drink = False
        # self.wait_drink = False
        # self.ask_again = False
    
    def startSubscriber(self):

        #Subscribe to grammar_data
        rospy.Subscriber('/grammar_data', String, self.storeWords)
        #rospy.spin()

    def storeWords(self, speech_data):
        #Only accept words if 
        if speech_data and self.waitingForReply:
            words = speech_data.data.strip()
            self.words = words

    def speak(self, speech):
        #Get average time to speak
        time = max(1, 0.2*(len(speech)))

        rospy.loginfo("[SPEAKING] %s" % (speech))
        self.soundhandle.say(speech)
        rospy.sleep(time)

    def mainloop(self):

        ### NAME ###
        bundle = "WHAT IS YOUR NAME?"
        self.speak(bundle)

        # Wait for reply
        self.talking = False
        self.waitingForReply = True
        #log
        rospy.loginfo("[WAITING]")

        # tic = rospy.get_rostime()
        comp = [name in self.words for name in self.names]
        haveName = sum(comp) > 0
        # toc = tic
        while not haveName:
            comp = [name in self.words for name in self.names]
            haveName = sum(comp) > 0
            #toc = rospy.get_rostime()

        #Got a name
        self.waitingForReply = False

        #Index of name
        idx = comp.index(True)

        #Get name, store key and assign empty value
        name = self.names[idx]
        self.current_name = name
        self.drink_dict[name] = ''

        #Publish
        self.pub.publish(json.dumps(self.drink_dict))

        #Reply
        self.talking = True
        bundle = 'NICE TO MEET YOU, %s' %(name)
        self.speak(bundle)

        ### DRINK ###
        bundle = "WHAT IS YOUR FAVOURITE DRINK?"
        self.speak(bundle)

        # Wait for reply
        self.talking = False
        self.waitingForReply = True
        #log
        rospy.loginfo("[WAITING]")

        # tic = rospy.get_rostime()
        comp = [drink in self.words for drink in self.drinks]
        haveDrink = sum(comp) > 0
        # toc = tic
        while not haveDrink:
            comp = [drink in self.words for drink in self.drinks]
            haveDrink = sum(comp) > 0
            #toc = rospy.get_rostime()

        #Got a drink
        self.waitingForReply = False

        #Index of drink
        idx = comp.index(True)

        #Get drink, store value
        drink = self.drinks[idx]
        self.drink_dict[self.current_name] = drink

        #Publish
        self.pub.publish(json.dumps(self.drink_dict))
        
        #Reply
        self.talking = True
        bundle = 'YOUR NAME IS %s, AND YOU LIKE %s' %(self.current_name, self.drink_dict[self.current_name])
        self.speak(bundle)



if __name__ == "__main__":
    namesFile = rospy.get_param("/demo_controller/namesFile")
    objectsFile = rospy.get_param("/demo_controller/objectsFile")
    locationsFile = rospy.get_param("/demo_controller/locationsFile")
    gesturesFile = rospy.get_param("/demo_controller/gesturesFile")
    questionsFile = rospy.get_param("/demo_controller/questionsFile")
    actionsFile = rospy.get_param("/demo_controller/actionsFile")
    
    corpGen = CorpusGenerator()
    corpGen.loadFiles(namesFile, objectsFile, locationsFile, gesturesFile, questionsFile, actionsFile)
    ReceptionistController(corpGen)











