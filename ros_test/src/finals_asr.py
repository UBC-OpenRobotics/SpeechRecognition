#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
from FinalsCorpusGenerator import FinalsCorpusGenerator
import json
import time

class FinalsController():

    def __init__(self, corpGen):
        """
        Attributes:
        corpGen -- CorpusGenerator instance that holds all vocab knowledge
        """

        rospy.init_node('finals_controller')
        ### Dataset
        # Need information on known names and foods
        self.corpGen = corpGen
        self.names = [name.upper() for name in corpGen.listNames()]
        self.food = [food.upper() for food in corpGen.listFood()]

        ### Knowledge
        #  will store Names as keys and food as values - keep track of orders
        self.orders={}
        self.current_name = ''

        ### Publisher
        self.pub = rospy.Publisher("orders", String, queue_size=5)

        ### State Machine
        ## State Booleans
        # Two main states are talking and waitingForReply
        self.talking = False
        self.waitingForReply = True

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
        self.waitRequest()
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

        rospy.loginfo("[SPEAKING] %s" % (speech))
        self.soundhandle.say(speech, blocking=True)
        rospy.loginfo("[DONE SPEAKING]")

    def waitRequest(self):
        ### WAIT FOR REQUEST ###
        rospy.loginfo("[WAITING]")

        #Keywords
        kws = ['CAN','PLACE','ORDER']

        #Accept if two or more keywords in the sentence
        order_requested = sum([kw in self.words for kw in kws]) >= 2
        while not order_requested:
            order_requested = sum([kw in self.words for kw in kws]) >= 2

        #Reset words
        self.words = ''

        #Got a request
        self.waitingForReply = False
        self.talking = True

    def mainloop(self):

        ### GET NAME AND ORDER ###
        # bundle = "CERTAINLY"
        # self.speak(bundle)
        bundle = "CERTAINLY, CAN I HAVE YOUR NAME AND ORDER PLEASE"
        self.speak(bundle)

        #Wait for reply
        self.talking = False
        self.waitingForReply = True
        #log
        rospy.loginfo("[WAITING]")

        haveName = False
        haveFood = False

        while not haveName or not haveFood:

            name_comp = [name in self.words for name in self.names]
            haveName = sum(name_comp) > 0

            rospy.loginfo('[%s NAME IN %s]' % (haveName, str(self.names)))

            ###TODO: Change this so that sum captures how many food items
            food_comp = [food in self.words for food in self.food]
            haveFood = sum(food_comp) > 0

            rospy.loginfo('[%s FOOD IN %s]' % (haveFood,str(self.food)))

        #Reset words
        self.words = ''

        #Index of name and food (Change to list later for food)
        idx_name = name_comp.index(True)
        idx_food = food_comp.index(True)

        #Get name, store key and assign food
        name = self.names[idx_name]
        self.current_name = name
        self.orders[name] = self.food[idx_food]

        ### CONFIRM NAME AND ORDER ###
        self.talking = True
        self.waitingForReply = False

        bundle = 'YOUR NAME IS %s, AND YOU WANT TO ORDER %s, IS THIS CORRECT?' %(self.current_name, self.orders[self.current_name])
        self.speak(bundle)

        # bundle = 'IS THIS CORRECT?'
        # self.speak(bundle)

        self.talking = False
        self.waitingForReply = True

        confirmed = False
        while not confirmed:
            if 'YES' in self.words:
                #Pass
                confirmed = True
            elif 'NO' in self.words:
                #Redo mainloop
                self.mainloop()

        #Reset words
        self.words = ''

        ### FACE SCAN ###
        self.talking = True
        self.waitingForReply = False

        bundle = 'PLEASE APPROACH SO THAT I MAY SCAN YOUR FACE'
        self.speak(bundle)

        rospy.loginfo("[SCANNING FACE]")

        """
        This is where the facial scanning will be ordered from
        """

        bundle = 'FACE SCAN SUCCESSFULL'
        self.speak(bundle)
        bundle = 'PLEASE WAIT FOR YOUR ORDER'
        self.speak(bundle)

        ### MOVING TO SERVER TABLE ###

        rospy.loginfo("[FINDING SERVER TABLE]")

        """
        This is where the moving to the server is ordered from

        """

        ### PLACING THE ORDER ###
        bundle = 'ORDER OF %s FOR %s' % (self.orders[self.current_name], self.current_name)
        self.speak(bundle)

        self.talking = False
        self.waitingForReply = True

        ### WAITING FOR ORDER ###
        #Keywords
        kws = ['ORDER','READY',self.current_name]

        #Accept if two or more keywords in the sentence
        order_ready = sum([kw in self.words for kw in kws]) >= 2
        while not order_ready:
            order_ready = sum([kw in self.words for kw in kws]) >= 2

        #Reset words
        self.words = ''

        ### GRABBING ORDER ###
        rospy.loginfo("[GRABBING ORDER]")

        """
        This is where arm extension and movement is ordered
        """

        ### FINDING CLIENT ###
        rospy.loginfo("[FINDING %s]" % (self.current_name))

        """
        This is where person tracking and robot movement are ordered
        """

        ### GIVING ORDER ###
        rospy.loginfo("[DEPOSITING ORDER]")

        """
        This is where the arm is extended to drop the order
        """

        ### WAIT FOR GOODBYE ###
        goodbye = 'GOODBYE' in self.words or 'BYE' in self.words
        while not goodbye:
            goodbye = 'GOODBYE' in self.words or 'BYE' in self.words

        #Reset words
        self.words = ''
        
        self.waitingForReply = False
        self.talking = True

        bundle = 'THANK YOU FOR COMING, GOODBYE'
        self.speak(bundle)

        self.talking = False

        ### ARM WAVING ###
        rospy.loginfo("[ARM WAVING]")

        """
        This is where the arm waving is ordered
        """


if __name__ == "__main__":
    namesFile = rospy.get_param("/demo_controller/namesFile")
    foodFile = rospy.get_param("/demo_controller/foodFile")
    fillerFile = rospy.get_param("/demo_controller/fillerFile")

    corpGen = FinalsCorpusGenerator()
    corpGen.loadFiles(namesFile, foodFile, fillerFile)
    FinalsController(corpGen)