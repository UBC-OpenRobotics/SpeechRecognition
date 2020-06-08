#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
from CorpusGenerator import CorpusGenerator

class ASRController():

    def __init__(self, corpGen):
        rospy.init_node('asr_controller')
        self.corpGen = corpGen
        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.voice = 'voice_don_diphone'
        self.volume = 1.0

        rospy.Subscriber('/grammar_data', String, self.parse_speech)
        rospy.spin()

    def parse_speech(self, speech_data):
        if speech_data.data.strip() in self.corpGen.listQuestions():
            answer = self.corpGen.getAnswer(speech_data.data.strip())
            rospy.loginfo('[SYNTHESIS] Matched Question')
            self.say(answer, voice=self.voice)

    def say(self, speech):
        self.soundhandle.say(speech)

if __name__ == "__main__":
    namesFile = rospy.get_param("/asr_controller/namesFile")
    objectsFile = rospy.get_param("/asr_controller/objectsFile")
    locationsFile = rospy.get_param("/asr_controller/locationsFile")
    gesturesFile = rospy.get_param("/asr_controller/gesturesFile")
    questionsFile = rospy.get_param("/asr_controller/questionsFile")
    
    corpGen = CorpusGenerator()
    corpGen.loadFiles(namesFile, objectsFile, locationsFile, gesturesFile, questionsFile)
    ASRController(corpGen)