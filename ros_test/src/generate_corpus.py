#!/usr/bin/python
from CorpusGenerator import CorpusGenerator

if __name__=='__main__':
	
	namesFile = '../asr/resources/Names.xml'
	objectsFile = '../asr/resources/Objects.xml'
	locationsFile = '../asr/resources/Locations.xml' 
	gesturesFile = '../asr/resources/Gestures.xml'
	questionsFile = '../asr/resources/Questions.xml'
	actionsFile = '../asr/resources/Actions.xml'

	outFile = '../asr/robocup.corpus'

	corpGen = CorpusGenerator()
	corpGen.loadFiles(namesFile, objectsFile, locationsFile, gesturesFile, questionsFile, actionsFile)
	corpGen.generateCorpus(outFile)
