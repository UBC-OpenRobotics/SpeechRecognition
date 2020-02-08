#!/usr/bin/python
from CorpusGenerator import CorpusGenerator

if __name__=='__main__':
	
	namesFile = 'Names.xml'
	objectsFile = 'Objects.xml'
	locationsFile = 'Locations.xml' 
	gesturesFile = 'Gestures.xml'
	questionsFile = 'Questions.xml'

	outFile = '../robocup.corpus'

	corpGen = CorpusGenerator()
	corpGen.loadFiles(namesFile, objectsFile, locationsFile, gesturesFile, questionsFile)
	corpGen.generateCorpus(outFile)
