#!/usr/bin/python
from CorpusGenerator import CorpusGenerator

if __name__ == '__main__':
	namesFile = 'Names.xml'
	objectsFile = 'Objects.xml'
	locationsFile = 'Locations.xml' 
	gesturesFile = 'Gestures.xml'
	questionsFile = 'Questions.xml'

	outFile = '../robocup.gram'

	gramGen = CorpusGenerator()
	gramGen.loadFiles(namesFile, objectsFile, locationsFile, gesturesFile, questionsFile)
	gramGen.buildGrammar(outFile)