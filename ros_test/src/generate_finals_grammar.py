#!/usr/bin/python
from FinalsCorpusGenerator import FinalsCorpusGenerator

if __name__ == '__main__':
	namesFile = '../asr/resources_finals/Names.xml'
	foodFile = '../asr/resources_finals/Food.xml'
	fillerFile = '../asr/resources_finals/filler.txt' 

	outFile = '../asr/finals.gram'

	gramGen = FinalsCorpusGenerator()
	gramGen.loadFiles(namesFile, foodFile, fillerFile)
	gramGen.buildGrammar(outFile)