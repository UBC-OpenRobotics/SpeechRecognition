#!/usr/bin/python
import os
import xmltodict
import xml.etree.ElementTree as ET
    
class Name():
    def __init__(self, name):
        self.name = name
        
    def getName(self):
        return self.name

    def __str__(self):
        return "name"

    def __repr__(self):
        return "name"

class Food():
    def __init__(self, food):
        self.food = food

    def getName(self):
        return self.food 

    def __str__(self):
        return "food"

    def __repr__(self):
        return "food"



class FinalsCorpusGenerator():
    def __init__(self):
        self.names = []
        self.food = []
        self.filler = []
        
    def loadFiles(self,namesFile, foodFile, fillerFile):
        namestree = ET.parse(namesFile)
        namesroot = namestree.getroot()

        foodTree = ET.parse(foodFile)
        foodRoot = foodTree.getroot()

        # extracts names
        for name in namesroot:
            self.names.append(Name(name.text))

        # extracts food
        for food in foodRoot:
            self.food.append(Food(food.text))

        #Extract filler
        with open(fillerFile,'r') as inFile:
            for line in inFile.readlines():
                self.filler.append(line.strip())
    
    def generateCorpus(self, outputPath):
        with open(outputPath, 'w') as outFile:
            for name in self.names:
                outFile.write(name.getName()+'\n')

            for food in self.food:
                outFile.write(food.getName()+'\n')

            for filler in self.filler:
                outFile.write(filler+'\n')

    def buildGrammar(self, grammarFile):
        with open(grammarFile, 'w') as outFile:

            ### Header
            outFile.write("#JSGF V1.0;\n/**\n* JSGF Grammar\n*/\ngrammar finals;\n\n")
            
            ### Write alternatives
            for category in [self.names, self.food]:
                outFile.write('<%s> = ' % str(category[0]).split('.')[0])
                for elem in category[:-1]:
                    outFile.write(elem.getName().upper()+' | ')
                outFile.write(category[-1].getName().upper()+'\n\n')

            ### GRAMMAR RULES ###
            outFile.write('public <ask> = (CAN I | I WANT TO) PLACE AN ORDER;\n')

            outFile.write('public <order> = [(MY NAME IS | I AM)] <name> [AND] [(AND I WOULD LIKE | I WANT)] <food>;\n')

            outFile.write('public <confirm> = (YES | NO);\n')

            outFile.write('public <order_ready> = ORDER [(READY | DONE)] FOR <name>;\n')

            outFile.write('public <leaving> = (GOODBYE | BYE);\n')



    def listFood(self):
        food_arr = [food.getName() for food in self.food]
        return food_arr

    def listNames(self):
        names = [name.getName() for name in self.names]
        return names





