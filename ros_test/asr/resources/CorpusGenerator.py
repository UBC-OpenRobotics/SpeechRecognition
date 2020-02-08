#!/usr/bin/python
import os
import xmltodict
import xml.etree.ElementTree as ET

class Location():
    def __init__(self, room, contents):
        self.room = room
        self.contents = contents
    
    def getRoom(self):
        return self.room
    
    def getContents(self):
        return self.contents
    
    
class Object():
    
    def __init__(self, name, category, defaultLocation, room):
        self.name = name
        self.category = category
        self.defaultLocation = defaultLocation
        self.room = room
    
    def getName(self):
        return self.name
    
    def getCategory(self):
        return self.category
    
    def getLocation(self):
        return self.defaultLocation
    
    def getRoom(self):
        return self.room
    

class Gesture():
    def __init__(self, name):
        self.name = name
        
    def getName(self):
        return self.name


class Question():
    def __init__(self, question, answer):
        self.question = question
        self.answer = answer
    
    def getQuestion(self):
        return self.question
    
    def getAnswer(self):
        return self.answer

    
class Name():
    def __init__(self, name):
        self.name = name
        
    def getName(self):
        return self.name


class CorpusGenerator():
    def __init__(self):
        self.names = []
        self.objects = []
        self.locations = []
        self.gestures = []
        self.questions = []
        
    def loadFiles(self,namesFile, objectsFile, locationsFile, gesturesFile, questionsFile):
        namestree = ET.parse(namesFile)
        namesroot = namestree.getroot()

        gesturestree = ET.parse(gesturesFile)
        gesturesroot = gesturestree.getroot()

        locationstree = ET.parse(locationsFile)
        locationsroot = locationstree.getroot()

        objectstree = ET.parse(objectsFile)
        objectsroot = objectstree.getroot()

        questionsTree = ET.parse(questionsFile)
        questionsRoot = questionsTree.getroot()

        # extracts names
        for name in namesroot:
            self.names.append(Name(name.text))

        # extract gestures
        for gesture in gesturesroot.findall('gesture'):
            gestureName =  gesture.get('name')
            self.gestures.append(Gesture(gestureName))

        # extract questions
        for QA in questionsRoot.findall('question'):
            question =  QA.getchildren()[0].text
            answer =  QA.getchildren()[1].text

            self.questions.append(Question(question,answer))

        #extract locations
        #locations are composed of the name of a room and its contents
        for room in locationsroot.findall('room'):
            roomName = room.get('name')

            eachRoom = "./room[@name='"+ roomName +"']/location"
            roomContents = []
            for location in locationsroot.findall(eachRoom):
                placeInRoom = location.get('name')
                roomContents.append(placeInRoom)
            loc = Location(roomName, roomContents)
            self.locations.append(loc)


        #extract objects
        for category in objectsroot.findall('category'):
            categoryName = category.get('name')
            locationName = category.get('defaultLocation')
            roomName= category.get('room')

            eachCategory = "./category[@name='"+ categoryName +"']/object"
            for obj in objectsroot.findall(eachCategory):
                objectName = obj.get('name')

                self.objects.append(Object(objectName, categoryName, locationName, roomName))
        
    def generateCorpus(self, outputPath):
        with open(outputPath, 'w') as outFile:
            for name in self.names:
                outFile.write(name.getName()+'\n')

            for loc in self.locations:
                outFile.write(loc.getRoom()+'\n')
                for elem in loc.getContents():
                    outFile.write(elem+'\n')

            for obj in self.objects:
                outFile.write(obj.getName()+'\n')
                outFile.write(obj.getLocation()+'\n')
                outFile.write(obj.getCategory()+'\n')
                outFile.write(obj.getRoom()+'\n')

            for gesture in self.gestures:
                outFile.write(gesture.getName()+'\n')

            for question in self.questions:
                outFile.write(question.getQuestion()+'\n')
                outFile.write(question.getAnswer()+'\n')
        
    