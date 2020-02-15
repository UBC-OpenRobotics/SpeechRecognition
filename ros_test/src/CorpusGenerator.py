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

    def __str__(self):
        return "location"

    def __repr__(self):
        return "location"
    
    
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

    def __str__(self):
        return "object"

    def __repr__(self):
        return "object"
    

class Gesture():
    def __init__(self, name):
        self.name = name
        
    def getName(self):
        return self.name

    def __str__(self):
        return "gesture"

    def __repr__(self):
        return "gesture"


class Question():
    def __init__(self, question, answer):
        self.question = question
        self.answer = answer
    
    def getQuestion(self):
        return self.question
    
    def getAnswer(self):
        return self.answer

    def __str__(self):
        return "question"

    def __repr__(self):
        return "question"

    
class Name():
    def __init__(self, name):
        self.name = name
        
    def getName(self):
        return self.name

    def __str__(self):
        return "name"

    def __repr__(self):
        return "name"



class CorpusGenerator():
    def __init__(self):
        self.names = []
        self.objects = []
        self.locations = []
        self.gestures = []
        self.questions = []

        #Hardcoded for now
        self._greetings = [""]
        
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
            self.questions_str.append(question)

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

    def buildGrammar(self, grammarFile):
        with open(grammarFile, 'w') as outFile:

            ### Header
            outFile.write("#JSGF V1.0;\n/**\n* JSGF Grammar\n*/\ngrammar robocup;\n\n")
            
            ### Write alternatives
            for category in [self.names, self.objects, self.gestures]:
                outFile.write('<%s> = ' % str(category[0]).split('.')[0])
                for elem in category[:-1]:
                    outFile.write(elem.getName().upper()+' | ')
                outFile.write(category[-1].getName().upper()+'\n\n')


            #build list of unique rooms
            unique_rooms = []
            for category in [self.objects, self.locations]:
                for elem in category:
                    if elem.getRoom() not in unique_rooms:
                        unique_rooms.append(elem.getRoom())

            outFile.write('<room> = ')
            for room in unique_rooms[:-1]:
              outFile.write(room.upper()+' | ')
            outFile.write(unique_rooms[-1].upper()+'\n\n')

            #Questions are public
            outFile.write('public <question> = ')
            for question in self.questions[:-1]:
                outFile.write(question.getQuestion().upper()+' | ')
            outFile.write(self.questions[-1].getQuestion().upper()+'{QUESTION}\n\n')

    def listQuestions(self):
        q_arr = [question.getQuestion() for question in self.questions]
        return q_arr

    def getAnswer(self, question_str):
        q_arr = self.listQuestions()
        idx = q_arr.index(question_str)
        q = self.questions[idx]
        return q.getAnswer()





