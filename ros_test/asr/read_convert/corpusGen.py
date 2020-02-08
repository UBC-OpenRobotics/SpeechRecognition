#Purpose: to Convert Gestures, Locations, Names and Objects given by Robocup in xml to txt

import xml.etree.ElementTree as ET

exportFile = open("Extract","w")

namestree = ET.parse('Names.xml')
namesroot = namestree.getroot()
gesturestree = ET.parse('Gestures.xml')
gesturesroot = gesturestree.getroot()
locationstree = ET.parse('Locations.xml')
locationsroot = locationstree.getroot()
objectstree = ET.parse('Objects.xml')
objectsroot = objectstree.getroot()

# extracts names
x=names[0]
for elem in namesroot:
    names.append(elem.txt)
    print(names)

# extract gestures
exportFile.write("Gestures:\n")
for gesture in gesturesroot.findall('gesture'):
    gestureName =  gesture.get('name')
    exportFile.write("  " + gestureName + "\n")
exportFile.write("\n")

#extract locations
exportFile.write("Locations:\n")
for room in locationsroot.findall('room'):
    roomName = room.get('name')
    exportFile.write("  " + roomName+"\n")
    
    eachRoom = "./room[@name='"+ roomName +"']/location"
    for location in locationsroot.findall(eachRoom):
        placeInRoom = location.get('name')
        exportFile.write("      " + placeInRoom+":\n")
exportFile.write("\n")

#extract objects
exportFile.write("Categories:\n")
for category in objectsroot.findall('category'):
    categoryName = category.get('name')
    exportFile.write(categoryName+":\n")
    
    eachCategory = "./category[@name='"+ categoryName +"']/object"
    for object in objectsroot.findall(eachCategory):
        objectName = object.get('name')
        exportFile.write("      " + objectName+"\n")
exportFile.write("\n")
    

exportFile.close()