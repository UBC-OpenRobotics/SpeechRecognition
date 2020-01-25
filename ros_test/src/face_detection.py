#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
import numpy as np
from cv_bridge import CvBridge
import face_recognition
import tensorflow as tf
from tensorflow.keras import datasets, layers, models
import os


#define constants
fx = 1
fy = 1

#Define paths to model and dataset
model_path = "/home/francisco/Desktop/Notebooks/OpenRobotics/CNN.h5"
dataset_path = "/home/francisco/Desktop/Notebooks/OpenRobotics/dataset/"

#define width and heigth constants
width = 64
height = 64

#define threshold for unknown
thresh = 0.5

#Load CNN model
#model = tf.keras.models.load_model(model_path)


def get_one_hot(dataset_path):
    one_hot = []
    for name in os.listdir(dataset_path):
        one_hot.append(name)

    return one_hot


def detect_faces(image):
    #Load model
    model = tf.keras.models.load_model(model_path)

    #Get one_hot encodings
    one_hot = get_one_hot(dataset_path)

    #convert ros image to cv image
    bridge = CvBridge()

    frame = bridge.imgmsg_to_cv2(image)

    #convert bgr to rgb
    rgb_frame = frame[:,:,::-1]

    #detect faces
    bounding_boxes = face_recognition.face_locations(rgb_frame, model='cnn')
    #rospy.loginfo(bounding_boxes)
    if(bounding_boxes):
        for top, right, bottom, left in bounding_boxes:
            face = rgb_frame[top:bottom,left:right]
            face = cv2.resize(face,(width,height))
            face = face/255.0
            face = np.expand_dims(face,axis=0)
            prediction = model.predict(face)[0]
            #rospy.loginfo(model.predict(face)[0])
            #print(prediction)                   
            if max(prediction) < thresh:
                name = "Unknown"
            else:
                i = np.where(prediction == max(prediction))[0][0]
                name = one_hot[i]

            top = int(top/fx)
            right = int(right/fy)
            bottom = int(bottom/fx)
            left = int(left/fy)

            cv2.rectangle(frame, (left, top), (right, bottom), (0,0,255), 2)

            #Add label
            cv2.rectangle(frame, (left, bottom-35), (right, bottom), (0,0,255), cv2.FILLED)
            #Add name
            cv2.putText(frame, name + " {:.2f}%".format(max(prediction)*100), (left+6, bottom-6), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,255,255),1)

    cv2.imshow('Face Detection', frame)
    cv2.waitKey(1)



def listener():
	rospy.init_node('face_detection')

	rospy.Subscriber('/robot/camera2/image_raw', Image, detect_faces)

	rospy.spin()

if __name__ == '__main__':
	listener()
