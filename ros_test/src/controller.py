#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
import numpy as np
from cv_bridge import CvBridge

#Define constants
base_linear_speed = 0.3


pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

def line_follow(image):

	#Convert ros image to cv image
	bridge = CvBridge()
	frame = bridge.imgmsg_to_cv2(image)

	rows = [200, 215, 230]

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(gray[190:],90,255,cv2.THRESH_BINARY_INV)

	#Center of frame
	frame_width = gray.shape[1]
	x_center = frame_width/2.0

	try:
		M = cv2.moments(thresh)
		x = int(M["m10"] / M["m00"])
		y = int(M["m01"] / M["m00"])
	except Exception as e:
		pass

	cv2.circle(frame, (x, y+190), 5, (255, 255, 255), -1)
	cv2.putText(frame, "centroid", (x - 25, y +170),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

	for row in rows:
		lane = np.where(thresh[row-190]==255)

		try:
			x1 = lane[0][0]
			x2 = lane[0][-1]
		except Exception as e:
			pass

		cv2.circle(frame, (x1, row), 5, (255, 255, 255), -1)
		cv2.circle(frame, (x2, row), 5, (255, 255, 255), -1)

	try:
		cv2.imshow("Image",frame)
		cv2.waitKey(1)

		err = (x-x_center)
		rospy.loginfo("Error: " + str(err/10))
		pid(err)
	except Exception as e:
		pass

def pid(dx):

	move = Twist()

	move.linear.x= base_linear_speed - abs(dx/100)

	move.angular.z = -(float(dx)/10)

	pub.publish(move)



def listener():
	rospy.init_node('listener')

	rospy.Subscriber('/robot/camera/image_raw', Image, line_follow)

	rospy.spin()

def move_robot():
	rospy.init_node('topic_publisher')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	rate = rospy.Rate(2)
	move=Twist()
	move.linear.x = 0.5
	move.angular.z = 0.5

	while not rospy.is_shutdown():
		pub.publish(move)
		rate.sleep()

if __name__ == '__main__':
	listener()