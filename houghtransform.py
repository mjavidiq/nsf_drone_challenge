# USAGE
# python detect_shapes.py --image shapes_and_colors.png

# import the necessary packages
from pyimagesearch.shapedetector import ShapeDetector
import argparse
import imutils
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class Obj_det():
	flag = False
	def __init__(self):
		rospy.init_node('offboard_test', anonymous=True)
		img_sub = rospy.Subscriber('/uav_camera/image_raw_down', Image, self.callback)
		rate = rospy.Rate(10)
		rate.sleep()

		while not rospy.is_shutdown():
			if self.flag == True:
				
				bridge = CvBridge()
				image = bridge.imgmsg_to_cv2(self.camImage, desired_encoding='passthrough')
				

				
				resized = imutils.resize(image, width=300)
				ratio = image.shape[0] / float(resized.shape[0])
				cimg = resized.copy()

				# convert the resized image to grayscale, blur it slightly,
				# and threshold it
				# convert BGR to RGB to be suitable for showing using matplotlib library
				#resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
				gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
				#print(gray.shape)
				##########
				#Sam's code 
				###########
				# apply a blur using the median filter
				img = cv2.medianBlur(gray, 5)
                
                		#img = cv2.GaussianBlur(gray, (5,5),0)
                		print(img)
                		#cv2.imshow('blur', img)
                
				
				# detect circles in the image using hough lines techniqu
				circles = cv2.HoughCircles(image=img, method=cv2.HOUGH_GRADIENT, dp=0.9, 
				            minDist=80, param1=50, param2=30, maxRadius=0)
				#print(circles.shape)
				#print(len(circles[0, :]))
				for co, i in enumerate(circles[0, :], start=1):
				    # draw the outer circle in green
				    cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
				    # draw the center of the circle in red
				    cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

				
			

				# show the output image
				cv2.imshow("Image", cimg)
				cv2.waitKey(0)

	def callback(self,msg):
		#print("hi")
		self.flag = True
		self.camImage = msg

if __name__ == "__main__":
	Obj_det()
