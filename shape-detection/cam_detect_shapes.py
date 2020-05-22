# USAGE
# python detect_shapes.py --image shapes_and_colors.png

# import the necessary packages
from pyimagesearch.shapedetector import ShapeDetector
import argparse
import imutils
import cv2
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Obj_det():
	flag = False
	subs_pose_info = Pose()
	def __init__(self):
		rospy.init_node('offboard_test', anonymous=True)
		img_coor_pub = rospy.Publisher('/our_topic', Pose, queue_size=10)
		img_sub = rospy.Subscriber('/uav_camera/image_raw_down', Image, self.callback)
		rate = rospy.Rate(10)
		rate.sleep()

		while not rospy.is_shutdown():
			if self.flag == True:
				self.subs_pose_info.position.x = 0
				bridge = CvBridge()
				image = bridge.imgmsg_to_cv2(self.camImage, desired_encoding='passthrough')
				# load the image and resize it to a smaller factor so that
				# the shapes can be approximated better
				# image = cv2.imread(self.camImage)
				resized = imutils.resize(image, width=300)
				ratio = image.shape[0] / float(resized.shape[0])

				# convert the resized image to grayscale, blur it slightly,
				# and threshold it
				gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
				blurred = cv2.GaussianBlur(gray, (5, 5), 0)
				thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
				#cv2.imshow("threshold", thresh)
				#cv2.waitKey(0)

				# find contours in the thresholded image and initialize the
				# shape detector
				cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
					cv2.CHAIN_APPROX_SIMPLE)
				cnts = imutils.grab_contours(cnts)
				# print(len(cnts))
				sd = ShapeDetector()

				# loop over the contours
				for c in cnts:
					# compute the center of the contour, then detecnt the name of the
					# shape using only the contour
					M = cv2.moments(c)
					if M["m00"] != 0:
						cX = int((M["m10"] / M["m00"]) * ratio)
						cY = int((M["m01"] / M["m00"]) * ratio)
						self.subs_pose_info.position.x = 1
					else:
						cX = 0
						cY = 0 
						self.subs_pose_info.position.x = 0
					shape = sd.detect(c)

					# multiply the contour (x, y)-coordinates by the resize ratio,
					# then draw the contours and the name of the shape on the image
					if shape=="circle":
						
						self.subs_pose_info.orientation.x = cX
						self.subs_pose_info.orientation.y = cY

						print("x:", cX, " y:",cY)
						c = c.astype("float")
						c *= ratio
						c = c.astype("int")
						cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
						cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
							0.5, (255, 255, 255), 2)

						# show the output image
						#cv2.imshow("Detected", image)
						#cv2.waitKey(0)
						break
						
				img_coor_pub.publish(self.subs_pose_info)
				rate.sleep()


	def callback(self,msg):
		#print("hi")
		self.flag = True
		self.camImage = msg

if __name__ == "__main__":
	Obj_det()