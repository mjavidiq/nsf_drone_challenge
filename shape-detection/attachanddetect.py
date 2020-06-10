from pyimagesearch.shapedetector import ShapeDetector
import argparse
import imutils
import cv2
import rospy
from geometry_msgs.msg import Pose,PoseStamped
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import numpy as np
import math
from std_msgs.msg import String
from mavros_msgs.msg import *
from mavros_msgs.srv import *
global mode
global rovermoved
mode = None
rovermoved=False
rospy.init_node('offboard_test', anonymous=True)


######## Class for Object detection#########
class Obj_det():
	print "Import complete"
		
	
	def __init__(self):
		print("inside init")
		
		self.img_coor_pub = rospy.Publisher('/our_topic', Pose, queue_size=10)
		self.img_sub = rospy.Subscriber('/uav_camera_down/image_raw', Image, self.callback)
		self.rate = rospy.Rate(10)
		self.rate.sleep()
		self.flag=False
		self.subs_pose_info = Pose()
	def action(self):
		#print("inside action detect")
		if self.flag == True:
			print("inside here")
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
			thresh = cv2.threshold(blurred, 220, 255, cv2.THRESH_BINARY)[1]
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
					cX = 320
					cY = 240 
					self.subs_pose_info.position.x = 1
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
					
			self.img_coor_pub.publish(self.subs_pose_info)
			self.rate.sleep()


	def callback(self,msg):
		#print("hi")
		self.flag = True
		self.camImage = msg
		

##########Class for Gazebo Probe link pose for attaching################

class GazeboLinkPose:

  def __init__(self, link_name):
    self.link_name = link_name
    self.link_pose = PoseStamped()

    if not self.link_name:
      raise ValueError("'link_name' is an empty string")

    self.states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

  def callback(self, data):
    try:
      ind = data.name.index(self.link_name)
      self.link_pose.pose = data.pose[ind]
      self.link_pose.header.frame_id = "/base_link"
      self.link_pose.header.stamp = rospy.Time.now()
      #print "inside gazebo link pose"
    except ValueError:
      pass
    #print "callback done"


##### class for attaching both drone and link################

class attach:
	
	print (mode)
	#iris=1
	#rospy.init_node('demo_attach_links')
	iris = GazeboLinkPose('iris_1')
	probe = GazeboLinkPose('sample_probe')
	rate = rospy.Rate(10)
	def init(self):
    
		#self.rospy.init_node('demo_attach_links')
		#self.iris = GazeboLinkPose('iris_1')
		#self.probe = GazeboLinkPose('sample_probe')
		self.mag = 1
		self.degrees = 0
		self.rate.sleep()
		
		
	def action(self):
		global mode
		#mode="ATTACH"
		#print mode

		if mode == "ATTACH":
			print("inside attach")
			vector_iris = np.array([
		        self.iris.link_pose.pose.position.x-self.probe.link_pose.pose.position.x,
		        self.iris.link_pose.pose.position.y-self.probe.link_pose.pose.position.y,
		        self.iris.link_pose.pose.position.z-self.probe.link_pose.pose.position.z
		        ]) 
			self.mag = np.sqrt(vector_iris.dot(vector_iris))
			self.degrees = np.degrees(np.arccos(np.clip(vector_iris[2] / self.mag, -1.0, 1.0)))
			# The vector here is the position of iris from probe
			# mag shows the magnitude of the vector
			# degrees shows the angle the vector makes with z axis
			# Print below to see the magnitude and degrees
			# print("{} {}".format(mag, np.degrees(np.arccos(np.clip(vector_iris[2] / mag, -1.0, 1.0)))))

			# 15 degree threshold and magnitude is less than 0.6 meters, attach works
			print self.mag , self.degrees
			if self.mag < 1.0 and self.degrees < 30:
				rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
				attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
				                                Attach)
				attach_srv.wait_for_service()
				rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

				# Link them
				rospy.loginfo("Attach drone to sample probe")
				req = AttachRequest()
				req.model_name_1 = "iris_1"
				req.link_name_1 = "base_link"
				req.model_name_2 = "sample_probe"
				req.link_name_2 = "base_link"
				attach_srv.call(req)
			mode = None
			self.rate.sleep()
		elif mode == "DETACH":
		    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
		    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
		                                    Attach)
		    attach_srv.wait_for_service()
		    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

		    # Link them
		    rospy.loginfo("Detaching iris and sample_probe")
		    req = AttachRequest()
		    req.model_name_1 = "iris_1"
		    req.link_name_1 = "base_link"
		    req.model_name_2 = "sample_probe"
		    req.link_name_2 = "base_link"
		    attach_srv.call(req)
		    mode = None
		    self.rate.sleep()
		#self.rate.sleep()

def set_mode(msg):
    global mode
    print "mode set"
    mode = str(msg.data)
    print mode,"set_mode"

def rover_state(msg):
	global rovermoved
	print msg.mode
	if(msg.mode=='OFFBOARD'):
		rovermoved = True
		print "rovermoved"
def roversetArm():
    rospy.wait_for_service('/uav0/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/uav0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
        #rate.sleep()
        #print "ARMING"
    except rospy.ServiceException, e:
        print "Service arming call failed: %s"%e
def rover_set_mode():
    rospy.wait_for_service('/uav0/mavros/set_mode')
    try:
        # setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.set_mode.request.custom_mode)
        setModeService = rospy.ServiceProxy('/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
        setModeService(custom_mode="OFFBOARD")
        #print "OFFBOARD"
    except rospy.ServiceException, e:
        print "Service takeoff call failed: %s"%e


if __name__ == "__main__":
	#Obj_det()
	a=Obj_det()
	pick=attach()
	state_cb=rospy.Subscriber('/uav0/mavros/state', State, callback=rover_state)
	rospy.Subscriber('/attach', String, callback=set_mode)
	# Wait for probe and iris pose values to get updated
	#while (GazeboLinkPose('sample_probe').link_pose == PoseStamped() or GazeboLinkPose('iris_1').link_pose == PoseStamped()):
	#    pass
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():

		a.action()
		
		print "action init"
		pick.action()

		if not rovermoved:

			#	des_x =  start_x
			#	des_y =  start_y

				rover_set_mode()
					
				# Arming the drone
				#while not stateMt.state.armed:
				rate.sleep()
				roversetArm()