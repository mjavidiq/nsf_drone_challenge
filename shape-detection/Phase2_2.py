# Importing Necessary Libraries
import rospy
import mavros
from mavros import command
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import PoseStamped, Point, Quaternion , PoseArray, Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ContactsState
import math
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Range
from gazebo_msgs.srv import GetModelState

class OffbPosCtl:
	curr_pose = PoseStamped()
	des_pose = PoseStamped()
	vel = Twist()


	#Global Flags
	isReadyToFly = False
	tagDetected = False
	firstTag = False
	isStart = True

	#Global Variables
	x_cam = 0
	y_cam = 0
	x_min = 0
	sonar_height = 0.0
	sonar0_height = 0.0
	sonar1_height = 0.0
	rover_x = 0
	rover_y = 0

	def __init__(self):
		#ROS Initializations
		rospy.init_node('offboard_test', anonymous=True)
		pose_pub = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
		vel_pub = rospy.Publisher('/uav1/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
		tag_pose = rospy.Subscriber('/our_topic', Pose, callback=self.tag_pose_cb)
		mocap_sub = rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, callback=self.mocap_cb)
		state_sub = rospy.Subscriber('/uav1/mavros/state', State, callback=self.state_cb)
		rover_sub = rospy.Subscriber('/uav0/mavros/local_position/odom', Odometry, callback=self.rover_cb)
		sonar_sub = rospy.Subscriber('/sonar', Range, callback=self.sonar_sub_cb)
		sonar1_sub = rospy.Subscriber('/sonar1', Range, callback=self.sonar1_sub_cb)
		self.attach = rospy.Publisher('/attach', String, queue_size=10)

		rate = rospy.Rate(5)  # Hz
		rate.sleep()
		self.des_pose = self.copy_pose(self.curr_pose)

		current_x = self.curr_pose.pose.position.x
		current_y = self.curr_pose.pose.position.y
		curr_z = self.curr_pose.pose.position.z


		#Local Variables 
		distThreshold = 0.5
		err_thresh = 0.2
		del_val = 0.004
		pre_z = 6
		drop_check = 0
		tag_lost_y = 0.0
		split_path = 2.0
		start_x = 84.72
		start_y = -54.40

		a = 0
		b = 0

		grid_loc_x = [start_x , start_x, start_x, start_x, start_x] 
		grid_loc_y = [start_y, start_y, start_y,start_y,start_y]

		#Local Flags
		attach = False
		detach = False
		height_min = 6
		height_max = 9
		probePicked = False
		probeDeployed = True
		useSonar = True
		random = False
		someFlag = False
		count = 0
		counter= 0
		vel_control = False
		des_x =  start_x
		des_y =  start_y
		des_z =  pre_z


		while not rospy.is_shutdown():
			print self.firstTag, "Tag"
			# self.vel.twist = Twist()
			self.sonar_height = (self.sonar0_height)  #+ self.sonar1_height)/2
			self.model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
			self.object_coordinates = self.model_coordinates('rover_0',"")
			
			#print (self.object_coordinates.pose.position) 
			if not self.isReadyToFly:

			#	des_x =  start_x
			#	des_y =  start_y

				self.auto_set_mode()
					
				# Arming the drone
				#while not stateMt.state.armed:
				rate.sleep()
				self.setArm()
			if self.isReadyToFly:

				# Landing on Rover
				if probeDeployed:
					vel_control = False
					# if self.rover_x == 0 and self.rover_y == 0:
					des_x = self.rover_x - 1
					des_y = self.rover_y + 4
					# else:
					# des_x = self.object_coordinates.pose.position.x -1
					# des_y = self.object_coordinates.pose.position.y
					des_z = self.curr_pose.pose.position.z

					# err_x = des_x - self.curr_pose.pose.position.x
					# err_y = des_y - self.curr_pose.pose.position.y

					# if someFlag:
					# 	print("On Top ###################")
					# 	change_x = self.object_coordinates.pose.position.x - prev_rover_x
					# 	change_y = self.object_coordinates.pose.position.y - prev_rover_y
					# 	des_x = self.curr_pose.pose.position.x + change_x
					# 	des_y = self.curr_pose.pose.position.y + change_y
					# 	print (change_x,change_y)
					# 	print (self.object_coordinates.pose.position.x,self.object_coordinates.pose.position.y)
					# 	print (self.curr_pose.pose.position.x,self.curr_pose.pose.position.y)
					# 	print (des_x,des_y)
					# 	# des_z = self.curr_pose.pose.position.z - 0.1
					# 	prev_rover_x = self.object_coordinates.pose.position.x
					# 	prev_rover_y = self.object_coordinates.pose.position.y

	

					if self.sonar_height < height_min and not self.tagDetected:
						des_z = self.curr_pose.pose.position.z + 2*(self.sonar_height) #4 for ditch avoiding
					elif self.sonar_height > height_max and not self.tagDetected:
						des_z = self.curr_pose.pose.position.z - 0.5*(self.sonar_height)

					# # if abs(err_x) < 1 and abs(err_y) < 1:
					# # 	useSonar = False
					if self.tagDetected:
						# des_z = self.curr_pose.pose.position.z - 1#*(self.sonar_height)
						vel_control = True
						height_min = 1.8
						self.vel.linear.z = 0

						# if abs(del_val*self.x_cam) > err_thresh:
						# 	des_y = curr_y - 0.005*self.sonar_height*self.x_cam
						# 	print "Correcting x..",des_y, self.curr_pose.pose.position.y
				
						# if abs(del_val*self.y_cam) > err_thresh:
						# 	des_x = curr_x - 0.004*self.y_cam
						# 	print "Correcting y..",des_x, self.curr_pose.pose.position.x
						# if abs(del_val*self.x_cam)< err_thresh and abs(del_val*self.y_cam)< err_thresh:
						# 	# prev_rover_x = self.object_coordinates.pose.position.x
						# 	# prev_rover_y = self.object_coordinates.pose.position.y
						# 	# someFlag = True
						# 	#self.tagDetected = False
						# 	pre_z = self.curr_pose.pose.position.z - 1
						# 	# des_y = self.object_coordinates.pose.position.y
						# 	# des_x = self.object_coordinates.pose.position.x
						# 	des_z = pre_z
						# 	print "Corrected", des_z, self.curr_pose.pose.position.z

							# if self.sonar_height< 4:
							# 	random = true
						# if abs(del_val*self.y_cam) > err_thresh:
						# 	self.vel.linear.x = self.vel.linear.x #+ 0.01*self.y_cam

						if abs(del_val*self.y_cam)> err_thresh:
							self.vel.linear.x = self.vel.linear.x - 1*del_val*self.y_cam
							print(del_val*self.y_cam, err_thresh, "Threshold exceeded")
						if (del_val*self.x_cam)> err_thresh:
							self.vel.linear.y = self.vel.linear.y - 1*del_val*self.x_cam
							print(del_val*self.y_cam, err_thresh, "Threshold exceeded")
						elif (del_val*self.x_cam)< -err_thresh:
							self.vel.linear.y = self.vel.linear.y - 1.25*del_val*self.x_cam
							print(del_val*self.y_cam, err_thresh, "Threshold exceeded")
						if abs(del_val*self.x_cam)< err_thresh and abs(del_val*self.y_cam)< err_thresh:
							self.vel.linear.z = -2.5

							# print ("IF x correction")
						# else:
						# 	self.vel.linear.x = self.vel.linear.x + 0.5
							# print ("ELIF x correction", self.x_cam)
						# if self.x_cam > 0:
						# 	self.vel.linear.y = self.vel.linear.y - 0.5
						# 	print ("IF y correction", self.x_cam)
						# else:
						# 	self.vel.linear.y = self.vel.linear.y + 0.5
						# 	print ("ELIF y correction", self.x_cam)
						# if (del_val*self.y_cam) > err_thresh:
						# 	self.vel.linear.y = self.vel.linear.y - 0.005*self.x_cam
						# if abs(del_val*self.x_cam)< err_thresh and abs(del_val*self.y_cam)< err_thresh:
						# 	self.vel.linear.z = -0.5
						print("Published velocity:",self.vel)

						if self.sonar_height < 1.2:
							rospy.wait_for_service('/uav1/mavros/set_mode')
							try:
								setModeService = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
								setModeService(custom_mode="AUTO.LAND")
								print("Landing initiated!")
							except rospy.ServiceException, e:
								print "Service takeoff call failed: %s"%e
					print("---------------------------Sonar height:", self.sonar_height)
					# print(self.curr_pose.pose.position.x , self.curr_pose.pose.position.y, self.curr_pose.pose.position.z)
					# print(des_x,des_y,des_z)
					# if random and not self.tagDetected:
					# 	des_x = self.object_coordinates.pose.position.x
					# 	des_y = self.object_coordinates.pose.position.y
					# 	err_x = des_x - self.curr_pose.pose.position.x
					# 	err_y = des_y - self.curr_pose.pose.position.y
					# 	random = False		


				# GoBack Feature/Safety Not Necessary
				# if self.firstTag and not self.tagDetected and not attach and not detach:
				# 	des_x = self.curr_pose.pose.position.x
				# 	des_y = self.curr_pose.pose.position.y + (self.curr_pose.pose.position.y - tag_lost_y)/10.0
				# 	print("Tag lost")
				# 	print("Cur: ", self.curr_pose.pose.position.y, "Des: ", des_y)
				# 	des_z = self.curr_pose.pose.position.z				

				self.des_pose.pose.position.x = des_x
				self.des_pose.pose.position.y = des_y
				self.des_pose.pose.position.z = des_z
				#print "#############################################", des_z

				curr_x = self.curr_pose.pose.position.x
				curr_y = self.curr_pose.pose.position.y
				curr_z = self.curr_pose.pose.position.z

				dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y))
				#print dist

				#print curr_x , curr_y , curr_z, "Current Pose"
				if dist < distThreshold:
					dist_start = math.sqrt((curr_x - start_x)*(curr_x - start_x) + (curr_y - start_y)*(curr_y - start_y))

					if self.isStart and count<4:
						print("Count",count)
						count+=1
					if not self.isStart and dist_start<distThreshold:
						print "Reached start"
						self.isStart = True
					current_x = curr_x
					current_y = curr_y
					prev_x = des_x
					prev_y = des_y

			if not vel_control:
				pose_pub.publish(self.des_pose)
			else:
				if self.rover_x == 0 and self.rover_y == 0:
					print(self.vel.linear,"Incorrect")
				else:
					# pose_pub.publish(self.des_pose)
					vel_pub.publish(self.vel)
					print(self.vel.linear,"Correct")
			rate.sleep()
		

	def tag_pose_cb(self,msg):
		if msg.position.x == 1:
			if self.isStart == True:
				self.firstTag = True
				self.tagDetected = True
			x = msg.orientation.x
			y = msg.orientation.y

			self.x_cam = x - 320
			self.y_cam = y - 240
		else:
			self.tagDetected = False

	def sonar_sub_cb(self,msg):
		#print "sonar: ", msg.range
		self.sonar0_height = msg.range

	def sonar1_sub_cb(self,msg):
		#print "sonar1: ", msg.range
		self.sonar1_height = msg.range

	def rover_cb(self,msg):

		self.rover_x = msg.pose.pose.position.x
		self.rover_y = msg.pose.pose.position.y

		# self.vel = msg.twist.twist

		self.vel.linear.x = msg.twist.twist.linear.y
		self.vel.linear.y = msg.twist.twist.linear.x


		#print(type(self.vel.twist.linear))


		# print(self.vel)
	def auto_set_mode(self):
	    rospy.wait_for_service('/uav1/mavros/set_mode')
	    try:
	        # setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.set_mode.request.custom_mode)
	        setModeService = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
	        setModeService(custom_mode="OFFBOARD")
	        #print "OFFBOARD"
	    except rospy.ServiceException, e:
	        print "Service takeoff call failed: %s"%e
	def setArm(self):
	    rospy.wait_for_service('/uav1/mavros/cmd/arming')
	    try:
	        armService = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
	        armService(True)
	        #rate.sleep()
	        #print "ARMING"
	    except rospy.ServiceException, e:
	        print "Service arming call failed: %s"%e


#################DO NOT TOUCH BELOW #########################
	def copy_pose(self, pose):
		pt = pose.pose.position
		quat = pose.pose.orientation
		copied_pose = PoseStamped()
		copied_pose.header.frame_id = pose.header.frame_id
		copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
		copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
		return copied_pose

	def mocap_cb(self, msg):
		# print msg
		self.curr_pose = msg

	def state_cb(self,msg):
		print msg.mode
		if(msg.mode=='OFFBOARD'):
			self.isReadyToFly = True
			print "readyToFly"

	def setDisarm(self):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
				armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
				armService(False)
		except rospy.ServiceException, e:
				print "Service arm call failed: %s"%e

	def setLandMode(self):
		rospy.wait_for_service('/mavros/cmd/land')
		try:
				landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
				#http://wiki.ros.org/mavros/CustomModes for custom modes
				isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
		except rospy.ServiceException, e:
				print "service land call failed: %s. The vehicle cannot land "%e

if __name__ == "__main__":
	OffbPosCtl()
