# Importing Necessary Libraries
import rospy
import mavros
from mavros import command
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import PoseStamped, Point, Quaternion , PoseArray, Twist, Pose
from gazebo_msgs.msg import ContactsState
import math
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Range

class OffbPosCtl:
	curr_pose = PoseStamped()
	des_pose = PoseStamped()
	vel = Twist()

	#Global Flags
	isReadyToFly = False
	tagDetected = False
	firstTag = False
	isStart = False

	#Global Variables
	x_cam = 0
	y_cam = 0
	x_min = 0
	sonar_height = 0.0
	sonar0_height = 0.0
	sonar1_height = 0.0
	def __init__(self):
		#ROS Initializations
		rospy.init_node('offboard_test', anonymous=True)
		pose_pub = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
		tag_pose = rospy.Subscriber('/our_topic', Pose, callback=self.tag_pose_cb)
		mocap_sub = rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, callback=self.mocap_cb)
		state_sub = rospy.Subscriber('/uav1/mavros/state', State, callback=self.state_cb)
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
		rover_x = 12.62
		rover_y = -65.74
		a = 0
		b = 0

		grid_loc_x = [start_x , start_x-10, start_x, start_x+10, start_x] 
		grid_loc_y = [start_y, start_y, start_y+10,start_y,start_y-10]

		land_x = [rover_x,rover_x-10, rover_x, rover_x+10, rover_x] 
		land_y = [rover_y,rover_y, rover_y+10,rover_y,rover_y-10]

		#Local Flags
		attach = False
		detach = False
		height_min = 7
		height_max = 12
		probePicked = False
		probeDeployed = False
		useSonar = True
		random = False
		count = 0
		counter= 0
		des_x =  start_x
		des_y =  start_y
		des_z =  pre_z

		while not rospy.is_shutdown():
			print attach, "Attach"
			self.sonar_height = (self.sonar0_height  + self.sonar1_height)/2
			if self.isReadyToFly:
				if self.firstTag:
					print "tag detected"
					
				# des_x =  start_x
				# des_y =  start_y
				#Going to Probe Location
				if not self.isStart:
					if self.sonar_height < height_min and not self.tagDetected:
						des_z = self.curr_pose.pose.position.z + 2 #(height_min-self.sonar_height)*0.5
						des_x = self.curr_pose.pose.position.x
						des_y = self.curr_pose.pose.position.y
					else:
						des_x = start_x
						des_y = start_y
						print "Hola"
					print "Going to start:",des_x,des_y
				if self.isStart and not self.firstTag:
					if self.sonar_height < height_min:
						des_z = self.curr_pose.pose.position.z + (height_min-self.sonar_height)
					elif self.sonar_height > height_max:
						des_z = self.curr_pose.pose.position.z - (self.sonar_height-height_max)
					des_x = grid_loc_x[count]
					des_y = grid_loc_y[count]
					print ("Reached, Tag not detected :", des_x , des_y, des_z)

				#Visual Surveying and Attaching Probe
				if self.tagDetected and self.isStart and  not attach and not detach:
					des_x = self.des_pose.pose.position.x
					des_y = self.des_pose.pose.position.y
					des_z = self.des_pose.pose.position.z
					if self.sonar_height>0.5: # and self.sonar_height<height_max:
						if abs(del_val*self.x_cam) > err_thresh:
							des_y = curr_y - del_val*self.x_cam
							print "Correcting x..",des_y, curr_y
				
						if abs(del_val*self.y_cam) > err_thresh:
							des_x = curr_x - del_val*self.y_cam
							print "Correcting y..",des_x, curr_x
						if abs(del_val*self.x_cam)<err_thresh and abs(del_val*self.y_cam)<err_thresh:
							#self.tagDetected = False
							print "Corrected", des_z, self.curr_pose.pose.position.z
							pre_z = des_z - 0.05*des_z
							des_y = curr_y
							des_x = curr_x
							des_z = pre_z
					# elif self.sonar_height>height_max:
					# 	des_z = self.curr_pose.pose.position.z - (self.sonar_height-height_max)
					else: 
						self.tagDetected = False
						print "Z value",pre_z
						if not attach:
							self.attach.publish("ATTACH")
							attach = True
							probePicked = True
							rate.sleep()

				# Going to Detach Location
				if attach == True:
					if self.sonar0_height < 1:
						a +=1
						if a >100:
							sonar_height = sonar1_height
					if self.sonar1_height < 1:
						b +=1
						if a >100:
							sonar_height = sonar0_height
					if  self.sonar_height<6 and probePicked: 
						des_z = 0.2* self.curr_pose.pose.position.z + self.curr_pose.pose.position.z 
						print "I'm going up",self.curr_pose.pose.position.z
						# des_x = 40
						# des_y = 4
					else:
						probePicked = False
						print "Going to drop location"
						des_x = -75.99
						des_y = 425.00
						des_z = self.curr_pose.pose.position.z

						if drop_check==0:
							if self.sonar_height < height_min:
								des_z = self.curr_pose.pose.position.z + self.curr_pose.pose.position.z * 0.5
							elif self.sonar_height > height_max:
								des_z = self.curr_pose.pose.position.z - self.curr_pose.pose.position.z * 0.5

						err_x = des_x - self.curr_pose.pose.position.x
						err_y = des_y - self.curr_pose.pose.position.y

						if abs(err_x) < err_thresh: 
							des_x = curr_x 
						if abs(err_y) < err_thresh:
							des_y = curr_y 
						# if abs(err_x) < err_thresh+0.2 and abs(err_y) < err_thresh+0.2:
						if abs(err_x) < self.sonar_height*0.1 and abs(err_y) < self.sonar_height*0.1: 
							drop_check+=1
							pre_z = des_z - 0.1*des_z
							des_y = curr_y
							des_x = curr_x
							des_z = pre_z
							print(self.curr_pose.pose.position.x,self.curr_pose.pose.position.y,self.curr_pose.pose.position.z)

						if self.sonar_height <1.5 and abs(err_x) < 1 and abs(err_y) < 1:
							self.attach.publish("DETACH")
							print("Detached!!!!!")
							rospy.sleep(2)
							attach = False
							detach = True
							probeDeployed = True

				# Landing on Rover
				if probeDeployed:
					des_x = land_x[counter]
					des_y = land_y[counter]
					
					err_x = des_x - self.curr_pose.pose.position.x
					err_y = des_y - self.curr_pose.pose.position.y	

					if self.sonar_height < height_min and useSonar:
						des_z = self.curr_pose.pose.position.z + self.curr_pose.pose.position.z * 0.5
					elif self.sonar_height > height_max and useSonar:
						des_z = self.curr_pose.pose.position.z - self.curr_pose.pose.position.z * 0.5

					if abs(err_x) < 1 and abs(err_y) < 1:
						useSonar = False
					if self.tagDetected and not useSonar:
						random = True
						if abs(del_val*self.x_cam) > err_thresh:
							des_y = curr_y - del_val*self.x_cam
							print "Correcting x..",des_y, self.curr_pose.pose.position.y
				
						if abs(del_val*self.y_cam) > err_thresh:
							des_x = curr_x - del_val*self.y_cam
							print "Correcting y..",des_x, self.curr_pose.pose.position.x
						if abs(del_val*self.x_cam)< err_thresh and abs(del_val*self.y_cam)< err_thresh:
							#self.tagDetected = False
							print "Corrected", des_z, self.curr_pose.pose.position.z
							pre_z = self.curr_pose.pose.position.z - 1#*self.curr_pose.pose.position.z
							des_y = self.curr_pose.pose.position.y
							des_x = self.curr_pose.pose.position.x
							des_z = pre_z
						if self.sonar_height < 2:
							rospy.wait_for_service('mavros/set_mode')
							try:
								setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
								setModeService(custom_mode="AUTO.LAND")
							except rospy.ServiceException, e:
								print "Service takeoff call failed: %s"%e
					if random and not self.tagDetected:
						des_x = land_x[counter]
						des_y = land_y[counter]
						err_x = des_x - self.curr_pose.pose.position.x
						err_y = des_y - self.curr_pose.pose.position.y
						random = False		
					if not self.tagDetected and not useSonar and counter<4 and abs(err_x) < 1 and abs(err_y) < 1:
						counter+=1
						useSonar = True

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
				print "#############################################", des_z

				curr_x = self.curr_pose.pose.position.x
				curr_y = self.curr_pose.pose.position.y
				curr_z = self.curr_pose.pose.position.z

				dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y))
				print dist

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

			pose_pub.publish(self.des_pose)
			vel_pub.publish(self.vel)
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
		print "sonar: ", msg.range
		self.sonar0_height = msg.range

	def sonar1_sub_cb(self,msg):
		print "sonar1: ", msg.range
		self.sonar1_height = msg.range
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
