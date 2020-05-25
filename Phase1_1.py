"""
testing offboard positon control with a simple takeoff script
"""

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

class OffbPosCtl:
	curr_pose = PoseStamped()
	des_pose = PoseStamped()
	vel = Twist()
	isReadyToFly = False
	tagDetected = False
	firstTag = False
	isStart = False
	x_cam = 0
	y_cam = 0
	x_min = 0
	
	def __init__(self):
		rospy.init_node('offboard_test', anonymous=True)
		pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
		vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
		tag_pose = rospy.Subscriber('/our_topic', Pose, callback=self.tag_pose_cb)
		mocap_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.mocap_cb)
		state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_cb)
		self.attach = rospy.Publisher('/attach', String, queue_size=10)

		rate = rospy.Rate(5)  # Hz
		rate.sleep()
		self.des_pose = self.copy_pose(self.curr_pose)

		current_x = self.curr_pose.pose.position.x
		current_y = self.curr_pose.pose.position.y
		curr_z = self.curr_pose.pose.position.z

	#New Addition Below 
		count = 0
		#grid = [[9,-10],[9,2],[-9,2],[-9,-10]]
		grid = [[10,2],[10,-10],[-9,-10],[-9,2]]

		min_dist = math.sqrt((current_x - grid[0][0])**2 + (current_y - grid[0][1])**2)
		start_x = grid[0][0]
		start_y = grid[0][1]
		a = 0
		for i in range(1, len(grid)):
			dist = math.sqrt((current_x - grid[i][0])**2 + (current_y - grid[i][1])**2)
			if dist < min_dist:
				min_dist = dist
				start_x = grid[i][0]
				start_y = grid[i][1]
				a = i
		start_x = 40.2
		start_y = 4.14
		if a ==0 or a==1:
			grid_loc_x = [0,-2,0,-2]
		else:
			grid_loc_x = [0,2,0,2]
		if a == 0 or a ==3:
			grid_loc_y = [grid[2][1]-grid[0][1],0,grid[0][1]-grid[2][1],0]
		else:
			grid_loc_y = [grid[0][1]-grid[2][1],0,grid[2][1]-grid[0][1],0]

		#Above
		distThreshold = 0.5
		del_val = 0.004
		pre_z = 16
		flag = False
		searchEnd = False
		hover = False
		a = 0
		tag_lost_y = 0.0
		inc = 0.0
		split_path = 2.0
		prev_x = start_x
		prev_y = start_y
		attach = False
		detach = False
		while not rospy.is_shutdown():
			print attach, "Attach"
			if self.isReadyToFly:
				if self.firstTag:
					print "tag detected"
				print "Not hover",hover
				des_z =  pre_z
				#Below
				if not searchEnd:
					# print "Yo"
					if self.isStart:
						print "Reached start"
						des_x = prev_x + (grid_loc_x[count])/split_path
						des_y = prev_y + (grid_loc_y[count])/split_path
						tag_lost_y = des_y
					else:
						des_x = start_x
						des_y = start_y
						print des_x,des_y,"Going to start:"

				#Above
				if self.tagDetected and hover == True and not attach and not detach:
					searchEnd = True
					des_x = self.des_pose.pose.position.x
					des_y = self.des_pose.pose.position.y
					if self.curr_pose.pose.position.z >(11.24 + 0.5):
						if abs(del_val*self.x_cam) > 0.2:
							des_y = curr_y - del_val*self.x_cam
							print "Correcting x..",des_y, curr_y
				
						if abs(del_val*self.y_cam) > 0.2:
							des_x = curr_x - del_val*self.y_cam
							print "Correcting y..",des_x, curr_x
						if abs(del_val*self.x_cam)<0.2 and abs(del_val*self.y_cam)<0.2 and flag == False:
							#self.tagDetected = False
							print "Corrected", des_z, self.curr_pose.pose.position.z
							pre_z = des_z - 0.05*des_z
							des_y = curr_y
							des_x = curr_x
							des_z = pre_z
					else: 
						flag = True
						#self.setLandMode()
						#self.setDisarm()
						self.tagDetected = False
						print "Z value",pre_z
						if not attach:
							self.attach.publish("ATTACH")
							attach = True
							rate.sleep()
							
				# elif self.tagDetected == False and not detach and not attach:
				# 	if pre_z < 2:
				# 		print "Hi", pre_z, des_z
				# 		distThreshold = 0.1
				# 		pre_z = pre_z - 0.2
				# 		des_z = pre_z
				# 		des_y = curr_y
				# 		des_x = curr_x
				# 	if pre_z<0.4:
				# 		flag = True
				# 		#self.setLandMode()
				# 		#self.setDisarm()
				# 		self.tagDetected = False
				# 		print "Z value",pre_z
				# 		if not attach:
				# 			self.attach.publish("ATTACH")
				# 			attach = True
				# 			rospy.sleep(2)
				if attach == True:
					if  self.curr_pose.pose.position.z < 21.8: 
						des_z = 0.05* self.curr_pose.pose.position.z + self.curr_pose.pose.position.z 
						print "I'm going up",self.curr_pose.pose.position.z
					# 	des_x = 40
					# 	des_y = 4
					# else:
					# 	print "Going to drop location"
					# 	des_x = 84
					# 	des_y = -54
					# var_x = self.curr_pose.pose.position.x
					# var_y = self.curr_pose.pose.position.y
					# var_z = self.curr_pose.pose.position.z
					# dis_x = math.sqrt((var_x-des_x)*(var_x-des_x) + (var_y-des_y)*(var_y-des_y)+ (var_y-des_y)*(var_y-des_y))
					# print "Going to:",var_x,var_y, self.curr_pose.pose.position.z
					# if dis_x<distThreshold:
					# 	pre_z = self.curr_pose.pose.position.z - 0.2*self.curr_pose.pose.position.z
					# 	des_z = pre_z
					# 	print(self.curr_pose.pose.position.z, pre_z, des_z)
					# 	if pre_z<17.75 and self.curr_pose.pose.position.x < 84.2 and self.curr_pose.pose.position.x < 83.8 and self.curr_pose.pose.position.x < -53.8 and self.curr_pose.pose.position.x > -54.2:
					# 		self.attach.publish("DETACH")
					# 		print("Detached!!!!!")
					# 		rospy.sleep(2)
					# 		rospy.wait_for_service('mavros/set_mode')
					# 		try:
					# 			setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
					# 			setModeService(custom_mode="AUTO.RTL")
					# 		except rospy.ServiceException, e:
					# 			print "Service takeoff call failed: %s"%e
					# 		attach = False
					# 		detach = True
							# self.setLandMode()


				
				if self.firstTag and hover and not self.tagDetected and not attach and not detach:
					des_x = self.curr_pose.pose.position.x
					des_y = self.curr_pose.pose.position.y + (self.curr_pose.pose.position.y - tag_lost_y)/10.0
					print("Tag lost")
					print("Cur: ", self.curr_pose.pose.position.y, "Des: ", des_y)
					des_z = 18
					
				if self.firstTag and hover == False:
					searchEnd = True
					a += 1
					if a == 1:
						x = self.curr_pose.pose.position.x
						y = self.curr_pose.pose.position.y
					des_x = x
					des_y = y
					des_z = 18
					pre_z = des_z
					print curr_z, "current z" , x ,self.curr_pose.pose.position.x, y , self.curr_pose.pose.position.y
					if curr_z >17.9 and curr_x > x-0.1 and curr_x < x+0.1 and curr_y > y-0.1 and curr_y < y+0.1 :
						hover = True
						print("hovering...")

				self.des_pose.pose.position.x = des_x
				self.des_pose.pose.position.y = des_y
				self.des_pose.pose.position.z = des_z

				curr_x = self.curr_pose.pose.position.x
				curr_y = self.curr_pose.pose.position.y
				curr_z = self.curr_pose.pose.position.z

				dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))

				#print curr_x , curr_y , curr_z, "Current Pose"
				if dist < distThreshold:
					if inc==int(split_path):
						#print inc
						count+=1
						inc = 0
					if self.isStart == False:
						self.isStart = True
						count = 0
					current_x = curr_x
					current_y = curr_y
					inc += 1
					prev_x = des_x
					prev_y = des_y
					if count == 4:
						count = 0

			pose_pub.publish(self.des_pose)
			vel_pub.publish(self.vel)
			rate.sleep()
		

	def tag_pose_cb(self,msg):
		if msg.position.x == 1:
			if self.isStart == True:
				self.firstTag = True
				self.tagDetected = True
			# self.x_cam =  8.17018795013msg.poses[0].position.x
			# self.y_cam = msg.poses[0].position.y
						
			x = msg.orientation.x
			y = msg.orientation.y

			self.x_cam = x - 320
			self.y_cam = y - 240
			# print("Camera:",x,y, self.x_cam, self.y_cam)
		else:
			self.tagDetected = False

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
