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

class OffbPosCtl:
	curr_pose = PoseStamped()
	des_pose = PoseStamped()
	vel = Twist()
	isReadyToFly = False
	tagDetected = False
	bumperDetected = False
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
		bumper_sub = rospy.Subscriber('/benchmarker/collision', ContactsState, callback=self.bumper_cb)

		rate = rospy.Rate(5)  # Hz
		rate.sleep()
		self.des_pose = self.copy_pose(self.curr_pose)

		current_x = self.curr_pose.pose.position.x
		current_y = self.curr_pose.pose.position.y
		curr_z = self.curr_pose.pose.position.z

	#New Addition Below 
		count = 0
		grid = [[9,-10],[9,2],[-9,2],[-9,-10]]
		grid_loc_y = [grid[2][1]-grid[0][1],0,grid[0][1]-grid[2][1],0]
		grid_loc_x = [0,-2,0,-2]

		min_dist = math.sqrt((current_x - grid[0][0])**2 + (current_y - grid[0][1])**2)
		start_x = grid[0][0]
		start_y = grid[0][1]
		for i in range(1, len(grid)):
			dist = math.sqrt((current_x - grid[i][0])**2 + (current_y - grid[i][1])**2)
			if dist < min_dist:
				min_dist = dist
				start_x = grid[i][0]
				start_y = grid[i][1]
		#Above
		distThreshold = 0.2
		del_val = 0.004
		pre_z = 4
		flag = False
		searchEnd = False
		hover = False
		a = 0
		tag_lost_y = 0.0
		inc = 0.0
		split_path = 1.0
		prev_x = start_x
		prev_y = start_y
		while not rospy.is_shutdown():
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
						print des_x,des_y,"Going to start.."

				#Above
				if self.tagDetected and hover == True:
					searchEnd = True
					des_x = self.des_pose.pose.position.x
					des_y = self.des_pose.pose.position.y

					if abs(del_val*self.x_cam) > 0.2:
						des_y = curr_y - del_val*self.x_cam
						print "Correcting x..",des_y, curr_y
			
					if abs(del_val*self.y_cam) > 0.2:
						des_x = curr_x - del_val*self.y_cam
						print "Correcting y..",des_x, curr_x
					if abs(del_val*self.x_cam)<0.2 and abs(del_val*self.y_cam)<0.2 and flag == False:
						#self.tagDetected = False
						pre_z = des_z - 0.05*des_z
						des_y = curr_y
						des_x = curr_x
						des_z = pre_z
						if pre_z<0.25:
							flag = True
							self.setLandMode()
							#self.setDisarm()
							self.tagDetected = False
							print "Arming",pre_z

						print "Corrected", des_z
				elif self.tagDetected == False and self.bumperDetected == False:
					if pre_z < 2:
						print "Hi", pre_z, des_z
						distThreshold = 0.1
						pre_z = pre_z - 0.2
						des_z = pre_z
						des_y = curr_y
						des_x = curr_x

				if self.bumperDetected:
					print "bumper!!!!!!!!"
					des_x = 0
					des_y = 0
					var_x = self.curr_pose.pose.position.x
					var_y = self.curr_pose.pose.position.y
					dis_x = math.sqrt((var_x-des_x)*(var_x-des_x) + (var_y-des_y)*(var_y-des_y))
					print var_x,var_y
					if dis_x>distThreshold:
						des_z = 5
					else:
						pre_z = des_z - 0.05*des_z
						if pre_z<0.25:
							self.setLandMode()

				if self.firstTag and hover and not self.tagDetected and not self.bumperDetected:
					des_x = self.curr_pose.pose.position.x
					des_y = self.curr_pose.pose.position.y + (self.curr_pose.pose.position.y - tag_lost_y)/10.0
					print("Tag lost")
					print("Cur: ", self.curr_pose.pose.position.y, "Des: ", des_y)
					des_z = 6
					
				if self.firstTag and hover == False:
					searchEnd = True
					a += 1
					if a == 1:
						x = self.curr_pose.pose.position.x
						y = self.curr_pose.pose.position.y
					des_x = x
					des_y = y
					des_z = 6
					pre_z = des_z
					print curr_z, "current z" , x ,self.curr_pose.pose.position.x, y , self.curr_pose.pose.position.y
					if curr_z >5.9 and curr_x > x-0.1 and curr_x < x+0.1 and curr_y > y-0.1 and curr_y < y+0.1 :
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
						print inc
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

	def bumper_cb(self,msg):
		if msg.states != [] and self.firstTag:
			self.bumperDetected = True


if __name__ == "__main__":
	OffbPosCtl()
