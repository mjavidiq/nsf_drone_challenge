"""
testing offboard positon control with a simple takeoff script
"""

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import numpy

class OffbPosCtl:
	curr_pose = PoseStamped()
	waypointIndex = 0
	distThreshold = 0.4
	sim_ctr = 1

	des_pose = PoseStamped()
	isReadyToFly = False


	def __init__(self):
		rospy.init_node('offboard_test', anonymous=True)
		pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
		mocap_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.mocap_cb)
		state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_cb)

		rate = rospy.Rate(10)  # Hz
		rate.sleep()
		self.des_pose = self.copy_pose(self.curr_pose)
		current_x = self.curr_pose.pose.position.x
		current_y = self.curr_pose.pose.position.y
		count = 0
		
		grid = [[-9,2],[-9,-10],[9,-10],[9,2]]
		grid_loc_y = [grid[0][1]-grid[2][1],0,grid[2][1]-grid[0][1],0]
		grid_loc_x = [0,-2,0,-2]

		while not rospy.is_shutdown():
			if self.isReadyToFly:

				des_x = current_x + grid_loc_x[count]
				des_y = current_y + grid_loc_y[count]
				des_z = 4
				self.des_pose.pose.position.x = des_x
				self.des_pose.pose.position.y = des_y
				self.des_pose.pose.position.z = des_z


				curr_x = self.curr_pose.pose.position.x
				curr_y = self.curr_pose.pose.position.y
				curr_z = self.curr_pose.pose.position.z

				dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
				if dist < self.distThreshold:
					current_x = curr_x
					current_y = curr_y
					count+=1
					if count == 4:
						count = 0	
				# print dist, curr_x, curr_y, curr_z, self.waypointIndex
			pose_pub.publish(self.des_pose)
			rate.sleep()

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


if __name__ == "__main__":
	OffbPosCtl()
