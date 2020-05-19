
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist , Vector3, TwistStamped
import math
import numpy
from mavros_msgs.srv import *
from opencv_apps.msg import CircleArrayStamped




class OffbPosCtl:
    curr_pose = PoseStamped()
    waypointIndex = 0
    distThreshold = 0.4
    sim_ctr = 1

    des_pose = PoseStamped()
    vel_msg = Twist()
    curr_vel= Twist()
    isReadyToFly = False
    x_vel = 0
    y_vel=0
    z_vel=0
    yaw_rate = 0
    # locations = numpy.matrix([[0, 0, 2, 0, 0, 0],
    #                           [8, 8, 2, 0, 0, 0],
    #                           [0, 0, 2, 0, 0, 0]
    #                           ])

   



    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        #pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
	velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        mocap_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.mocap_cb)
        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_cb)
	vel_sub = rospy.Subscriber('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, callback=self.vel_cb)
        #cicrcles_sub = rospy.Subscriber('/hough_circles/circles', CircleArrayStamped, callback=self.circles_cb)

	wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear',
                                               WaypointClear)
	res = wp_clear_srv()
        rate = rospy.Rate(10)  # Hz
        rate.sleep()
	t0=rospy.Time.now().to_sec()
        #self.des_pose = self.copy_pose(self.curr_pose)
        #shape = self.locations.shape
	self.vel_msg = Twist()#Stamped()
	self.vel_msg.twist = Twist() 	

	self.vel_msg.linear.x = 4
	self.vel_msg.angular.z = 0 

	#print(self.vel_msg)
        while not rospy.is_shutdown():
            #print self.sim_ctr, shape[0], self.waypointIndex
            #if self.waypointIndex is shape[0]:
            #    self.waypointIndex = 0
            #    self.sim_ctr += 1

            if self.isReadyToFly:

                t1=rospy.Time.now().to_sec()
		self.vel_msg.linear.x =3
           	
        	self.vel_msg.angular.z = 0
		print("setting velocities")
		curr_dist=5*(t1-t0)
		#if(curr_dist>50):
		#	self.vel_msg.linear=Vector3(0,0,0)
   	
		#	self.vel_msg.angular=Vector3(0,0,0)
		
		#self.setvelocities(self.x_vel,self.y_vel,self.z_vel,self.yaw_rate)
		#self.setArm()
	    # print("isReadyToFly")
	   
		
	    
            #pose_pub.publish(self.des_pose)
	    velocity_publisher.publish(self.vel_msg)
	    
	    
	    
            rate.sleep()
	    
	    rospy.wait_for_service('mavros/set_mode')
	    try:
    	    
    	    	setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
    	    	setModeService(custom_mode="OFFBOARD")
    	    
	    except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e
	    
            # Arming the drone
	    #while not stateMt.state.armed:
		#md.
		#rate.sleep()

	    #rate.sleep()
	

    #def copy_pose(self, pose):
    #    pt = pose.pose.position
    #    quat = pose.pose.orientation
    #    copied_pose = PoseStamped()
    #    copied_pose.header.frame_id = pose.header.frame_id
    #    copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
    #    copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
    #    return copied_pose


    def mocap_cb(self, msg):
        
        self.curr_pose = msg

    def setvelocities(self, x_vel, y_vel, z_vel, yaw_rate):
        self.vel_msg = self.copy_vel(self.curr_vel)
        self.vel_msg.linear.x = x_vel
        self.vel_msg.linear.y = y_vel
        self.vel_msg.linear.z = z_vel
        self.vel_msg.angular.z = yaw_rate	

   
    def copy_vel(self, vel):
        copied_vel = Twist()
        copied_vel= vel
        return copied_vel
 
    def vel_cb(self, msg):
        self.curr_vel = msg

    #def circles_cb(self,msg):
    #    print msg.circles[0].center.x, msg.circles[0].center.y, msg.circles[0].radius


    def state_cb(self,msg):
        print msg.mode
        if(msg.mode=='OFFBOARD'):
            self.isReadyToFly = True
            print "readyToFly"
	else:
	    print "not ready"
    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e


if __name__ == "__main__":
    OffbPosCtl()

