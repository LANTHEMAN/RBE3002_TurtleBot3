#!/usr/bin/env python
import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
from lab3.srv import PathRequest
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from heapq import *
from nav_msgs.msg import GridCells, Path
from std_msgs.msg import *
from lab3.srv import *
class Robot:
	
    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
        """
        self.mapChange = False	   
        self._current = Pose() # initlize correctly _
        self._current.position.x=0
        self._current.position.y = 0
        self._current.orientation.x = 0
        self._current.orientation.y = 0
        self._current.orientation.z = 0
        self._current.orientation.w = 0
        self._yaw = 0

        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.01), self.timerCallback)
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.distAccuracy = .01
        self.angleAccuracy = .01
        self.maxLinAcc = 1 #meters per second per meter
        self.maxSpeed = .22
        self.maxAngAcc = .8 #radians per second per radian
        self.aStarService = rospy.ServiceProxy('request_path',PathRequest)
        rospy.Subscriber('/mapChange',Bool, self.setMapChange, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.navigate, queue_size=1) # handle nav goal events
    
    def setMapChange(self,bool):
        self.mapChange = True

    def navigate(self,goal):
        #create srv request using goal and current pos
        
        start = self._current.position
        end = goal.pose.position
        pathRequestResult = self.aStarService(start,end)
        i = 0
        while (i < len(pathRequestResult.path.poses) and len(pathRequestResult.path.poses) > 0):
            self.navToPose(pathRequestResult.path.poses[i])
            i += 1
            if(self.mapChange == True):
                start = self._current.position
                pathRequestResult = self.aStarService(start,end)
                i = 0
                self.mapChange = False



       #use srv reqeust resposne to call runwholepath

    def runWholePath(self,pathRequest):
        poses = pathRequest.path.poses
        for nextGoal in poses:
            self.navToPose(nextGoal)

    def navToPose(self,goal):
        """
            This is a callback function. It should exract data from goal, drive in a striaght line to reach the goal and 
            then spin to match the goal orientation.
        """
        goalPose = goal.pose
        goal.header.stamp = rospy.Time.now()
        print "waiting for transform", goal.header.frame_id
        self._odom_list.waitForTransform('/odom', goal.header.frame_id, rospy.Time.now(), rospy.Duration(2.0))
        self._odom_list.waitForTransform('/odom', goal.header.frame_id, rospy.Time.now(), rospy.Duration(2.0))
        robotGoal = self._odom_list.transformPose('/odom', goal) # transform the nav goal to the global coordinate system
        
        #find angle for arriving at position
        tempAngle = math.atan2(goalPose.position.y - self._current.position.y, goalPose.position.x - self._current.position.x)
        
        self.rotate(tempAngle - self._yaw) #rotate towards goal
        #print(self._yaw)
        self.driveStraight(self.maxSpeed, self.distanceFrom(goalPose)) #drive to goal
        self.rotate(self.angleFrom(goalPose)) #rotate to orientation


    def executeTrajectory(self):
      """
        See lab manual for the dance the robot has to excute
      """
      self.driveStraight(self.maxSpeed, .6)
      self.rotate(-math.pi/2)
      self.driveStraight(self.maxSpeed, .45)
      self.rotate(2.356)


    def driveStraight(self, speed, distance):
        """
            This method should populate a Twist message type and publish it to /cmd_vel in order to move the robot
        """
        assert distance>=0

        origin = copy.deepcopy(self._current) #hint:  use this
        #print("origin", origin.position.x, origin.position.y)

        error = distance - self.distanceFrom(origin)
        while(error>self.distAccuracy):
            #Trapezoidal velocity ramping- accelerates to max speed and decelerates to 0
            if(error>distance/2.0):
                vel = min(self.maxLinAcc * self.distanceFrom(origin) + .1, abs(speed)) #accelerating #slgiht starting speed needed to get moving
            else:
                vel = min(-self.maxLinAcc*self.distanceFrom(origin) + self.maxLinAcc*distance + .1, abs(speed))#deccelerating #slight ending speed so it doesnt take too long
            vel = np.sign(speed)*vel #controls the direction of travel

            self._vel_pub.publish(self.makeTwist(vel,0))
            #print(vel)
            error = distance - self.distanceFrom(origin)
            pass
        self._vel_pub.publish(self.makeTwist(0,0))
        #print("end", self._current.position.x, self._current.position.y)
        print("went Straight ", self.distanceFrom(origin), "m");
        
    
    def spinWheels(self, v_left, v_right, time):
        """
           This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
           It should then create a Twist message type, and publish it to /cmd_vel in order to move the robot
        """

        L = 0.23 # based on wheel track from https://yujinrobot.github.io/kobuki/doxygen/enAppendixKobukiParameters.html

        omega = (v_right - v_left) / L 
        linearX = (v_left + v_right)/2 #average of wheel speeds is velocity
        
       
        driveStartTime = rospy.Time.now().secs
       
        self._vel_pub.publish(self.makeTwist(linearX,omega))
        print("publishing " , linearX, " ", omega)
        rospy.sleep(.5)

        while rospy.Time.now().secs - driveStartTime < time:                 
            pass

        self._vel_pub.publish(self.makeTwist(0,0))
        print("stopping")
        rospy.sleep(.5)

    def makeTwist(self,linearX, omega):
        twist = Twist()
        twist.linear.x=linearX
        twist.angular.z=omega
        return twist


        
    def rotate(self,angle):
        """
            This method should populate a ??? message type and publish it to ??? in order to spin the robot
        """
        if(angle > math.pi): #turn the shortest amount
            angle = - (2*math.pi - angle)   
        if(angle < -math.pi):
            angle = 2*math.pi - abs(angle)
      
        maxOmega = .5 
        

        origin = copy.deepcopy(self._current)
        posAngle = abs(angle)
        

        error = posAngle - abs(self.angleFrom(origin))
        print('starting error', error)
        while(error>self.angleAccuracy):
            #trapezoidal velocity curve for omega
            if(error>posAngle/2.0):
                omega = min(self.maxAngAcc * abs(self.angleFrom(origin)) + .4, abs(maxOmega)) #accelerating to max omega
            else:
                omega = min(-self.maxAngAcc*abs(self.angleFrom(origin)) + self.maxAngAcc*posAngle +.25, abs(maxOmega)) #deccelerating to zero
            omega = np.sign(angle)*omega

            self._vel_pub.publish(self.makeTwist(0,omega))
            error = posAngle - abs(self.angleFrom(origin))
            #print ('error ', error, 'omega ', omega)
            pass
        self._vel_pub.publish(self.makeTwist(0,0))
        print("turned ", self.angleFrom(origin), "rads");
        

    def driveArc(radius,speed,angle):
        origin = copy.deepcopy(self._current)



    def timerCallback(self,evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
        """
	# wait for and get the transform between two frames
        self._odom_list.waitForTransform('/odom', '/base_footprint', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('/odom','/base_footprint', rospy.Time(0)) 
        #print("position", position[0], position[1])

	   # save the current position and orientation

        self._current.position.x=position[0]
        self._current.position.y = position[1]
        self._current.orientation.x = orientation[0]
        self._current.orientation.y = orientation[1]
        self._current.orientation.z = orientation[2]
        self._current.orientation.w = orientation[3]

        
	# create a quaternion
    	q = [self._current.orientation.x,
                 self._current.orientation.y,
                 self._current.orientation.z,
                 self._current.orientation.w] 

	# convert the quaternion to roll pitch yaw
        (roll, pitch, yaw) = euler_from_quaternion(q)   
        self._yaw = yaw 
        #print("orientation", yaw)
        

    # helper functions
    def planTraj(self, b, t):
        """
            Bonus Question:  compute the coefs for a cubic polynomial (hint:  you did this in 3001)
        """

    #computes the robots current distance from origin
    def distanceFrom(self, origin):
        return math.sqrt((origin.position.x - self._current.position.x)**2 + (origin.position.y - self._current.position.y)**2)

    #determine the roboots curent angle from origin
    def angleFrom(self,origin):
        curQ = [self._current.orientation.x,
                 self._current.orientation.y,
                 self._current.orientation.z,
                 self._current.orientation.w] 
        originQ = [origin.orientation.x,
                 origin.orientation.y,
                 origin.orientation.z,
                 origin.orientation.w] 

        return euler_from_quaternion(originQ)[2] - euler_from_quaternion(curQ)[2]
        
if __name__ == '__main__':
    
    rospy.init_node('drive_base')
    turtle = Robot()
    rospy.sleep(2000);    

    rospy.wait_for_service('request_path')
    rospy.spin()
    while  not rospy.is_shutdown():
        rospy.wait_for_service('request_path')
        pass 
