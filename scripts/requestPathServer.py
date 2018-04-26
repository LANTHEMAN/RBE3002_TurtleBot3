#!/usr/bin/env python
from tf.transformations import quaternion_from_euler
import rospy
from nav_msgs.msg import OccupancyGrid
#from lab3.srv import PathRequest
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from heapq import *
from nav_msgs.msg import GridCells, Path
from std_msgs.msg import *
import math
import tf
from lab3.srv import *

import cv2
import numpy as np
from matplotlib import pyplot as plt

import copy

class PathFinder:
	def __init__(self):#constructor
		self.robotRadius = .135
		self.map = None
		self.local_costmap = None
		self.downSizedMap = None
		self.start = None
		self.goal = None
		self.dumbcounter = 0
		rospy.init_node('request_path_server')#initiate path finding node
		s = rospy.Service('request_path', PathRequest, self.aStar)

		self._odom_list = tf.TransformListener()



		#publisher for path,obstacles
		self.pathPub = rospy.Publisher("/path", Path,queue_size = 1)
		self.obstaclePub = rospy.Publisher("/obstacles", GridCells, queue_size =1)
		self.pathGrid = rospy.Publisher("/pathGrid", GridCells, queue_size =1)
		self.closedPub = rospy.Publisher("/closed_set", GridCells,queue_size = 1)
		self.openPub = rospy.Publisher("/open_set", GridCells, queue_size = 1)
		self.changeMap = rospy.Publisher("/mapChange",Bool, queue_size = 1)

		#rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.updateLocalMap, queue_size=1)
		rospy.Subscriber('/map', OccupancyGrid, self.updateMap, queue_size=1) # handle nav goal events
		while(self.map == None): 
			print 'waiting for map'
			pass		
		# while(self.local_costmap == None):
		# 	print 'waiting for local costmap'
		# 	pass	
		#rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.updateGoal, queue_size=1)#subscribe to Rviz goal marker
		#rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,self.updateStart,queue_size=1)#subscribe to Rviz starting marker

		rospy.spin()

	def updateLocalMap(self,occGrid):
		self.dumbcounter += 1
		if (self.dumbcounter > 1000):
			self.changeMap.publish(True)
			self.dumbcounter = 0
		self.local_costmap = copy.deepcopy(occGrid)

	# def updateStart(self, p):
	# 	poseStamped = PoseStamped()
	# 	poseStamped.header = p.header
	# 	poseStamped.pose = p.pose.pose

	# 	self._odom_list.waitForTransform('/odom', p.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
	# 	newPose = self._odom_list.transformPose("/odom", poseStamped)
	# 	self.start = newPose.pose.position
	# 	print "start updated to \n", self.start
	# 	#print("xy index", self.pointToIndex(p.pose.pose.position))

	def updateGoal(self,goal):
		self._odom_list.waitForTransform('/map', goal.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
		newPose = self._odom_list.transformPose('/map', goal).pose # transform the nav goal to the global coordinate system
		self.goal = newPose.position
		request = PathRequest()
		request.start = self.start
		request.end = self.goal
		print "end updated to \n",self.goal
		self.aStar(request)


	# def downsizeMap(self,occGrid,ratio):
	# 	self.map = copy.deepcopy(occGrid)
	# 	for i in range(len(map.data)):
	# 		if(map.data[i] > 50):







	def updateMap(self,occGrid):

		# print "Updating map"
			print "new map found"
			self.map = copy.deepcopy(occGrid)
			obstacles = []
			print("expanding obstacles")
			npArray = np.array(self.map.data).reshape(self.map.info.width, self.map.info.height)
			np.add(npArray, 1, npArray)
			npArray = npArray.astype("uint8")


			#print(np.max(npArray))
			#print(np.min(npArray))
			#print npArray
			#print(np.shape(npArray))
			kernel = np.ones((5,5), np.uint8)
			npArray  = cv2.dilate(npArray, kernel, iterations=1)
			# cv2.namedWindow("image", cv2.WINDOW_NORMAL)
			# cv2.imshow("image", npArray)
			# cv2.waitKey(0)
			npArray = npArray.astype("int")
			np.add(npArray, -1, npArray)
			#print(np.max(npArray))
			#print(np.min(npArray))
			npArray = npArray.reshape(self.map.info.width * self.map.info.height)
			self.map.data = npArray.tolist()
			#print self.map.data
			possibleFrontier = []
			print "publishing obstacles"
			for i in range(len(self.map.data)):	#Publish the obstacles in the map
				if(self.map.data[i] > 50 ):
					obstacles.append(self.indexToPoint(i))
				#if(self.map.data[i] < 0):
					#possibleFrontier.append(self.indexToPoint(i))
			#print(possibleFrontier)
			print "done"


			grid = self.makeGridCell(obstacles)	

			for i in range(3):
				self.obstaclePub.publish(grid)
			print("published Obstacles")


	def expandObstacleFromPoint(self,point):
		self.expandObstacle(self.pointToIndex(point,self.map));

	def expandObstacle(self,index):
		expansionSize = (int) (self.robotRadius / self.map.info.resolution)

		self.map.data = list(self.map.data)

		neighbors = self.getNeighbors(self.indexToPoint(index),expansionSize)
		for n in neighbors:
			if(self.pointToIndex(n,self.map) > 0 and self.pointToIndex(n,self.map) < len(self.map.data)):
				self.map.data[self.pointToIndex(n,self.map)] = self.map.data[index]
		self.map.data = tuple(self.map.data)




		#print(occGrid.info)
		#print("\n")

	def indexToPoint(self, i):
		x = (i % self.map.info.width) * self.map.info.resolution +self.map.info.origin.position.x
		y= i / self.map.info.width * self.map.info.resolution + self.map.info.origin.position.y

		return Point(x,y,0)



	def xyToIndex(self,x,y,map1):
		x -= map1.info.origin.position.x
		y-= map1.info.origin.position.y
		xIndex = (int)(x/map1.info.resolution)
		yIndex = (int)(y/map1.info.resolution)
		return int(yIndex*map1.info.width + xIndex)
	def pointToIndex(self,p,map1):
		# newP = PoseStamped()
		# newP.header.frame_id = "/odom"
		# newP.header.stamp = rospy.Time.now()
		# newP.pose.position = p
		# self._odom_list.waitForTransform(map1.header.frame_id, newP.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
		# finalMapPose = newP# self._odom_list.transformPose(map1.header.frame_id, newP)

		x = p.x#Pose is in odom finalMapPose.pose.position.x
		y = p.y# pose is in odom finalMapPose.pose.position.y
		return self.xyToIndex(x,y,map1)

	def aStar(self,req):
		print "Processing Request to go from\n", req.start, "\nto\n",
		self.goal = req.end #In Map
		start = req.start # In Map
		end = req.end
		closedSet = []
		openSet = [] #a heap of tuples
		heappush(openSet,(0,start))
		cameFrom = {}
		cameFrom[self.pointToIndex(start,self.map)] = None #indexed usingPointToIndex
		costSoFar = {} #indexed using pointToIndex
		costSoFar[self.pointToIndex(start,self.map)] = 0;

		while not len(openSet) == 0:
			#print("open size, ", len(openSet), "closed size", len(closedSet))

			current = heappop(openSet) #pull out just the point from the tuple

			current = current[1]
			currentIdx = self.pointToIndex(current,self.map)
			#print("current cost so far", costSoFar[currentIdx])
			if(self.distance(current,end) < .15):				

				return self.reconstruct_path(cameFrom, current,costSoFar)
				print "path found quitting"
				break

			closedSet.append(current)
			#print ("closed set size: ", len(closedSet))
			self.closedPub.publish(self.makeGridCell(closedSet))

			for nextNode in self.getNeighbors(current,1):
				nextIdx = self.pointToIndex(nextNode,self.map)
				if (nextIdx < len(self.map.data) and nextIdx > 0):
					if(self.map.data[nextIdx] == -1):
						mapData = 100
					else:
						mapData = self.map.data[nextIdx]

					
					#heur = self.localCostHeuristic(nextNode, self.local_costmap)
					heur = 0
					#discourage turning more than necessary - additional heuristic value
					prevNode = cameFrom[self.pointToIndex(current,self.map)]
					if(prevNode != None):
					 	prevAngle = math.atan2(prevNode.y - current.y, prevNode.x - current.x)
						nextAngle = math.atan2(current.y - nextNode.y, current.x - nextNode.x)
						heur += (abs(prevAngle - nextAngle) / (math.pi)) *0.01

					new_cost = 0
					if(nextNode.x != current.x and nextNode.y != current.y): #it was a diaganol movement
						new_cost += costSoFar[currentIdx] + mapData+ self.map.info.resolution*4.4141
					else:
						new_cost += costSoFar[currentIdx] + mapData + self.map.info.resolution #new cost = old cost + probability its an obstacled + 1 striaght movement
					if not nextIdx in costSoFar or new_cost < costSoFar[nextIdx]:
						costSoFar[nextIdx] = new_cost
						priority = new_cost + heur+ self.heuristic(nextNode,end) #Fn
						heappush(openSet, (priority, nextNode)) #heaps sort on first value of tuple
						self.openPub.publish(self.makeGridCell([x[1] for x in openSet]))
						#print ("open set size: ", len(openSet))
						cameFrom[nextIdx] = current




	def heapToList(self, queue):
		q = copy.deepcopy(queue)
		l = []
		while(not q.isEmpty()):
			l.append(q.pop())
		return l



	def makeGridCell(self,pointList):
		header = Header()
		header.seq = 0
		header.stamp= rospy.Time.now()
		header.frame_id = "/map"
		cell_width = self.map.info.resolution
		cell_height = self.map.info.resolution
		cells = pointList

		grid = GridCells(header,cell_width,cell_height,pointList)
		return grid


	def reconstruct_path(self,cameFrom,current,costSoFar): #current is the goal
		path = []
		header = Header()
		header.seq = 0
		header.stamp= rospy.Time.now()
		header.frame_id = "/map"
		p = Path()
		p.header = header
		gridPath=[]

		while current != None:
			pStamped = PoseStamped()
			pStamped.header = header
			pStamped.header.stamp= rospy.Time.now()
			pStamped.pose.position = current
			parent = cameFrom[self.pointToIndex(current,self.map)]


			if parent != None:
				orientation = math.atan2((current.y - parent.y),(current.x - parent.x))
				q = quaternion_from_euler(0,0,orientation)
				pStamped.pose.orientation.x = q[0]
				pStamped.pose.orientation.y = q[1]
				pStamped.pose.orientation.z = q[2]
				pStamped.pose.orientation.w = q[3]
			path.insert(0, pStamped)
			gridPath.insert(0,current)
			current = parent
		
		gridPath = gridPath[1:]
		path = path[1:]
		p.poses = path
		grid = self.makeGridCell(gridPath)
		#self.pathGrid.publish(grid)
		#print(path)

		self.pathPub.publish(p)
		return PathRequestResponse(p)

	def distance(self, p1, p2):
		return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y) **2)

	def localCostHeuristic(self,p,map1):
		index = self.pointToIndex(p,map1)
		if(index > 0 and index < len(map1.data)):
			return map1.data[index]
		else:
			return 0

	def heuristic(self,p,g):
		return 2*((p.x - g.x)**2 + (p.y - g.y) **2)





		#this function expects an xy point and returns a list of xy points
	def getNeighbors(self, point, expansionSize):
		#8Connected Neighbors
		neighbors = []
		for xMod in range(-1* expansionSize,expansionSize+1):
			for yMod in range(-1* expansionSize,expansionSize+1):
				if(not (xMod==0 and yMod==0)):
					x = point.x + xMod*self.map.info.resolution
					y = point.y + yMod*self.map.info.resolution
					z = 0
					neighbors.append(Point(x,y,z))

		return neighbors

if __name__ == "__main__":
    p = PathFinder()

    # test_service = rospy.ServiceProxy('request_path',PathRequest)
    # Start = Point(10,10,0)
    # End = Point(5,5,0)
    # test_result = test_service(Start,End)
    # print(test_result)

    # rospy.wait_for_service('request_path')
    # rospy.spin()
    while  not rospy.is_shutdown():
    	rospy.wait_for_service('request_path')
        pass
