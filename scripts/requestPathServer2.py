#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from lab3.srv import PathRequest
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from heapq import *
from nav_msgs.msg import GridCells, Path
from std_msgs.msg import *
import math
import tf

import copy

class PathFinder:
	def __init__(self):#constructor
		self.map = None
		self.start = None
		self.goal = None
		rospy.init_node('request_path_server')#initiate path finding node
		s = rospy.Service('request_path', PathRequest, self.aStar)
		
		self._odom_list = tf.TransformListener()
		
		rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.updateGoal, queue_size=1)#subscribe to Rviz goal marker
		rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,self.updateStart,queue_size=1)#subscribe to Rviz starting marker
		rospy.Subscriber('/map', OccupancyGrid, self.updateMap, queue_size=1) # handle nav goal events

		#publisher for path,obstacles,close set & open set to Rviz to show 
		self.pathPub = rospy.Publisher("/path", Path)
		self.obstaclePub = rospy.Publisher("/obstacles", GridCells, queue_size =1)
		self.pathGrid = rospy.Publisher("/pathGrid", GridCells, queue_size =1)
		self.closedPub = rospy.Publisher("/closed_set", GridCells)
		self.openPub = rospy.Publisher("/open_set", GridCells, queue_size = 1)

		while(self.map == None): #waiting for map to update
			pass			


	def updateStart(self, p):#update starting position 
		#create message header
		poseStamped = PoseStamped()
		poseStamped.header = p.header
		poseStamped.pose = p.pose.pose

		self._odom_list.waitForTransform('/map', p.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
		newPose = self._odom_list.transformPose("/map", poseStamped)
		self.start = newPose.pose.position
		print "start updated to \n", self.start

	def updateGoal(self,goal):#update goal position 
		self._odom_list.waitForTransform('/map', goal.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
		newPose = self._odom_list.transformPose('/map', goal).pose # transform the nav goal to the global coordinate system
		self.goal = newPose.position
		request = PathRequest()
		request.start = self.start
		request.end = self.goal
		print "end updated to \n",self.goal
		self.aStar(request)#invoke path finding

	def updateMap(self,occGrid):#update map when received 
		self.map = occGrid

		obstacles = []
		for i in range(len(self.map.data)):#load all obstacles 
			x = (i % self.map.info.width) * self.map.info.resolution +self.map.info.origin.position.x
			y= i / self.map.info.width * self.map.info.resolution + self.map.info.origin.position.y
			if(self.map.data[i] > 50):
				obstacles.append(Point(x,y,0))
		grid = self.makeGridCell(obstacles)
		self.obstaclePub.publish(grid)#publish obstacle to Rviz




	def pointToIndex(self,p):#convert X, Y coordinate to indexing 
		x = p.x
		y = p.y
		x -= self.map.info.origin.position.x #account for offset 
		y-= self.map.info.origin.position.y
		xIndex = (int)(x/self.map.info.resolution)
		yIndex = (int)(y/self.map.info.resolution)
		return int(yIndex*self.map.info.width + xIndex)

	def aStar(self,req): #path finding 
		print "Processing Request to go from\n", req.start, "\nto\n", req.end
		start = req.start #initializing start & end nodes
		end = req.end
		closedSet = [] #visited nodes 
		openSet = [] #frontier, a heap of tuples
		heappush(openSet,(0,start))#insert the starting node to queue
		cameFrom = {}#dictonary of node parents
		cameFrom[self.pointToIndex(start)] = None #indexed usingPointToIndex
		costSoFar = {} #indexed using pointToIndex
		costSoFar[self.pointToIndex(start)] = 0;

		while not len(openSet) == 0: #while the current node has children
			#print("open size, ", len(openSet), "closed size", len(closedSet))		

			current = heappop(openSet) #pull out just the point from the tuple
			print("current priority", current[0]) 
			current = current[1] #g cost of current node
			currentIdx = self.pointToIndex(current)#index of current node
			print("current cost so far", costSoFar[currentIdx])
			if(self.distance(current,end) < self.map.info.resolution): #check if reached goal 
				print "path found quitting"
				self.reconstruct_path(cameFrom, current,costSoFar)#print path
				break

			closedSet.append(current) #append current to visited 
			self.closedPub.publish(self.makeGridCell(closedSet))#publish visited node 

			for next in self.getNeighbors(current):#visit current's children 
				nextIdx = self.pointToIndex(next)
				if (nextIdx < len(self.map.data) and nextIdx > 0): #if next index is inbound of map 

					if(next.x != current.x and next.y != current.y): #it was a diaganol movement
						new_cost = costSoFar[currentIdx] + self.map.data[nextIdx] + math.sqrt(2)#new cost = old cost + probability its an obstacled + 1 diagonal movement
					else:
						new_cost = costSoFar[currentIdx] + self.map.data[nextIdx] + 1 #new cost = old cost + probability its an obstacled + 1 striaght movement

					if not costSoFar.has_key(nextIdx) or new_cost < costSoFar[nextIdx]:#if it is a new node or a short path
						costSoFar[nextIdx] = new_cost #update new node cost 
						priority = new_cost + self.heuristic(next,end) #Fn
						heappush(openSet, (priority, next)) #heaps sort on first value of tuple
						self.openPub.publish(self.makeGridCell([x[1] for x in openSet]))#publish openset nodes
						cameFrom[nextIdx] = current#set new node parent to current

	
	def makeGridCell(self,pointList): #make a grid cell message 
		#write Header
		header = Header()
		header.seq = 0
		header.stamp= rospy.Time.now()
		header.frame_id = "/map"
		cell_width = self.map.info.resolution
		cell_height = self.map.info.resolution
		cells = pointList #nodes 

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
		while current != None: # looping through parents, write the path into gridPath
			pStamped = PoseStamped()
			pStamped.header = header
			pStamped.pose.position = current
			path.insert(0, pStamped)
			gridPath.insert(0,current)
			current = cameFrom[self.pointToIndex(current)] 
		p.poses = path
		grid = self.makeGridCell(gridPath)
		self.pathGrid.publish(grid)#publish path
		#print(path)

		self.pathPub.publish(p)#publish path in grid form 


	def distance(self, p1, p2): #calculate the euclidean distance between 2 points  
		return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y) **2)

	def heuristic(self,p,g): ##calculate h(n) of a given point
		return math.sqrt((p.x - g.x)**2 + (p.y - g.y) **2)




	def getNeighbors(self, point): #get neighbor nodes of a node 
		#8Connected Neighbors
		#print('getting neighbors')
		neighbors = []
		for xMod in range(-1,2):
			for yMod in range(-1,2):
				if(not (xMod==0 and yMod==0)):
					x = point.x + xMod*self.map.info.resolution #current.x +- 1
					y = point.y + yMod*self.map.info.resolution #current.y +- 1
					z = 0
					neighbors.append(Point(x,y,z))#record all neighbors
		return neighbors

    
if __name__ == "__main__":
    p = PathFinder()

    
    while  not rospy.is_shutdown():
        pass 
