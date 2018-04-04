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

		#publisher for path,obstacles
		self.pathPub = rospy.Publisher("/path", Path)
		self.obstaclePub = rospy.Publisher("/obstacles", GridCells, queue_size =1)
		self.pathGrid = rospy.Publisher("/pathGrid", GridCells, queue_size =1)
		self.closedPub = rospy.Publisher("/closed_set", GridCells)
		self.openPub = rospy.Publisher("/open_set", GridCells, queue_size = 1)

		while(self.map == None):
			pass			


	def updateStart(self, p):
		poseStamped = PoseStamped()
		poseStamped.header = p.header
		poseStamped.pose = p.pose.pose

		self._odom_list.waitForTransform('/map', p.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
		newPose = self._odom_list.transformPose("/map", poseStamped)
		self.start = newPose.pose.position
		print "start updated to \n", self.start
		print("xy index", self.pointToIndex(p.pose.pose.position))

	def updateGoal(self,goal):
		self._odom_list.waitForTransform('/map', goal.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
		newPose = self._odom_list.transformPose('/map', goal).pose # transform the nav goal to the global coordinate system
		self.goal = newPose.position
		request = PathRequest()
		request.start = self.start
		request.end = self.goal
		print "end updated to \n",self.goal
		self.aStar(request)

	def updateMap(self,occGrid):
		self.map = occGrid

		obstacles = []
		for i in range(len(self.map.data)):
			
			x = (i % self.map.info.width) * self.map.info.resolution +self.map.info.origin.position.x
			y= i / self.map.info.width * self.map.info.resolution + self.map.info.origin.position.y
			if(self.map.data[i] > 50):
				obstacles.append(Point(x,y,0))
		grid = self.makeGridCell(obstacles)
		self.obstaclePub.publish(grid)

		#print(occGrid.info)
		#print("\n")

	def xyToIndex(self,x,y):
		x -= self.map.info.origin.position.x
		y-= self.map.info.origin.position.y
		xIndex = (int)(x/self.map.info.resolution)
		yIndex = (int)(y/self.map.info.resolution)
		return int(yIndex*self.map.info.width + xIndex)
	def pointToIndex(self,p):
		x = p.x
		y = p.y
		return self.xyToIndex(x,y)

	def aStar(self,req):
		print "Processing Request to go from\n", req.start, "\nto\n", req.end
		start = req.start
		end = req.end
		closedSet = []
		openSet = [] #a heap of tuples
		heappush(openSet,(0,start))
		cameFrom = {}
		cameFrom[self.pointToIndex(start)] = None #indexed usingPointToIndex
		costSoFar = {} #indexed using pointToIndex
		costSoFar[self.pointToIndex(start)] = 0;

		while not len(openSet) == 0:
			#print("open size, ", len(openSet), "closed size", len(closedSet))		

			current = heappop(openSet) #pull out just the point from the tuple
			print("current priority", current[0])
			current = current[1]
			currentIdx = self.pointToIndex(current)
			print("current cost so far", costSoFar[currentIdx])
			if(self.distance(current,end) < self.map.info.resolution):
				print "path found quitting"
				self.reconstruct_path(cameFrom, current,costSoFar)
				break

			closedSet.append(current)
			self.closedPub.publish(self.makeGridCell(closedSet))

			for next in self.getNeighbors(current):
				nextIdx = self.pointToIndex(next)
				if (nextIdx < len(self.map.data) and nextIdx > 0):

					if(next.x != current.x and next.y != current.y): #it was a diaganol movement
						new_cost = costSoFar[currentIdx] + self.map.data[nextIdx] + math.sqrt(2)
					else:
						new_cost = costSoFar[currentIdx] + self.map.data[nextIdx] + 1 #new cost = old cost + probability its an obstacled + 1 striaght movement

					if not costSoFar.has_key(nextIdx) or new_cost < costSoFar[nextIdx]:
						costSoFar[nextIdx] = new_cost
						priority = new_cost + self.heuristic(next,end) #Fn
						heappush(openSet, (priority, next)) #heaps sort on first value of tuple
						self.openPub.publish(self.makeGridCell([x[1] for x in openSet]))
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
			pStamped.pose.position = current
			path.insert(0, pStamped)
			gridPath.insert(0,current)
			current = cameFrom[self.pointToIndex(current)]
		p.poses = path
		grid = self.makeGridCell(gridPath)
		self.pathGrid.publish(grid)
		#print(path)

		self.pathPub.publish(p)


	def distance(self, p1, p2):
		return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y) **2)

	def heuristic(self,p,g):
		return math.sqrt((p.x - g.x)**2 + (p.y - g.y) **2)




	def getNeighbors(self, point):
		#8Connected Neighbors
		#print('getting neighbors')
		neighbors = []
		for xMod in range(-1,2):
			for yMod in range(-1,2):
				if(not (xMod==0 and yMod==0)):
					x = point.x + xMod*self.map.info.resolution
					y = point.y + yMod*self.map.info.resolution
					z = 0
					neighbors.append(Point(x,y,z))
		return neighbors

    
if __name__ == "__main__":
    p = PathFinder()

    
    while  not rospy.is_shutdown():
        pass 