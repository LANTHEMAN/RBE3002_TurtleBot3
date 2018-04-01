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
	def __init__(self):
		self.map = None
		self.start = None
		self.goal = None
		rospy.init_node('request_path_server')
		s = rospy.Service('request_path', PathRequest, self.aStar)
		self.closedPub = rospy.Publisher("/closed_set", GridCells)
		self.openPub = rospy.Publisher("/open_set", GridCells, queue_size = 1)
		self._odom_list = tf.TransformListener()
		self.pathPub = rospy.Publisher("/path", Path)
		rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.updateGoal, queue_size=1)
		rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,self.updateStart,queue_size=1)
		rospy.Subscriber('/map', OccupancyGrid, self.updateMap, queue_size=1) # handle nav goal events
		while(self.map == None):
			pass

		print self.map.data

		print "Path request server ready"
		testReq = PathRequest()
		rospy.spin()

	def updateStart(self, p):
		poseStamped = PoseStamped()
		poseStamped.header = p.header
		poseStamped.pose = p.pose.pose

		self._odom_list.waitForTransform('/map', p.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
		newPose = self._odom_list.transformPose("/map", poseStamped)
		self.start = newPose.pose.position
		print "start updated to \n", self.start

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
		print(occGrid.info)
		print("\n")

	def xyToIndex(self,x,y):
		#x -= self.map.info.origin.position.x
		#y-= self.map.info.origin.position.y
		xIndex = x/self.map.info.resolution
		yIndex = y/self.map.info.resolution
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
		openSet = []
		

		cameFrom = [None for point in self.map.data]
		gCost = [100 for point in self.map.data]
		fCost = [None for point in self.map.data]

		gCost[self.pointToIndex(start)] = 0
		fCost[self.pointToIndex(start)] = 0
		heappush(openSet,(0,start))

		while not len(openSet) == 0:
			print("open size, ", len(openSet), "closed size", len(closedSet))


			self.closedPub.publish(self.makeGridCell(closedSet))
			self.openPub.publish(self.makeGridCell([x[1] for x in openSet]))


			current = heappop(openSet)[1] #pull out just the point from the tuple
			if(self.distance(current,end) < self.map.info.resolution):
				print "path found quitting"
				self.reconstruct_path(cameFrom, current)
				break
			closedSet.append(current)

			for n in self.getNeighbors(current):
				if n in closedSet:
					continue
					#currnet cost plus cost to get to neighbor
				tempGCost = gCost[self.pointToIndex(current)] + max(.01, self.map.data[self.pointToIndex(n)]*100)
				print(self.map.data[self.pointToIndex(n)])

				if cameFrom[self.pointToIndex(n)] == None:
					totalCost = self.heuristic(n,req.end) + tempGCost
					heappush(openSet,(totalCost,n))
				elif tempGCost >= gCost[self.pointToIndex(n)]: #already a shorter path to get here dont update value
					continue

				#best path so far
				cameFrom[self.pointToIndex(n)] = current
				gCost[self.pointToIndex(n)] = tempGCost
				fCost[self.pointToIndex(n)] = tempGCost + self.heuristic(n,req.end)
				
		print "no path found"

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
		

	def reconstruct_path(self,cameFrom,current): #current is the goal
		path = []
		header = Header()
		header.seq = 0
		header.stamp= rospy.Time.now()
		header.frame_id = "/map"
		p = Path()
		p.header = header
		while current != None:
			pStamped = PoseStamped()
			pStamped.header = header
			pStamped.pose.position = current
			path.insert(0, pStamped)
			current = cameFrom[self.pointToIndex(current)]
		p.poses = path
		print(path)
		self.pathPub.publish(p)


	def distance(self, p1, p2):
		return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y) **2)

	def heuristic(self,p,g):
		return math.sqrt((p.x - g.x)**2 + (p.y - g.y) **2)




	def getNeighbors(self, point):
		#8Connected Neighbors
		print('getting neighbors')
		neighbors = []
		for xMod in range(-1,2):
			for yMod in range(-1,2):
				if(not (xMod==0 and yMod==0)):
					x = point.x + xMod*self.map.info.resolution
					y = point.y + yMod*self.map.info.resolution
					z = 0
					neighbors.append(Point(x,y,z))
		print('done')
		return neighbors
    
if __name__ == "__main__":
    p = PathFinder()