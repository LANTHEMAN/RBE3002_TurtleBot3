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

import copy

class PathFinder:
	def __init__(self):#constructor
		self.robotRadius = .135
		self.map = None
		self.local_costmap = None
		self.downSizedMap = None
		self.start = None
		self.goal = None
		rospy.init_node('request_path_server')#initiate path finding node
		s = rospy.Service('request_path', PathRequest, self.aStar)
		
		self._odom_list = tf.TransformListener()
		
		

		#publisher for path,obstacles
		self.pathPub = rospy.Publisher("/path", Path,queue_size = 1)
		self.obstaclePub = rospy.Publisher("/obstacles", GridCells, queue_size =1)
		self.pathGrid = rospy.Publisher("/pathGrid", GridCells, queue_size =1)
		self.closedPub = rospy.Publisher("/closed_set", GridCells,queue_size = 1)
		self.openPub = rospy.Publisher("/open_set", GridCells, queue_size = 1)
		
		rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.updateLocalMap, queue_size=1)
		rospy.Subscriber('/map', OccupancyGrid, self.updateMap, queue_size=1) # handle nav goal events
		while(self.map == None): 
			pass			
		#rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.updateGoal, queue_size=1)#subscribe to Rviz goal marker
		#rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,self.updateStart,queue_size=1)#subscribe to Rviz starting marker
		rospy.spin()

	def updateLocalMap(self,occGrid):
		self.local_costmap = occGrid

	def updateStart(self, p):
		poseStamped = PoseStamped()
		poseStamped.header = p.header
		poseStamped.pose = p.pose.pose

		self._odom_list.waitForTransform('/map', p.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
		newPose = self._odom_list.transformPose("/map", poseStamped)
		self.start = newPose.pose.position
		print "start updated to \n", self.start
		#print("xy index", self.pointToIndex(p.pose.pose.position))

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
		self.map = copy.deepcopy(occGrid)


		obstacles = []
		for i in range(len(occGrid.data)):	
			if(occGrid.data[i] > 50):
				self.expandObstacle(i) #this updates self.map

		for i in range(len(occGrid.data)):		#Publish the obstacles in the map
			if(self.map.data[i] > 50):
				obstacles.append(self.indexToPont(i))


		grid = self.makeGridCell(obstacles)
		#print(grid)
		# while(self.obstaclePub.get_num_connections() < 1):
		# 	print(self.obstaclePub.get_num_connections())
		

		#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		#we need to keep publishing in order to get rviz to recognize the message
		for i in range(1):
			self.obstaclePub.publish(grid)
			rospy.sleep(.5)
		print("published")



	def expandObstacleFromPoint(self,point):
		self.expandObstacle(self.pointToIndex(point,self.map));

	def expandObstacle(self,index):
		expansionSize = (int) (self.robotRadius / self.map.info.resolution)

		self.map.data = list(self.map.data)

		neighbors = self.getNeighbors(self.indexToPont(index),expansionSize)
		for n in neighbors:
			if(self.pointToIndex(n,self.map) > 0 and self.pointToIndex(n,self.map) < len(self.map.data)):
				self.map.data[self.pointToIndex(n,self.map)] = self.map.data[index]
		self.map.data = tuple(self.map.data)




		#print(occGrid.info)
		#print("\n")
	def indexToPont(self, i):
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
		newP = PoseStamped()
		newP.header.frame_id = "/map"
		newP.header.stamp = rospy.Time.now()
		newP.pose.position = p
		self._odom_list.waitForTransform(map1.header.frame_id, newP.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
		finalMapPose = self._odom_list.transformPose(map1.header.frame_id, newP)

		x = finalMapPose.pose.position.x
		y = finalMapPose.pose.position.y
		return self.xyToIndex(x,y,map1)

	def aStar(self,req):
		print "Processing Request to go from\n", req.start, "\nto\n", req.end
		self.goal = req.end
		start = req.start
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
			if(self.distance(current,end) < self.map.info.resolution):				
				return self.reconstruct_path(cameFrom, current,costSoFar)
				print "path found quitting"
				break

			closedSet.append(current)
			#print ("closed set size: ", len(closedSet))
			#self.closedPub.publish(self.makeGridCell(closedSet))

			for next in self.getNeighbors(current,1):
				nextIdx = self.pointToIndex(next,self.map)
				if (nextIdx < len(self.map.data) and nextIdx > 0):
					if(self.map.data[nextIdx] == -1):
						mapData = 100
					else:
						mapData = self.map.data[nextIdx]

					new_cost = 0
					new_cost += self.localCostHeuristic(next, self.local_costmap)

					#discourage turning more than necessary - additional heuristic value
					prevNode = cameFrom[self.pointToIndex(current,self.map)]
					if(prevNode != None):
						prevAngle = math.atan2(prevNode.y - current.y, prevNode.x - current.x)
						nextAngle = math.atan2(current.y - next.y, current.x - next.x)

						new_cost += (abs(prevAngle - nextAngle) / (math.pi)) *2

					if(next.x != current.x and next.y != current.y): #it was a diaganol movement
						new_cost += costSoFar[currentIdx] + mapData+ self.map.info.resolution
					else:
						new_cost += costSoFar[currentIdx] + mapData + self.map.info.resolution #new cost = old cost + probability its an obstacled + 1 striaght movement
					if not nextIdx in costSoFar or new_cost < costSoFar[nextIdx]:
						costSoFar[nextIdx] = new_cost
						priority = new_cost + self.heuristic(next,end) #Fn
						heappush(openSet, (priority, next)) #heaps sort on first value of tuple
						#self.openPub.publish(self.makeGridCell([x[1] for x in openSet]))
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
			return 50

	def heuristic(self,p,g):
		return (p.x - g.x)**2 + (p.y - g.y) **2





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