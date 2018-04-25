#!/usr/bin/env python
from tf.transformations import quaternion_from_euler
import rospy
from nav_msgs.msg import OccupancyGrid
#from lab3.srv import PathRequest
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from heapq import *
from nav_msgs.msg import GridCells, Path
from std_msgs.msg import *
import math
import tf
from lab3.srv import *
import copy

import cv2
import numpy as np
from matplotlib import pyplot as plt

class FrontierExplorer:
	def __init__(self):#constructor
		self.robotRadius = .135
		self.map = None
		rospy.init_node('FrontierNode')#initiate path finding node
		self._odom_list = tf.TransformListener()

		rospy.Subscriber('/map', OccupancyGrid, self.updateMap, queue_size=1) # handle nav goal events
		self.obstaclePub = rospy.Publisher("/obstacles", GridCells, queue_size =1)
		self.goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1) # handle nav goal events
		self.frontierMap = rospy.Publisher('/frontierMap', OccupancyGrid,queue_size=1)

		print "Listening for maps"


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
			kernel = np.ones((7,7), np.uint8)
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

			for i in range(1):
				self.obstaclePub.publish(grid)
			print("published Obstacles")
			self.updateNearestFrontier()

	def updateNearestFrontier(self):
		npArray = np.array(self.map.data).reshape(self.map.info.width, self.map.info.height)
		np.add(npArray, 2, npArray)
		npArray = npArray.astype("uint8")
		kernel = np.ones((3,3), np.uint8)
		npArray = cv2.erode(npArray, kernel, iterations=1)

		mask = (npArray == 1)
		unknownOnly = (npArray * mask) * 100
		mask = npArray == 102
		obstaclesOnly = npArray*mask

		kernel = np.ones((3,3), np.uint8)

		unknownEdges = cv2.Canny(unknownOnly, 0, 255)		
		#unknownEdges  = cv2.dilate(unknownEdges, kernel, iterations=1)
		obstaclesEdges = cv2.Canny(obstaclesOnly, 0, 255)
		obstaclesEdges = cv2.dilate(obstaclesEdges, kernel, iterations=1)
		
		frontierEdges = unknownEdges - obstaclesEdges
		frontierEdges = cv2.dilate(frontierEdges, kernel, iterations=1)
		#frontierEdges = cv2.erode(frontierEdges, kernel, iterations=1)
		#kernel = np.ones((3,3), np.uint8)
		# #edges  = cv2.dilate(edges, kernel, iterations=1)
		# cv2.imshow("uknownAll", unknownOnly)
		# cv2.imshow("obstaclesAll", obstaclesOnly)
		# cv2.imshow("u", unknownEdges)
		# cv2.imshow("o", obstaclesEdges)
		params = cv2.SimpleBlobDetector_Params()

		params.minThreshold = 0;
		params.maxThreshold = 256;
		params.minDistBetweenBlobs = 1;
		params.filterByColor = False;
		params.blobColor = 255;


		params.filterByArea = True
		params.minArea = 1
		params.maxArea = 1000000
		params.filterByCircularity = False
		params.filterByConvexity = False
		params.filterByInertia=False

		detector = cv2.SimpleBlobDetector_create(params)
		keypoints = detector.detect(frontierEdges)
		im_with_keypoints = cv2.drawKeypoints(frontierEdges, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 		
		# Show keypoints
		print(len(keypoints))
		cv2.imshow("Keypoints", im_with_keypoints)
		cv2.waitKey(1)

		minDistance = 10000000000
		minX = 0
		minY = 0
		minIndex = 0
		minPoint = None
		self._odom_list.waitForTransform('/odom', '/base_footprint', rospy.Time(0), rospy.Duration(1.0))
		(position, orientation) = self._odom_list.lookupTransform('/odom','/base_footprint', rospy.Time(0)) 
		

		if(len(keypoints)== 0):
			print "Mapping Completed! Please Save!"
			return

		for keypoint in keypoints:
			x = keypoint.pt[0]
			y = keypoint.pt[1]

			index = int(y*self.map.info.width + x)
			
			#point = self.indexToPoint(index)
			point = self.indexXYtoPoint(x, y)
			d = self.distance(Point(position[0],position[1],0), point)
			if(d < minDistance):
				minDistance = d
				minX = x
				minY = y
				minIndex = index
				minPoint = point
		print "xyindex: ", x, y, minIndex

		print(d)

		pStamped = PoseStamped()
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = '/map'
		pStamped.header = header
		pStamped.pose.orientation = Quaternion(0,0,0,1)

		pStamped.pose.position = point

		self.goalPub.publish(pStamped)
		print "published goal point: " , pStamped.pose.position

		frontierEdges = frontierEdges.reshape(self.map.info.width * self.map.info.height)
		frontierMap = copy.deepcopy(self.map)
		print len(frontierMap.data)

		frontierEdges = frontierEdges/2
		frontierEdges = frontierEdges.astype("int8")
		newData = frontierEdges.tolist()	
		print(len(newData))

		self.frontierMap.publish(frontierMap)


	def distance(self, p1, p2):
		return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y) **2)


	def expandObstacle(self,index):
		expansionSize = (int) (self.robotRadius / self.map.info.resolution)


		self.map.data = list(self.map.data)

		neighbors = self.getNeighbors(self.indexToPoint(index),expansionSize)
		for n in neighbors:
			if(self.pointToIndex(n,self.map) > 0 and self.pointToIndex(n,self.map) < len(self.map.data)):
				self.map.data[self.pointToIndex(n,self.map)] = self.map.data[index]
		self.map.data = tuple(self.map.data)


	def indexXYtoPoint(self,x,y):
		x = self.map.info.origin.position.x + self.map.info.resolution * x
		y = self.map.info.origin.position.y + self.map.info.resolution * y
		return Point(x,y,0)
	def xyToIndex(self,x,y,map1):
		x -= map1.info.origin.position.x
		y-= map1.info.origin.position.y
		xIndex = (int)(x/map1.info.resolution)
		yIndex = (int)(y/map1.info.resolution)
		return int(yIndex*map1.info.width + xIndex)

	def indexToPoint(self, i):
		x = (i % self.map.info.width) * self.map.info.resolution +self.map.info.origin.position.x
		y= i / self.map.info.width * self.map.info.resolution + self.map.info.origin.position.y

		return Point(x,y,0)

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
    f = FrontierExplorer()

    # test_service = rospy.ServiceProxy('request_path',PathRequest)
    # Start = Point(10,10,0)
    # End = Point(5,5,0)
    # test_result = test_service(Start,End)
    # print(test_result)

    # rospy.wait_for_service('request_path')
    # rospy.spin()
    while  not rospy.is_shutdown():
        pass