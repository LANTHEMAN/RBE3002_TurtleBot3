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
		#s = rospy.Service('request_path', PathRequest, self.aStar)
		
		self._odom_list = tf.TransformListener()
		
		#rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.updateGoal, queue_size=1)
		#rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,self.updateStart,queue_size=1)
		rospy.Subscriber('/map', OccupancyGrid, self.updateMap, queue_size=1) # handle nav goal events

		self.obstaclePub = rospy.Publisher("/obstacles", GridCells, queue_size =1)
		self.pathGrid = rospy.Publisher("/pathGrid", GridCells, queue_size =1)
		self.pathPub = rospy.Publisher("/path", Path, queue_size = 1)
		self.closedPub = rospy.Publisher("/closed_set", GridCells, queue_size=1)
		self.openPub = rospy.Publisher("/open_set", GridCells, queue_size = 1)
		
		#wait for map to publish
		while(self.map == None):
			pass

		print "Path request server ready"


	def pointToIdx(self,point):
		x = point.x
		y = point.y
		x -= self.map.info.origin.position.x
		y -= self.map.info.origin.position.y
		xIndex = (int)(x/self.map.info.resolution)
		yIndex = (int)(y/self.map.info.resolution)
		return int(yIndex*self.map.info.width + xIndex)	


	def idxToCoord(self,idx):
		x = (idx % self.map.info.width) * self.map.info.resolution +self.map.info.origin.position.x
		y= idx / self.map.info.width * self.map.info.resolution + self.map.info.origin.position.y
		z = 0
		return Point(x,y,z)


	def updateMap(self,occGrid):
		self.map = occGrid
		#Publish the obstacles on the map
		obstacles = []
		for i in range(len(self.map.data)):			
			point = self.idxToCoord(i)
			if(self.map.data[i] > 50):
				obstacles.append(point)
		grid = self.makeGridCell(obstacles)
		self.obstaclePub.publish(grid)

	#Turns a list of points into a grid cell
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

	def testCases(self):
		p = Point(0,0,0)
		assert self.pointToIdx(p) == 0
		assert self.idxToCoord(0) == p
		p= Point(.9,.9,0)
		print(self.pointToIdx(p))
		assert self.pointToIdx(p) == 0


		p = Point(36,0,0)
		assert self.pointToIdx(p) == 36
		assert self.idxToCoord(36) == p

		p = Point(1,1,0)
		print(self.idxToCoord(38))
		assert self.idxToCoord(38) == p
		assert self.pointToIdx(p) == 38
		





if __name__ == '__main__':
	print("Starting request server")

	pathFinder = PathFinder()
	print("starting tests")
	pathFinder.testCases()
	print("tests passed")

	while not rospy.is_shutdown():
		pass    
