#!/usr/bin/env python
from tf.transformations import quaternion_from_euler
import rospy
from nav_msgs.msg import OccupancyGrid
from lab3.srv import PathRequest
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from heapq import *
from nav_msgs.msg import GridCells, Path
from std_msgs.msg import *
import math
import tf
from lab3.srv import *
import copy




if __name__ == "__main__":
    test_service = rospy.ServiceProxy('request_path',PathRequest)
    Start = Point(10,10,0)
    End = Point(5,5,0)
    while(1):
        test_result = test_service(Start,End)
        print(test_result.path)
        rospy.sleep(100)

    rospy.wait_for_service('request_path')
    rospy.spin()
    while  not rospy.is_shutdown():
        rospy.wait_for_service('request_path')
        pass 