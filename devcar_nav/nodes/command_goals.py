#!/usr/bin/env python

import rospy
import numpy as np
import tf
import math
import geometry_msgs.msg

from devcar_nav.msg import Path2D, State2D
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point, Pose2D, PoseStamped
from visualization_msgs.msg import Marker
from utils.cubic_spline_planner import *

class Planner:
	
	def __init__(self):

		self.marker = Marker()
		self.goals = 0

		# Initialise Publishers
		self.goals_viz_pub = rospy.Publisher('/viz_goals', Marker, queue_size=10)
		self.path_viz_pub = rospy.Publisher('/viz_path', Path, queue_size=10)
		self.target_path_pub = rospy.Publisher('/target_path', Path2D, queue_size=10)

		# Initialise Subscribers
		rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.clicked_goals_cb)
		# rospy.Subscriber("/camera/odom/sample", Odometry, self.pose_cb)		

		# Load parameters
		try:
			self.planner_params = rospy.get_param("/path_planner")
			self.ds = self.planner_params["distance_step"]
			self.frame_id = self.planner_params["frame_id"]
		
		except:
			raise Exception("Missing ROS parameters. Check the configuration file.")

		# Total Goals
		self.ax = []
		self.ay = []
		
		# Current Goals
		self.cx = []
		self.cy = []

	def clicked_goals_cb(self, msg):
		'''
		Fetch clicked way points
		'''
		self.goals += 1
		# Adding to master 2D waypoint list
		self.ax.append(msg.pose.position.x)
		self.ay.append(msg.pose.position.y)

		self.display_waypoint(msg.pose.position.x,msg.pose.position.y)
		
		if (self.goals >= 2):
			self.create_display_path()

	def display_waypoint(self,x,y):
		'''
		Display way points on the map
		'''
		points = Marker()		
		points.header.frame_id = "/map"	# Publish path in map frame		
		points.type = points.POINTS
		points.action = points.ADD
		points.lifetime = rospy.Duration(0)
		points.id = self.goals
		points.scale.x = 0.1
		points.scale.y = 0.1	
		points.color.a = 1.0
		points.color.r = 0.0
		points.color.g = 0.0
		points.color.b = 1.0
		points.pose.orientation.w = 1.0

		point = Point()
		point.x = x
		point.y = y
		
		points.points.append(point)

		# Publish the MarkerArray
		self.goals_viz_pub.publish(points)

	def create_display_path(self):
		'''
		Creates cubic spline polynomial path displays to map
		'''
		cx, cy, cyaw, a, b = calc_spline_course(self.ax, self.ay, self.ds)

		target_path = Path2D()
		viz_path = Path()

		viz_path.header.frame_id = "map"
		viz_path.header.stamp = rospy.Time.now()

		for n in range(0, len(cyaw)):

			# Appending to Target Path
			npose = Pose2D()
			npose.x = cx[n]
			npose.y = cy[n]
			npose.theta = cyaw[n]
			target_path.poses.append(npose)

			# Appending to Visualization Path
			vpose = PoseStamped()
			vpose.header.frame_id = self.frame_id
			vpose.header.seq = n
			vpose.header.stamp = rospy.Time.now()
			vpose.pose.position.x = cx[n]
			vpose.pose.position.y = cy[n]
			vpose.pose.position.z = 0.0
			vpose.pose.orientation.w = 1.0
			viz_path.poses.append(vpose)

		self.target_path_pub.publish(target_path)
		self.path_viz_pub.publish(viz_path)

def main():

	# Initialise the node
    rospy.init_node('rviz_goals')

    # Initialise the class
    planner = Planner()
    
    print("*********** waypoint.py: read and display way point on the map ***********")

    while not rospy.is_shutdown():
        try:
            rospy.spin()

        except KeyboardInterrupt:
            print("Shutting down ROS node...")


if __name__ == '__main__':
	main()