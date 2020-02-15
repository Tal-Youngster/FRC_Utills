import json
import tf
import rospy
import numpy as np
import cv2 as cv
from math import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from FRC_utils.math_utils import get_slope


class LinearPath:
	def __init__(self, start_point=None, end_point=None):
		self.start_point = start_point
		self.end_point = end_point
		self.points_count = 50  # Default Value. TODO: FIX
		self.points_array = None
		self.ros_path = None

	def generate_ros_path(self):
		self.ros_path = Path()
		kp = []
		for p in self.points_array:
			pose = PoseStamped()
			pose.header.stamp = rospy.Time.now()
			pose.header.frame_id = "/map"
			pose.pose.position.x = p[0]
			pose.pose.position.y = p[1]
			pose.pose.position.z = 0  # TODO: change to p[2]
			kp.append(pose)
		self.ros_path.poses = kp
		self.ros_path.header.frame_id = "/map"
		self.ros_path.header.stamp = rospy.Time.now()

		return self.ros_path

	def generate_path(self):
		linear_m = get_slope(self.start_point[:2], self.end_point[:2])
		linear_b = self.start_point[1] - (linear_m * self.end_point[0])
		dx = (self.start_point[0] - self.end_point[0]) / float(self.points_count)
		self.points_array = []

		for i in range(self.points_count):
			px = self.start_point[0] - (i * dx)
			py = ((linear_m * px) + linear_b)

			self.points_array.append((px, py, 0))

		self.generate_ros_path()
		return self.ros_path
