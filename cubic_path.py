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


class CubicPath:
    def __init__(self):
        self.start_point = None
        self.end_point = None
        self.multipliers = None
        self.points_count = 50  # Default value. TODO: Find function
        self.points_array = None
        self.ros_path = None
        self.points = []

    def generate_path_from_multipliers(self):
        """
        input: [(ax, ay), (bx, by), (cx, cy), (dx, dy)]
        takes as input multipliers from generate_spline_multipliers() and return an array of points.
        """
        (ax, ay), (bx, by), (cx, cy), (dx, dy) = self.multipliers

        for s in range(self.points_count):
            p = s / float(self.points_count)
            point = (ax * pow(p, 3) + bx * pow(p, 2) + cx * p + dx, ay * pow(p, 3) + by * pow(p, 2) + cy * p + dy)
            self.points.append(point)

    def generate_ros_path(self):
        kp = []
        self.ros_path = Path()
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

    def generate_spline(self, start_point=None, end_point=None, multipliers=None):
        self.start_point = start_point
        self.end_point = end_point
        self.multipliers = multipliers

        # Case 1: start point and end point
        if start_point is not None and end_point is not None and multipliers is None:
            x1, y1, theta1 = start_point
            x2, y2, theta2 = end_point
            dx = x2 - x1
            dy = y2 - y1
            distance = sqrt((dx * dx) + (dy * dy))
            k = distance / 2
            xDerB, xDerF = k * cos(theta1), k * cos(theta2)
            yDerB, yDerF = k * sin(theta1), k * sin(theta2)

            AInv = np.array([
                [2, 1, -2, 1],
                [-3, -2, 3, -1],
                [0, 1, 0, 0],
                [1, 0, 0, 0]
            ])
            Bx = [x1, xDerB, x2, xDerF]
            By = [y1, yDerB, y2, yDerF]

            poliXCoefs = np.dot(AInv, Bx)
            poliYCoefs = np.dot(AInv, By)

            self.multipliers = [(poliXCoefs[i], poliYCoefs[i]) for i in range(4)]

        if self.multiplers is not None:
            self.generate_path_from_multipliers()
            self.generate_ros_path()

            return self.ros_path

        else:
            return None
