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


arrow_scale = Vector3(.25, .12, .06)

def get_config():
	conf_path = '/home/than/catkin_ws/src/FRC_2020_ROBOT/nodes/config.json'
	config = json.loads(open(conf_path).read())
	return config


def generate_ros_path(points_array):
	ros_path = Path()
	kp = []
	for p in points_array:
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "/map"
		pose.pose.position.x = p[0]
		pose.pose.position.y = p[1]
		pose.pose.position.z = p[2]
		kp.append(pose)
	ros_path.poses = kp
	ros_path.header.frame_id = "/map"
	ros_path.header.stamp = rospy.Time.now()

	return ros_path


def create_pose(position, frame):
	pose = PoseStamped()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = frame
	pose.pose.position.x = position[0]
	pose.pose.position.y = position[1]
	pose.pose.position.z = position[2]
	pose.pose.orientation.x = position[3]
	pose.pose.orientation.y = position[4]
	pose.pose.orientation.z = position[5]
	pose.pose.orientation.w = 1.0
	return pose

def generate_random_color():
    color = list(np.random.choice(range(256), size=3))
    return color

def make_arrow_marker(position, orientation, color, frame, name):
    """
    :param name: namespace of ros frame
    :param vector: [x, y, z, roll, pitch, yaw]
    :return: Marker object of an arrow
    """
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = frame
    m.header.stamp = rospy.Time.now()
    m.ns = name
    m.id = 0
    m.type = Marker.ARROW
    m.pose.position.x = position[0]
    m.pose.position.y = position[1]
    m.pose.position.z = position[2]

    m.pose.orientation.x = orientation[0]
    m.pose.orientation.y = orientation[1]
    m.pose.orientation.z = orientation[2]
    m.pose.orientation.w = 1.0
    m.color.a = 1.0
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.scale = arrow_scale
    return m

def arrow_from_pose(pose, color, frame, name):
	m = Marker()
	m.action = Marker.ADD
	m.type = Marker.ARROW
	m.header.frame_id = frame
	m.header.stamp = rospy.Time.now()
	m.ns = name
	m.id = 0

	m.pose = pose
	m.color.a = 1.0
	m.color.r = color[0]
	m.color.g = color[1]
	m.color.b = color[2]
	m.scale = arrow_scale
	return m

def make_box_marker(place, size, name):
    """
    :param size:
    :param name: namespace of ros frame
    :param place: [x, y, z, roll, pitch, yaw]

    :return: Marker object of an arrow
    """
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = '/world'
    m.header.stamp = rospy.Time.now()
    m.ns = name
    m.id = 0
    m.type = Marker.CUBE
    m.pose.position.x = place[0]
    m.pose.position.y = place[1]
    m.pose.position.z = place[2]

    m.scale.x = size[0]
    m.scale.y = size[1]
    m.scale.z = size[2]

    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0
    m.color.a = 0.4
    m.color.r = 0.8
    m.color.g = 0.8
    m.color.b = 0.8
    return m

def make_sphere_marker(vector, size, name, color):
    """
    :param color:
    :param name: namespace of ros frame
    :param size: radius in x, y, z axis
    :param vector: [x, y, z]
    :return: Marker object of an arrow
    """
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = '/world'
    m.header.stamp = rospy.Time.now()
    m.ns = name
    m.id = 0
    m.type = Marker.SPHERE
    m.pose.position.x = vector[0]
    m.pose.position.y = vector[1]
    m.pose.position.z = vector[2]

    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1
    m.color.a = 1.0
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.scale = Vector3(size[0], size[1], size[2])
    return m
