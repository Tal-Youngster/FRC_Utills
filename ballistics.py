import numpy as np
from math import *
from FRC_utils.ros_utils import generate_ros_path
from FRC_utils.math_utils import get_dis
from FRC_utils.linear_path import LinearPath

#  constants
gravity = 9.7803  # m / s ^ 2


def calculate_vector(point_a, point_b):
    """
    takes two [x, y, z] points, and calculates the velocity and angle for shooting
    an object from point_a to point_b, no air friction included. The object will
    be vertical to the floot at point_b.

    input:
    point_a = [x, y, z] (meters)
    point_b = [x, y, z] (meters)

    output:
    alpha - float radians
    velocity - float m / s.
    """
    dh = point_b[2] - point_a[2]
    dx = get_dis(point_a, point_b)
    alpha = atan((2 * float(dh)) / float(dx))
    velocity = sqrt((gravity * float(dx)) / (cos(alpha) * sin(alpha)))
    return alpha, velocity


def velocities_to_vector(vx, vy):
    """
    transforms velocities to vector.

    input:
    vx - velocity at x axis (m / s)
    vy - velocity at y axis (m / s)

    output:
    alpha - float radians
    velocity - float m / s.
    """
    alpha = atan2(vx, vy)
    velocity = sqrt( (vx**2) + (xv**2))
    return alpha, velocity

def vector_to_velocities(vector):
    """
    transforms vector to velocities.

    input:
    alpha - float radians
    velocity - float m / s.

    input:
    vx - velocity at x axis (m / s)
    vy - velocity at y axis (m / s)
    """
    vx = cos(alpha) * velocity
    vy = sin(alpha) * velocity
    return vx, vy


class BallisticsGraph:
    """
    VISUALIZATION TOOL

    3D visualization of ballistic action.
    takes the start point and end point of the action.

    see: calculate_vector()

    points_array = an array of [x, y, z] points at the length of 'self.points_count',
    describing the path for the object.

    get_location_at_time() = returns the location of the object at a specified
    time.

    get_ros_path() = returns a ROS Path object built from the 'self.points_array'.

    """
    def __init__(self, start_point, end_point):
        """
        start_point = [x, y, z] (meters)
        end_point = [x, y, z] (meters)
        """
        self.max_time = 0

        self.points_count = 50  # TODO: Default value
        self.points_array = None

        self.z1, self.z2 = start_point[2], end_point[2]
        self.x1, self.x2 = start_point[0], end_point[0]
        self.y1, self.y2 = start_point[1], end_point[1]

        self.alpha, self.velocity = calculate_vector(start_point, end_point)
        self.generate_graph()

    def generate_graph(self):
        self.points_array = []

        dx = self.x2 - self.x1
        dy = self.y2 - self.y1
        dz = self.z2 - self.z1

        dis = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
        self.max_time = dis / self.velocity

        for i in range(self.points_count):
            x = self.x1 + (dx * i / self.points_count)
            y = self.y1 + (dy * i / self.points_count)
            y_temp = y - self.y1
            x_temp = x - self.x1
            dis = get_dis((0, 0), (y_temp, x_temp))
            z = self.z1 + (dis * tan(self.alpha)) - ((gravity * (dis**2)) / (2 * (self.velocity**2) * (cos(self.alpha)**2)))

            self.points_array.append([x, y, z])

    def get_location_at_time(self, time):
        if time > self.max_time:
            return self.points_array[-1]
        if time < 0:
            return self.points_array[0]

        dx = self.x2 - self.x1
        dy = self.y2 - self.y1
        dz = self.z2 - self.z1

        vx = dx / self.max_time
        vy = dy / self.max_time

        x = self.x1 + (time * vx)
        y = self.y1 + (time * vy)
        y_temp = y - self.y1
        x_temp = x - self.x1
        dis = get_dis((0, 0), (y_temp, x_temp))
        z = self.z1 + (dis * tan(self.alpha)) - ((gravity * (dis**2)) / (2 * (self.velocity**2) * (cos(self.alpha)**2)))

        return [x, y, z]

    def get_ros_path(self):
        ros_path = generate_ros_path(self.points_array)
        return ros_path
