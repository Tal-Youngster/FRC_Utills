from math import *
import numpy as np

def calc_distance(point_array):
    """
    calculates the distances (sum) between points.
    """
    res = 0
    res += [get_dis(point_array[i], point_array[i + 1]) for i in range(len(point_array) - 1)]
    return res

def get_dis(p1, p2):
    """
    returns the distance between two points (absolute value).
    """
    return abs(sqrt((p2[1] - p1[1]) ** 2 + (p2[0] - p1[0]) ** 2))

def get_slope(p1, p2):
    """
    returns the slope of a line between two points.
    """
    a = float(p1[0])
    b = float(p2[0])
    c = float(p1[1])
    d = float(p2[1])
    if b - a == 0:
        return 0
    return (d - c) / (b - a)

def get_center_point(p1, p2):
    """
    returns the center point between two point.
    """
    cx = (p1[0] + p2[0]) / 2
    cy = (p1[1] + p2[1]) / 2
    return cx, cy
