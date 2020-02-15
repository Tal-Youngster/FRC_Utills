import cv2 as cv
import numpy as np
from math import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from FRC_utils.math_utils import get_dis, get_slope
from threading import Thread
from scipy.spatial import distance as dist
import time


def get_x(point):
    return point[0]

def get_y(point):
    return point[1]

def draw_axis(img, rotate, tranlate, cam_mat, dist_coeff):
    # unit is mm
    img = img.copy()
    rotV, _ = cv.Rodrigues(rotate)
    points = np.float32([[100, 0, 0], [0, 100, 0], [0, 0, 100], [0, 0, 0]]).reshape(-1, 3)
    axisPoints, _ = cv.projectPoints(points, rotV, tranlate, cam_mat, dist_coeff)
    try:
        img = cv.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[0].ravel()), (255,0,0), 3)
        img = cv.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[1].ravel()), (0,255,0), 3)
        img = cv.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[2].ravel()), (0,0,255), 3)
    except:
        pass
    return img, axisPoints[3].ravel()

def order_points(pts):
    # sort the points based on their x-coordinates
    xSorted = pts[np.argsort(pts[:, 0]), :]

    # grab the left-most and right-most points from the sorted
    # x-coordinate points
    leftMost = xSorted[:2, :]
    rightMost = xSorted[2:, :]

    # now, sort the left-most coordinates according to their
    # y-coordinates so we can grab the top-left and bottom-left
    # points, respectively
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, bl) = leftMost

    # now that we have the top-left coordinate, use it as an
    # anchor to calculate the Euclidean distance between the
    # top-left and right-most points; by the Pythagorean
    # theorem, the point with the largest distance will be
    # our bottom-right point
    D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    (br, tr) = rightMost[np.argsort(D)[::-1], :]

    # return the coordinates in top-left, top-right,
    # bottom-right, and bottom-left order
    return np.array([tl, tr, br, bl], dtype="int32")

def get_mass_dir(cnt):
    rect = cv.minAreaRect(cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    max_num = 0
    a = 0
    for i in range(3):
        dis = get_dis(box[i], box[i + 1])
        if dis > max_num:
            max_num = dis
            a = get_slope(box[i], box[i + 1])
    return box, a


def show_images(images_array):
    for i, image in enumerate(images_array):
        if image is not None:
            try:
                cv.imshow(str(i), image)
            except Exception as exc:
                print(exc)
        cv.waitKey(1)


def publish_image(image, publisher):
    bridge = CvBridge()
    try:
        publisher.publish(bridge.cv2_to_imgmsg(image, 'rgb8'))
    except Exception as exc:
        print('wrong format: \n', str(exc))


class LogitechC922(Thread):
    def __init__(self, config):
        super(LogitechC922, self).__init__()

        self.config = config
        self.is_running = False

        self.cap = None
        self.mat = None

        self.start()

    def run(self):
        self.cap = cv.VideoCapture(self.config['PORT'])
        self.cap.set(3, int(self.config['FRAME_WIDTH']))
        self.cap.set(4, int(self.config['FRAME_HEIGHT']))
        self.cap.set(5, int(self.config['FPS']))
        self.cap.set(10 , int(self.config['BRIGHTNESS']))
        time.sleep(2)
        self.cap.set(15, float(self.config['EXPOSURE']))

        self.is_running = True
        while self.is_running:
            ret, self.mat = self.cap.read()

    def close(self):
        self.is_running = False
        cv.destroyAllWindows()
        cv.VideoCapture(self.config['PORT']).release()


class AstraCallback:
    def __init__(self):
        self.bridge1 = CvBridge()
        self.bridge2 = CvBridge()
        self.ir_sub = rospy.Subscriber('/camera/ir/image', Image, self.ir_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth_registered/image_raw', Image, self.depth_callback)
        self.ir = None
        self.depth = None

    def ir_callback(self, data):
        try:
            data.encoding = "mono16"
            self.ir = self.bridge1.imgmsg_to_cv2(data, "mono16")
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
            # print(data)
            depth = self.bridge2.imgmsg_to_cv2(data, "passthrough")
            self.depth = depth.astype(np.float32)

        except CvBridgeError as e:
            print(e)
