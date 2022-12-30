#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage, Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

LINES_PTS = {
    "0": (-0.01, -0.01, 0),
    "1": (0.01, 0.01, 0),
    "2": (0.01, -0.01, 0),
    "3": (-0.01, 0.01, 0),
    "4": (-0.01, 0.01, 0.01),
    "5": (0.01, 0.01, 0.01),
    "6": (0.01, -0.01, 0.01),
    "7": (-0.01, -0.01, 0.01)
}


class ArucoRec:
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        # self.loop_rate = rospy.Rate(1)
        self.mtx = []
        self.dist = []
        self.rot_mat = []
        self.trans_mat = []
        self.dim = {'width': 0, 'height': 0}
        self.border = {'width': 200, 'height': 50}
        self.default_fig = self.create_fig(0.02, 0.02, 0.02, 0.01)
        self.ID_SIZE = {
            24: self.create_fig(0.06, 0.06, 0.06, 0.06),
            21: self.create_fig(0.1, 0.1, 0.1, 0.1),
            22: self.create_fig(0.04, 0.1, 0.04, 0.06)
        }
        self.collision = Bool()
        self.known_id = {}

        # Subscribers
        self.sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback)
        # self.sub = rospy.Subscriber("/camera/rgb/image_raw", CompressedImage, self.callback)
        self.pub_image = rospy.Publisher("/camera/rgb/aruco", Image, queue_size=100)
        self.pub_collision = rospy.Publisher("/collision", Bool, queue_size=100)
        data = rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=5)
        self.get_intrisc(data)
        self.start()

    def get_intrisc(self, data):
        self.mtx = np.reshape(data.K, (3,3))
        self.dist = data.D
        self.rot_mat = data.R
        self.trans_mat = data.P
        self.dim['width'] = data.width
        self.dim['height'] = data.height

    def aruco_display(self, corners, ids, rejected, image):
        if len(corners) > 0:
        
            ids = ids.flatten()
    
            for (markerCorner, markerID) in zip(corners, ids):
                if not markerID in self.known_id:
                    if int(markerID) in self.ID_SIZE:
                        self.known_id[markerID] = self.ID_SIZE[markerID]
                    else:
                        self.known_id[markerID] = self.default_fig
                points = np.float32([self.known_id[markerID]['0'] ,self.known_id[markerID]['1'], self.known_id[markerID]['2'], self.known_id[markerID]['3'],
                                self.known_id[markerID]['4'], self.known_id[markerID]['5'], self.known_id[markerID]['6'], self.known_id[markerID]['7']])
                
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.02, self.mtx, self.dist)

                imgpts, _ = cv2.projectPoints(points, rvecs, tvecs, self.mtx, self.dist)
                if self.check_collision(imgpts):
                    print('COLLISION')
                    if not self.collision.data:
                        self.collision.data = True
                        self.pub_collision.publish(self.collision)
                elif self.collision.data == True:
                    self.collision.data = False
                    self.pub_collision.publish(self.collision)
                self.draw_lines(image, imgpts)
                # del points_dict

                print("[Inference] ArUco marker ID: {}".format(markerID))
        elif self.collision.data:
            self.collision.data = False
            self.pub_collision.publish(self.collision)
        return image

    def check_collision(self, points):
        l = [0, 3, 4, 7]
        viewed_points = 4
        for i in l:
            if points[i][0][1] < -self.border['width'] or points[i][0][1] > self.dim['height'] + self.border['width']:
              viewed_points -= 1
            if points[i][0][0] < -self.border['width'] or points[i][0][1] > self.dim['width'] + self.border['width']:
                viewed_points -= 1
        print(viewed_points)
        if viewed_points <= 2:
            return True
        
        return False

    def draw_lines(self, img, imgpts):
        imgpts = np.int32(imgpts).reshape(-1, 2)
        edges = [
            (0, 1),
            (0, 3),
            (0, 7),
            (1, 2),
            (1, 6),
            (2, 3),
            (2, 5),
            (3, 4),
            (4, 5),
            (4, 7),
            (6, 7),
            (6, 5),
        ]

        for edge in edges:
            img = cv2.line(img, tuple(imgpts[edge[0]]), tuple(imgpts[edge[1]]), (200, 0, 0), 20)

        return img

    @staticmethod
    def create_fig(w, h, l, o):
        w = w / 2
        h = h / 2
        
        points = {
            '0': [-w, -h, l + o],
            '1': [-w,  h, l + o],
            '2': [ w,  h, l + o],
            '3': [ w,  -h, l + o],
            '4': [ w,  -h,    o],
            '5': [ w,   h,    o],
            '6': [-w,   h,    o],
            '7': [-w,  -h,    o],
        }

        return points

    def callback(self, msg):
        array = np.fromstring(msg.data, np.uint8)
        self.image = cv2.imdecode(array, cv2.IMREAD_COLOR)
        # self.image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def start(self):
            aruco_type = "DICT_4X4_100"
            arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
            arucoParams = cv2.aruco.DetectorParameters_create()
            num_frame = time.time()
            while not rospy.is_shutdown():
                if self.image is not None and time.time() - num_frame > 0.1:
                    num_frame = time.time()
                    # if time.time() - num_frame < 0.5:
                    #     continue
                    frame = self.image
                    corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
                    detected_markers = self.aruco_display(corners, ids, rejected, frame)
                    image_ros = detected_markers
                    # image_ros = cv2.cvtColor(detected_markers, cv2.COLOR_BGR2GRAY)
                    image_ros = cv2.cvtColor(image_ros, cv2.COLOR_BGR2RGB)
                    height, width, _ = image_ros.shape
                    image_ros = cv2.resize(image_ros, (int(width/2) , int(height/2)))
                    image_ros = cv2.resize(image_ros, (256, 144))
                    image = self.br.cv2_to_imgmsg(image_ros, encoding="passthrough")
                    self.pub_image.publish(image)


if __name__ == '__main__':
    rospy.init_node("aruco_recognition", anonymous=True)
    aruco = ArucoRec()
