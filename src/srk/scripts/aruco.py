#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

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
        # self.loop_rate = rospy.Rate(50)
        self.mtx = []
        self.dist = []
        self.rot_mat = []
        self.trans_mat = []
        self.default_fig = self.create_fig(0.01, 0.01, 0.02, 0.01)
        self.ID_SIZE = {}

        # Subscribers
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.pub = rospy.Publisher("/camera/rgb/aruco", Image, queue_size=100)
        data = rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=5)
        self.get_intrisc(data)
        self.start()

    def get_intrisc(self, data):
        self.mtx = np.reshape(data.K, (3,3))
        self.dist = data.D
        self.rot_mat = data.R
        self.trans_mat = data.P

    def aruco_display(self, corners, ids, rejected, image):
        if len(corners) > 0:
        
            ids = ids.flatten()
    
            for (markerCorner, markerID) in zip(corners, ids):
                if markerID in self.ID_SIZE:
                    points_dict = self.ID_SIZE[markerID]
                else:
                    points_dict = self.default_fig
                points = np.float32([points_dict['0'] ,points_dict['1'], points_dict['2'], points_dict['3'],
                                points_dict['4'], points_dict['5'], points_dict['6'], points_dict['7']])
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.02, self.mtx, self.dist)

                imgpts, _ = cv2.projectPoints(points, rvecs, tvecs, self.mtx, self.dist)
                self.draw_lines(image, imgpts)

                print("[Inference] ArUco marker ID: {}".format(markerID))
            
        return image

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
            '3': [ w,  h, l + o],
            '4': [ w,  -h,    o],
            '5': [ w,   h,    o],
            '6': [-w,   h,    o],
            '7': [-w,  -h,    o],
        }

        return points

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def start(self):
            aruco_type = "DICT_4X4_100"
            arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
            arucoParams = cv2.aruco.DetectorParameters_create()
            while not rospy.is_shutdown():
                if self.image is not None:
                    frame = self.image
                    corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
                    detected_markers = self.aruco_display(corners, ids, rejected, frame)
                    image = self.br.cv2_to_imgmsg(detected_markers, encoding="passthrough")
                    self.pub.publish(image)

                    # Program Termination
                    # cv2.imshow("Multiple Color Detection in Real-TIme", frame)
                    if cv2.waitKey(10) & 0xFF == ord('q'):
                        # cap.release()
                        cv2.destroyAllWindows()
                        break


if __name__ == '__main__':
    rospy.init_node("aruco_recognition", anonymous=True)
    aruco = ArucoRec()
