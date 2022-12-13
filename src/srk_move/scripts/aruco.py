#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys

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

POINTS = {
    "0": (-0.01, -0.01, 0),
    "1": (0.01, 0.01, 0),
    "2": (0.01, -0.01, 0.01),
    "3": (-0.01, 0.01, 0.01),
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
        self.rot_mat= []
        self.trans_mat= []

        # Subscribers
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.pub = rospy.Publisher("/camera/rgb/aruco", Image, queue_size=10)
        data = rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=5)
        self.get_intrisc(data)
        print(self.mtx)
        print(self.dist)
        print(self.rot_mat)
        print(self.trans_mat)
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
                # corners = markerCorner.reshape((4, 2))
                points = np.float32([POINTS['0'] ,POINTS['1'], POINTS['2'], POINTS['3']])
                points = np.float32([LINES_PTS['0'] ,LINES_PTS['1'], LINES_PTS['2'], LINES_PTS['3'],
                                LINES_PTS['4'], LINES_PTS['5'], LINES_PTS['6'], LINES_PTS['7']])
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.02, self.mtx, self.dist)
                # ret, rvecs, tvecs = cv2.solvePnP(points, corners, self.mtx, self.dist)

                # (topLeft, topRight, bottomRight, bottomLeft) = corners
                imgpts, _ = cv2.projectPoints(points, rvecs, tvecs, self.mtx, self.dist)
                # self.draw_lines(image, imgpts)
                self.draw_square(image, imgpts)

                # topRight = (int(topRight[0]), int(topRight[1]))
                # bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                # bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                # topLeft = (int(topLeft[0]), int(topLeft[1]))
                # print(topLeft)
                # cv2.line(image, topLeft, topRight, (0, 255, 0), 6)
                # cv2.line(image, topRight, bottomRight, (0, 255, 0), 6)
                # cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 6)
                # cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 6)
            
                # cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                # cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                # cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            
                # cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                #     0.5, (0, 255, 0), 2)
                print("[Inference] ArUco marker ID: {}".format(markerID))
            
        return image

    def draw_lines(self, img, imgpts):
        imgpts = np.int32(imgpts).reshape(-1, 2)

        l1 = [0, 0, 0, 2, 2, 6, 6, 7, 4, 3, 1, 4]
        l2 = [2, 3, 7, 1, 6, 5, 7, 4, 3, 1, 5, 5]

        for i, j in zip(l1, l2):
            img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (200, 0, 0), 20)

        return img
    def draw_square(self, img, imgpts):
        imgpts = np.int32(imgpts).reshape(-1, 2)
        l1 = [imgpts[0]]
        l2 = []
        l3 = [(255,0,0), (255,255,0), (0, 255, 0), (0, 0, 255), (0, 255, 255), (125, 0, 125)]
        # for i, j, k in zip(l1, l2, l3):
        img = cv2.fillPoly(img, pts=np.array([imgpts[4], imgpts[5], imgpts[6], imgpts[7]]).reshape((-1, 1, 2)), color=(255, 255, 0))
        return img


    def callback(self, msg):
        # rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # self.pub.publish(msg)


    def start(self):
            aruco_type = "DICT_4X4_100"
            arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
            arucoParams = cv2.aruco.DetectorParameters_create()
            while not rospy.is_shutdown():
                #if self.image is not None and self.vagabundeo.data==False:
                if self.image is not None:
                    frame = self.image
                    corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
                    detected_markers = self.aruco_display(corners, ids, rejected, frame)
                    image = self.br.cv2_to_imgmsg(detected_markers, encoding="passthrough")
                    self.pub.publish(image)

                    # Program Termination
                    cv2.imshow("Multiple Color Detection in Real-TIme", frame)
                    if cv2.waitKey(10) & 0xFF == ord('q'):
                        # cap.release()
                        cv2.destroyAllWindows()
                        break


if __name__ == '__main__':
    rospy.init_node("arco_recognition", anonymous=True)
    aruco = ArucoRec()
    # move.start_up()