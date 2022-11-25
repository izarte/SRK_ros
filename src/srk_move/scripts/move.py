#!/usr/bin/env python3

import rospy
from unity_robotics_demo_msgs.msg import PosRot
from geometry_msgs.msg import Twist 


class Move:
    def __init__(self):
        self.message = Twist()
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
        self.velocity = PosRot()
        self.start_up()

    def callback(self, data):
        # rospy.loginfo(f"Recibo: Front: {data.front_distance}, Left: {data.left_distance}, forntwall: {data.front_wall} rightwall: {data.right_wall}, .")
        rospy.loginfo(f'front: {data.pos_x}')
        speed = 2
        self.message.linear.x = data.pos_x
        self.message.linear.y = data.pos_y
        self.message.linear.z = data.pos_z

        self.message.angular.x = data.rot_x
        self.message.angular.y = data.rot_y
        self.message.angular.z = data.rot_z

        self.pub.publish(self.message)


    def start_up(self):
        rospy.init_node("move", anonymous=True)
        rospy.Subscriber("/velocity", PosRot, self.callback)
        rospy.spin()


if __name__ == '__main__':
    move = Move()
    # move.start_up()