#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from math import pi

RIGHT = "right"
LEFT = "left"


class ImuConverter:
    def __init__(self):
        rospy.init_node("imu_script")

        # Subscriber
        rospy.Subscriber("/robot/imu/raw", Imu, self.publish_angle)

        # Publisher
        self.publisher = rospy.Publisher(
            "/robot/imu/angle", Float32, queue_size=1)

        # Variables
        self.prev_angle = None
        self.prev_side = None
        self.difference = 0

        rospy.spin()

    def get_angle(self, data):
        o = data.orientation
        quaternion = (o.x, o.y, o.z, o.w)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def publish_angle(self, data):
        angle = self.get_angle(data)
        self.publisher.publish(self.convert(angle))

    def convert(self, angle):
        if self.prev_angle is None:
            self.prev_angle = angle

        if self.prev_angle - angle > pi:
            self.difference += 2 * pi
        elif angle - self.prev_angle > pi:
            self.difference -= 2 * pi

        self.prev_angle = angle
        return angle + self.difference


if __name__ == '__main__':
    ImuConverter()
