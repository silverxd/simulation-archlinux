#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
from math import pi

RIGHT = "right"
LEFT = "left"


class ImuConverter:
    def __init__(self):
        rospy.init_node("imu_script")

        rospy.Subscriber("/robot/imu/raw", Imu, self.publish_angle)
        rospy.Subscriber("/robot/wheel/left/vel_cmd",
                         Float32, self.save_left_wheel_speed)
        rospy.Subscriber("/robot/wheel/right/vel_cmd",
                         Float32, self.save_right_wheel_speed)

        self.publisher = rospy.Publisher(
            "/robot/imu/angle", Float32, queue_size=1)
        self.left_wheel_speed = 0
        self.right_wheel_speed = 0
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

    def save_left_wheel_speed(self, data):
        self.left_wheel_speed = data.data

    def save_right_wheel_speed(self, data):
        self.right_wheel_speed = data.data

    def set_difference(self, angle):
        side = None
        prev_side_and_angle_exist = self.prev_side is not None and self.prev_angle is not None

        if self.right_wheel_speed > self.left_wheel_speed:
            side = LEFT
            if prev_side_and_angle_exist and self.prev_side == side and angle < self.prev_angle - 0.1:
                self.difference += 2 * pi
        elif self.left_wheel_speed > self.right_wheel_speed:
            side = RIGHT
            if prev_side_and_angle_exist and self.prev_side == side and angle > self.prev_angle + 0.1:
                self.difference -= 2 * pi
        self.prev_side = side

    def convert(self, angle):
        self.set_difference(angle)
        self.prev_angle = angle

        return angle + self.difference


if __name__ == '__main__':
    ImuConverter()
