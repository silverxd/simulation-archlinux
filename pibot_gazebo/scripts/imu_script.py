#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32

def callback(data, publisher):
    publisher.publish(get_angle(data))

def main():
    rospy.init_node("imu_script")
    publisher = rospy.Publisher("/robot/imu/angle", Float32, queue_size=1)
    rospy.Subscriber("/robot/imu/raw", Imu, (lambda publisher: lambda data: callback(data, publisher))(publisher))
    rospy.spin()

def get_angle(data):
    o = data.orientation
    quaternion = (o.x, o.y, o.z, o.w)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    return yaw

if __name__ == '__main__':
    main()