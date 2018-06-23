#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

def publish_intensity(image, publisher):
    if image.data is not None:
        publisher.publish(normalize(get_intensity(image)))

def main():
    rospy.init_node("line_sensor_script")
    topics = rospy.get_published_topics("/robot/line_sensor")

    for topic in topics:
        if topic[1] == "sensor_msgs/Image":
            topic = topic[0]
            topic_list = topic.split("/")
            topic_prefix = "/".join(topic_list[1:-1])
            publisher = rospy.Publisher("/"+topic_prefix+"/intensity", Float32, queue_size=1)
            rospy.Subscriber(topic, Image, (lambda publisher: lambda image: publish_intensity(image, publisher))(publisher))

    rospy.spin()

def normalize(intensity, max_in=255, max_out=950):
    return (intensity / max_in) * max_out


def get_intensity(image):
    intensity_sum = 0
    count = len(image.data)
    for pixel in image.data:
        intensity_sum += ord(pixel)

    return intensity_sum/count

if __name__ == '__main__':
    main()