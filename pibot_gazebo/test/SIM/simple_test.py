#!/usr/bin/env python
import unittest
import rostest
import os
import sys
import rospy
import time


class TestLaunch(unittest.TestCase):
    def setUp(self):
        rospy.init_node("simpleTestNode", anonymous=True)

    def test_run(self):
        print()
        timeout_t = rospy.get_time() + 30.0
        while not rospy.is_shutdown() and rospy.get_time() < timeout_t:
            rospy.sleep(0.1)
            topics = rospy.get_published_topics()
            if len(topics) > 0:
                print("TOPICS: ", "\n".join(map(str, rospy.get_published_topics())))
                return
        self.fail("No topics found")
        


#suite = unittest.TestSuite([TestBasic()])

class SimpleSuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SimpleSuiteTest, self).__init__()
        self.addTest(TestLaunch('test_run'))


if __name__ == '__main__':
    time.sleep(10.0)
    try:
        rostest.rosrun('pibot_gazebo', 'SIM_simple_tests', 'simple_test.SimpleSuiteTest')
    except KeyboardInterrupt:
        pass
