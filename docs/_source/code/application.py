#!/usr/bin/python
import unittest
import rospy
import rostest
import sys

from atf_core import ATF

class Application:
    def __init__(self):
        self.atf = ATF()

    def execute(self):
        # Insert your ATF test blocks here
        self.atf.start("test_both1")

        self.atf.start("test_block1")
        rospy.sleep(1.0) # Placeholder
        self.atf.stop("test_block1")

        self.atf.start("test_block2")
        rospy.sleep(1.0) # Another placeholder
        self.atf.stop("test_block2")

        self.atf.stop("test_both1")
        self.atf.shutdown()

class Test(unittest.TestCase):
    def setUp(self):
        self.app = Application()

    def tearDown(self):
        pass

    def test_Recording(self):
        self.app.execute()

if __name__ == '__main__':
    rospy.init_node('turtlebot3_atf')
    if "standalone" in sys.argv:
        app = Application()
        app.execute()
    else:
        rostest.rosrun('application', 'recording', Test, sysargs=None)
