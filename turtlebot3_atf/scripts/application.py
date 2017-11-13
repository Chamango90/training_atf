#!/usr/bin/python
import unittest
import rospy
import rostest
import sys

from atf_core import ATF

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
from math import radians, degrees

def create_nav_goal(goal_2d):
    "Create a MoveBaseGoal with x, y (in m) and yaw (in deg)."

    rospy.loginfo(type(goal_2d))
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = '/map'
    goal.target_pose.pose.position.x = goal_2d[0]
    goal.target_pose.pose.position.y = goal_2d[1]
    q = quaternion_from_euler(0.0, 0.0, radians(goal_2d[2]))
    goal.target_pose.pose.orientation = Quaternion(*q.tolist())
    return goal

def move_to_goal(move_base, goal):
    rospy.loginfo("Send goal to %s", goal.target_pose.pose)
    move_base.send_goal(goal)
    rospy.loginfo("Waiting for result...")
    move_base.wait_for_result()
    if move_base.get_state() == GoalStatus.SUCCEEDED: rospy.loginfo("Reached goal!")
    else: rospy.loginfo("Goal not reached!")



class Application:
    def __init__(self):
        self.atf = ATF()
        # Connect to the navigation action server
        self.move_base = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Connecting to /move_base action")
        self.move_base.wait_for_server()

    def execute(self):

        self.atf.start("move_to_goal")
        # Send move_base goal to robot
        move_to_goal(self.move_base, create_nav_goal(rospy.get_param("/test_goal")))
        self.atf.stop("move_to_goal")

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
