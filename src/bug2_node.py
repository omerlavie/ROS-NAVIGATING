#!/usr/bin/python
#
# bug2_node.py
#
#  Created on: Nov 13, 2019
#      Author: Omer Lavie
#
import rospy
import sys, time
from bug2 import Bug2node

if __name__ == "__main__":
    rospy.init_node('bug2', argv=sys.argv)
    goal_x = rospy.get_param('~goal_x')
    goal_y = rospy.get_param('~goal_y')
    bug2 = Bug2node(goal_x, goal_y)
    rospy.sleep(1)
    bug2.start()
