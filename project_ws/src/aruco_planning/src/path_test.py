#!/usr/bin/env python
"""
Path Planning Script for Lab 5
Author: Tiffany Cappellari
"""
import sys

from baxter_interface import Limb

import rospy
import numpy as np
import traceback
# import controller

from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner

# Uncomment this line for part 5 of Lab 5
from controller import Controller


def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    plannerRight = PathPlanner("right_arm")
    plannerLeft = PathPlanner("left_arm")

	# K values for Part 5
    Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Borrowed from 106B Students
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Borrowed from 106B Students
    Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned

	# Initialize the controller for Part 5
	# controller = Controller( . . . )

    #-----------------------------------------------------#
    ## Add any obstacles to the planning scene here
    #-----------------------------------------------------#
    # posStamp1 = PoseStamped()
    # posStamp1.header.frame_id = "base"
    # posStamp1.pose.position.x = 0.5
    # posStamp1.pose.position.y = 0
    # posStamp1.pose.position.z = 0
    # posStamp1.pose.orientation.x = 0
    # posStamp1.pose.orientation.y = 0
    # posStamp1.pose.orientation.z = 0
    # posStamp1.pose.orientation.w = 1
    # planner.add_box_obstacle([0.4, 1.2, 0.1], "table1", posStamp1)
    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    control1 = Controller(Kp, Ki, Kd, Kw, Limb("right"))
    control2 = Controller(Kp, Ki, Kd, Kw, Limb("left"))

    def move_to_goal(x, y, z, controller, planner, orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
        while not rospy.is_shutdown():
            try:
                goal = PoseStamped()
                goal.header.frame_id = "base"
                #x, y, and z position
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = z

		    #Orientation as a quaternion
                goal.pose.orientation.x = or_x
                goal.pose.orientation.y = or_y
                goal.pose.orientation.z = or_z
                goal.pose.orientation.w = or_w

                plan = planner.plan_to_pose(goal, orien_const)

                raw_input("Press <Enter> to move the right arm to goal pose: ")

                # Might have to edit this for part 5
                if not controller.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

    while not rospy.is_shutdown():

    # Set your goal positions here
        move_to_goal(0.3,0.5,0,control2, plannerLeft)
    	move_to_goal(0.47, -0.85, 0.07, control1, plannerRight)
        move_to_goal(0.6, -0.3, 0.0, control1, plannerRight)
        move_to_goal(0.6, -0.1, 0.1, control1, plannerRight)

        #Set the left hand pos
    control1.shutdown()
    control2.shutdown()

        

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
