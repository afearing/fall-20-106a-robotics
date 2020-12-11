#!/usr/bin/env python
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
# import cv2 # OpenCV library
from sensor_msgs.msg import Image
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialTransform
from geometry_msgs.msg import Vector3 
# import cv2.aruco as aruco

#Controller Imports
from path_planner import PathPlanner
# Uncomment this line for part 5 of Lab 5
import tf2_ros
import tf2_geometry_msgs

# Callback function to subscribe to images
def fiducial_callback(fiducial_tfArray):
    tfBuffer1 = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer1)

    vector3 = fiducial_tfArray.transforms[0].transform.translation
    quaternion = fiducial_tfArray.transforms[0].transform.rotation
    frame = fiducial_tfArray.header

    control(vector3.x, vector3.y, vector3.z, -.5, 0.5, -.5, 0.5)

def control(x_in, y_in, z_in, wx_in, wy_in, wz_in, ww_in):
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    plannerRight = PathPlanner("right_arm")
    plannerLeft = PathPlanner("left_arm")

    def move_to_goal(x, y, z, or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0, planner=plannerRight, orien_const=[]):
        while not rospy.is_shutdown():
            print("x,y,z in: ", x, " ", y, " ", z)
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
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break
    # print("coords in: ", x_in, y_in, z_in)
    while not rospy.is_shutdown():
        # input1 = float(raw_input("Please enter 3 points: "))
        # input2 = float(raw_input("Please enter 3 points: "))
        # input3 = float(raw_input("Please enter 3 points: "))
        # move_to_goal(input1, input2, input3, control2, plannerLeft)
        # move_to_goal(-0.62,0.5,0.24,control2, plannerLeft)
        # move_to_goal(0.3,0.5,0,control2, plannerLeft)
        move_to_goal(x_in, y_in, z_in, wx_in, wy_in, wz_in, ww_in)
        # move_to_goal(1.1,0.5,0,control2, plannerLeft, [], -.5, 0.5, -.5, 0.5)
        # move_to_goal(1.1,0.5,.23,control2, plannerLeft, [], -.5, 0.5, -.5, 0.5)
        # move_to_goal(-0.2,0,0.78,control2, plannerLeft)
        # move_to_goal(0.56,0.23,0.87,control2, plannerLeft)
        # move_to_goal(0.56,-0.4,0.87,control1, plannerRight)
        # move_to_goal(x_in, y_in, z_in, control2, plannerLeft)
    # Set your goal positions here
        # move_to_goal(-x_in, y_in, z_in - 0.93, control1, plannerRight)
    	# move_to_goal(0.47, -0.85, 0.07, control1, plannerRight)
        # move_to_goal(0.53, 0.2, 1.09, control2, plannerLeft)
        # move_to_goal(0.6, -0.1, 0.1, control1, plannerRight)

        #manually humanBot
        # move_to_goal(0.5, 0.5, 0.23, control1, plannerRight, [], 1, 0, 0, 0)
        #Set the left hand pos

if __name__ == '__main__':
    rospy.init_node('grabTransform',anonymous=True) # Initialze ROS node
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_callback)
    rospy.spin() 

