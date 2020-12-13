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
    camera_pose = PoseStamped()
    camera_pose.header.frame_id = "head_camera"
    #x, y, and z position
    camera_pose.pose.position.x = vector3.x
    camera_pose.pose.position.y = vector3.y
    camera_pose.pose.position.z = vector3.z

    #Orientation as a quaternion
    camera_pose.pose.orientation.x = quaternion.x
    camera_pose.pose.orientation.y = quaternion.y
    camera_pose.pose.orientation.z = quaternion.z
    camera_pose.pose.orientation.w = quaternion.w

    trans1 = 0
    aruco_pose = 0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown(): 
        try:
            trans1 = tfBuffer1.lookup_transform("base", "head_camera", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        break
    while not rospy.is_shutdown(): 
        try:
            aruco_pose = tf2_geometry_msgs.do_transform_pose(camera_pose, trans1)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        break
    print("pose: ", aruco_pose)

    # control(vector3.x, vector3.y, vector3.z, -.5, 0.5, -.5, 0.5)
    control(0,0,0,0,0,0,0)

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
        # move_to_goal(0.3,0.5, 0, 0, 0, 0, 0, plannerLeft)
        # move_to_goal(0.18,0.41, -0.71, -.5, 0.5, -.5, 0.5, plannerLeft)
        # move_to_goal(0.1816,0.41372, -0.71166, -.5, 0.5, -.5, 0.5, plannerLeft)
        move_to_goal(0.794176,-0.073172, -0.016992, -.5, 0.5, -.5, 0.5, plannerRight)
        move_to_goal(1.014176,-0.073172, -0.016992, -.5, 0.5, -.5, 0.5, plannerRight)
        move_to_goal(1.014176,-0.53, 0.2, -.5, 0.5, -.5, 0.5, plannerRight)
        # move_to_goal(x_in, y_in, z_in, wx_in, wy_in, wz_in, ww_in)
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

