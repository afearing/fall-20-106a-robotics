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
from controller import Controller
import tf2_ros


# Callback function to subscribe to images
def fiducial_callback(fiducial_tfArray):
    vector3 = fiducial_tfArray.transforms[0].transform.translation
    print("lengthOfFiducial: ", len(fiducial_tfArray.transforms))
    print("x,y,z: ", vector3.x, " ", vector3.y, " ", vector3.z)
    control(vector3.x, vector3.y, vector3.z)

def control(x_in, y_in, z_in):
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

    control1 = Controller(Kp, Ki, Kd, Kw, Limb("right"))
    control2 = Controller(Kp, Ki, Kd, Kw, Limb("left"))

    # tfBuffer = tf2_ros.Buffer()
	# tfListener = tf2_ros.TransformListener(tfBuffer)
	# rate = rospy.Rate(1.0)
	# # while not rospy.is_shutdown():
    # try:
    #     trans = tfBuffer.lookup_transform(targetFrame, referenceFrame, rospy.Time())
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     rate.sleep()
    #     continue

    def move_to_goal(x, y, z, controller, planner, orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
        print("x,y,z in: ", x_in, " ", y_in, " ", z_in)
        while not rospy.is_shutdown():
            try:
                goal = PoseStamped()
                goal.header.frame_id = "/"
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

    while not rospy.is_shutdown():
        move_to_goal(x_in, y_in, z_in, control2, plannerLeft)
    # Set your goal positions here
        # move_to_goal(0.3,0.5,0,control2, plannerLeft)
    	# move_to_goal(0.47, -0.85, 0.07, control1, plannerRight)
        # move_to_goal(0.6, -0.3, 0.0, control1, plannerRight)
        # move_to_goal(0.6, -0.1, 0.1, control1, plannerRight)

        #Set the left hand pos
    # control1.shutdown()
    control2.shutdown()

if __name__ == '__main__':
    rospy.init_node('grabTransform',anonymous=True) # Initialze ROS node
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_callback)
    rospy.spin() 

