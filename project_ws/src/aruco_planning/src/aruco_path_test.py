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
import tf2_geometry_msgs


# Callback function to subscribe to images
def fiducial_callback(fiducial_tfArray):
    tfBuffer1 = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer1)

    vector3 = fiducial_tfArray.transforms[0].transform.translation
    quaternion = fiducial_tfArray.transforms[0].transform.rotation

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

    # listener.waitForTransform()
    trans, rot = 0, 0
    aruco_pose = 0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown(): 
        try:
            trans = tfBuffer1.lookup_transform("base", "head_camera", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        break
    print("transform: ", trans)
    while not rospy.is_shutdown(): 
        try:
            aruco_pose = tf2_geometry_msgs.do_transform_pose(camera_pose, trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        break
    print("pose: ", aruco_pose)
    aruco_tag_posX = aruco_pose.pose.position.x - 0.74
    aruco_tag_posY = -aruco_pose.pose.position.y
    aruco_tag_posZ = aruco_pose.pose.position.z - 0.104

    aruco_tag_rotX = aruco_pose.pose.orientation.x
    aruco_tag_rotY = aruco_pose.pose.orientation.y
    aruco_tag_rotZ = aruco_pose.pose.orientation.z
    aruco_tag_rotW = aruco_pose.pose.orientation.w

    print("translation: ", aruco_tag_posX, aruco_tag_posY, aruco_tag_posZ)
    print("rotation: ", aruco_tag_rotX, aruco_tag_rotY, aruco_tag_rotZ, aruco_tag_rotW)
    control(aruco_tag_posX, aruco_tag_posY, aruco_tag_posZ, aruco_tag_rotX, aruco_tag_rotY, aruco_tag_rotZ, aruco_tag_rotW)

def control(x_in, y_in, z_in, wx_in, wy_in, wz_in, ww_in):
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

    while not rospy.is_shutdown():
        move_to_goal(x_in, y_in, z_in, control2, plannerLeft, [], wx_in, wy_in, wz_in, ww_in)
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

