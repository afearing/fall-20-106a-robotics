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

# translation_mat = np.matrix([[-0.0346,   -0.0868,    0.7219],
#                             [-0.9911,   -1.3159,    0.0159],
#                             [-0.0223,    1.4142,   0.4642]])
translation_mat = np.matrix([[ 0.9006,   -2.2108,    1.1064],
   [-1.0068,   -0.0008,   -0.0017],
    [7.5019,  -12.1340,    3.5229]
])
# Callback function to subscribe to images
def fiducial_callback(fiducial_tfArray):
    tfBuffer1 = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer1)

    br = tf2_ros.TransformBroadcaster()
    rate2 = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform

    vector3 = fiducial_tfArray.transforms[0].transform.translation
    quaternion = fiducial_tfArray.transforms[0].transform.rotation
    frame = fiducial_tfArray.header
    print(frame)

    camera_pose = PoseStamped()
    camera_pose.header.frame_id = "head_camera"
    #x, y, and z position
    camera_pose.pose.position.x = vector3.x
    camera_pose.pose.position.y = vector3.y
    camera_pose.pose.position.z = vector3.z
    # camera_coords = np.matrix([vector3.x, vector3.y, vector3.z])
    # print(camera_coords)
    # result_coords = np.matmul(translation_mat, camera_coords.T)
    # print(result_coords)

    #Orientation as a quaternion
    camera_pose.pose.orientation.x = quaternion.x
    camera_pose.pose.orientation.y = quaternion.y
    camera_pose.pose.orientation.z = quaternion.z
    camera_pose.pose.orientation.w = quaternion.w

    # # listener.waitForTransform()
    trans1, trans2, rot = 0, 0, 0
    aruco_pose = 0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown(): 
        try:
            trans1 = tfBuffer1.lookup_transform("world", "head_camera", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        break
    # while not rospy.is_shutdown(): 
    #     try:
    #         trans2 = tfBuffer1.lookup_transform("head_camera", "base", rospy.Time())
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         rate.sleep()
    #         continue
    #     break
#     testMat = np.array([[   -1.0032,    2.3667,    0.8309],
#    [-0.9545,   -0.1790,   -0.0016],
#    [-8.1999,   25.0806,    1.2777]])

    print(trans1)
    # print(trans2)
    # trans1Rot = np.array([[ -0.0002172,  0.1789026,  0.9838668, -0.0001813],
    #                     [0.9999999,  0.0002172,  0.0001813, 0],
    #                     [-0.0001813,  0.9838668, -0.1789026, 0.75 ],
    #                     [0, 0, 0, 1]])
    # coordinateCamera = np.array([0.11124, 0.197028, 0.6341])
    # coordCam2 = np.array([vector3.x, vector3.y, vector3.z])
    # print(np.matmul(trans1Rot, coordinateCamera.T))
    # print("test", np.matmul(testMat, coordCam2.T))
    # translation1 = np.array([0.1883, 0, 0.75, 1])
    # trans2Rot = np.array([[ -0.0003894,  0.9999999,  0.0000250, -.0000183],
    #                     [0.1786996,  0.0000450,  0.9839036, 0],
    #                     [0.9839036,  0.0003876, -0.1786996, 0.75],
    #                     [0, 0, 0, 1]])

    
    # while not rospy.is_shutdown(): 
    #     try:
    #         trans2 = tfBuffer1.lookup_transform("base", "head_pan", rospy.Time())
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         rate.sleep()
    #         continue
    #     break
    # trans1_mat = tf2_ros.transformations
    # print("transform: ", trans)
    while not rospy.is_shutdown(): 
        try:
            aruco_pose = tf2_geometry_msgs.do_transform_pose(camera_pose, trans1)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        break
    
    print("pose: ", aruco_pose)
    aruco_tag_posX = aruco_pose.pose.position.x 
    aruco_tag_posY = aruco_pose.pose.position.y
    aruco_tag_posZ = aruco_pose.pose.position.z 

    # aruco_tag_rotX = aruco_pose.pose.orientation.x
    # aruco_tag_rotY = aruco_pose.pose.orientation.y
    # aruco_tag_rotZ = aruco_pose.pose.orientation.z
    # aruco_tag_rotW = aruco_pose.pose.orientation.w

    # print("translation: ", aruco_tag_posX, aruco_tag_posY, aruco_tag_posZ)
    # print("rotation: ", aruco_tag_rotX, aruco_tag_rotY, aruco_tag_rotZ, aruco_tag_rotW)
    control(aruco_tag_posX, aruco_tag_posY, aruco_tag_posZ, 0, -1, 0, 0, frame)
    # control(vector3.x, vector3.y, vector3.z, 0, -1, 0, 0, frame)
    # control(vector3.x + 1.2,vector3.y + 0.34,vector3.z - 1,0,0,0,0,frame)

def control(x_in, y_in, z_in, wx_in, wy_in, wz_in, ww_in, header_id):
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

    # tfBuffer = tf2_ros.Buffer()
	# tfListener = tf2_ros.TransformListener(tfBuffer)
	# rate = rospy.Rate(1.0)
	# # while not rospy.is_shutdown():
    # try:
    #     trans = tfBuffer.lookup_transform(targetFrame, referenceFrame, rospy.Time())
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     rate.sleep()
    #     continue

    control1 = Controller(Kp, Ki, Kd, Kw, Limb("right"))
    control2 = Controller(Kp, Ki, Kd, Kw, Limb("left"))

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
    print("coords in: ", x_in, y_in, z_in)
    while not rospy.is_shutdown():
        # input1 = float(raw_input("Please enter 3 points: "))
        # input2 = float(raw_input("Please enter 3 points: "))
        # input3 = float(raw_input("Please enter 3 points: "))
        # move_to_goal(input1, input2, input3, control2, plannerLeft)
        # move_to_goal(-0.62,0.5,0.24,control2, plannerLeft)
        move_to_goal(0.3,0.5,0,control2, plannerLeft)
        # move_to_goal(-0.2,0,0.78,control2, plannerLeft)
        # move_to_goal(0.56,0.23,0.87,control2, plannerLeft)
        # move_to_goal(0.56,-0.4,0.87,control1, plannerRight)
        # move_to_goal(x_in, y_in, z_in, control2, plannerLeft)
    # Set your goal positions here
        # move_to_goal(-x_in, y_in, z_in - 0.93, control1, plannerRight)
    	# move_to_goal(0.47, -0.85, 0.07, control1, plannerRight)
        # move_to_goal(0.53, 0.2, 1.09, control2, plannerLeft)
        # move_to_goal(0.6, -0.1, 0.1, control1, plannerRight)

        #Set the left hand pos
    control1.shutdown()
    control2.shutdown()

if __name__ == '__main__':
    rospy.init_node('grabTransform',anonymous=True) # Initialze ROS node
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_callback)
    rospy.spin() 

