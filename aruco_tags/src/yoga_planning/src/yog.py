import sys
import copy
import rospy
importmoveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import std_msgs.msg



def fiducicial_callback(fiducial_tfArray):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('yog', anonymous=true)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSCeneInterface()



if __name__ == '__main__':
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_callback)



class YogaPlanner(object):
    def __init__(self, group_name):
        """
        Constructor.

        Inputs:
        group_name: the name of the move_group.
            For Baxter, this would be 'left_arm' or 'right_arm'
            For Sawyer, this would be 'right_arm'
        """

        # If the node is shutdown, call this function    
        rospy.on_shutdown(self.shutdown)

        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the robot
        self._robot = moveit_commander.RobotCommander()

        # Initialize the planning scene
        self._scene = moveit_commander.PlanningSceneInterface()

        # This publishes updates to the planning scene
        self._planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

        # Instantiate a move group
        self._group = moveit_commander.MoveGroupCommander(group_name)

        # Set the maximum time MoveIt will try to plan before giving up
        self._group.set_planning_time(5)

        # Set the bounds of the workspace
        self._group.set_workspace([-2, -2, -2, 2, 2, 2])

        # Sleep for a bit to ensure that all inititialization has finished
        rospy.sleep(0.5)

    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety

        Currently deletes the object's MoveGroup, so that further commands will do nothing
        """
        self._group = None
        rospy.loginfo("Stopping Path Planner")