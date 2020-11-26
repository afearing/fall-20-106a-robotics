#!/usr/bin/env python
import rospy
from baxter_interface.camera import CameraController

# Open camera (camera is a string and res is a 2-element vector)
def open_cam(camera, res):
    # Check if valid resolution
    if not any((res[0] == r[0] and res[1] == r[1]) for r in CameraController.MODES):
        rospy.logerr("Invalid resolution provided.")
    # Open camera
    cam = CameraController(camera) # Create camera object
    cam.resolution = res # Set resolution
    cam.open() # open

# Close camera
def close_cam(camera):
    cam = CameraController(camera) # Create camera object
    cam.close() # close

# Example usage, assuming left_hand_camera is open and
# right_hand_camera is closed (Check list of active cameras to know
# which cameras are open)

if __name__ == '__main__':
    rospy.init_node('camera_control', anonymous=True) # init ros node
    # Close left_hand_camera
    close_cam('left_hand_camera')
    # Open right_hand_camera and set resolution to 1280x800
    open_cam('right_hand_camera',(1280,800))
