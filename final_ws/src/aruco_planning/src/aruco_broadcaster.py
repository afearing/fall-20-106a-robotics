#!/usr/bin/env python
# import roslib
# roslib.load_manifest('learning_tf')
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialTransform

import rospy
import tf

# Callback function to subscribe to images
def fiducial_callback(fiducial_tfArray):
    vector3 = fiducial_tfArray.transforms[0].transform.translation
    quaternion = fiducial_tfArray.transforms[0].transform.rotation

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((vector3.x, vector3.y, vector3.z),
                            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                            rospy.Time.now(),
                            "aruco_tag",
                            "head_camera")
        rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('aruco_broadcaster',anonymous=True) # Initialze ROS node
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_callback)
    rospy.spin() 