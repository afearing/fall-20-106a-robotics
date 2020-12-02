#!/usr/bin/env python
import cv2 # OpenCV library
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
import cv2.aruco as aruco

bridge = cv_bridge.CvBridge()
#pub = rospy.Publisher('video_frames', Image, queue_size=10)
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

# Callback function to subscribe to images
def image_callback(ros_img):
 # Convert received image message to OpenCv image
    cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
    cv2.imshow('Image', cv_image) # display image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    font = cv2.FONT_HERSHEY_SIMPLEX
    # pub.publish(cv_image)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('CameraToOpenCV',anonymous=True) # Initialze ROS node
    rospy.Subscriber('/cameras/head_camera/image', Image, image_callback)
    rospy.spin() 
    cv2.destroyAllWindows() # Destroy CV image window on shut_down

# def talker():
# 	rate = rospy.Rate(10)
#     pub.publish()
# if __name__ == '__main__':
# 	rospy.init_node('OpenCV_HeadCam', anonymous=True)
# 	try:
# 		talker()
# 	except rospy.ROSInterruptException: pass

# def publish_message():
 
#   # Node is publishing to the video_frames topic using 
#   # the message type Image
#   pub = rospy.Publisher('video_frames', Image, queue_size=10)
     
#   # Tells rospy the name of the node.
#   # Anonymous = True makes sure the node has a unique name. Random
#   # numbers are added to the end of the name.
#   rospy.init_node('video_pub_py', anonymous=True)
     
#   # Go through the loop 10 times per second
#   rate = rospy.Rate(10) # 10hz
     
#   # Create a VideoCapture object
#   # The argument '0' gets the default webcam.
#   cap = cv2.VideoCapture(0)
     
#   # While ROS is still running.
#   while not rospy.is_shutdown():
     
#       # Capture frame-by-frame
#       # This method returns True/False as well
#       # as the video frame.
#       ret, frame = cap.read()
         
#       if ret == True:
#         # Print debugging information to the terminal
#         rospy.loginfo('publishing video frame')
             
#         # Publish the image.
#         # The 'cv2_to_imgmsg' method converts an OpenCV
#         # image to a ROS image message
#         pub.publish(br.cv2_to_imgmsg(frame))
             
#       # Sleep just enough to maintain the desired rate
#       rate.sleep()