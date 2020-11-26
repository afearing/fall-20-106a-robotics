#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    pub = rospy.Publisher('ik_new_pos', JointState, queue_size=10)
    sleepyTime = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_hand"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #YOUR CODE HERE
        #Write code to get the (x, y, z) coordinates of the end effector from the user
        #Hint: the function raw_input() might be useful
        inp = raw_input("Please enter the three positions x, y and z")
        posns = inp.split(" ")
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = float(posns[0])
        request.ik_request.pose_stamped.pose.position.y = float(posns[1])
        request.ik_request.pose_stamped.pose.position.z = float(posns[2])
        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            pub.publish(response.solution.joint_state)
            
            #Print the response HERE
            print(response)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main()

