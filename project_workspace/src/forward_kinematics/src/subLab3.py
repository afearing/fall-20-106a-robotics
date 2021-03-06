#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the dependencies as described in example_pub.py
import rospy
import lab3_skeleton
import numpy as np
import sys
#from my_chatter.msg import TimestampString
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def callback(message):

    #Print the contents of the message to the console
    print("current action:")
    print(message.header.stamp.secs)
    n = message.name
    left_thetas_indices = np.array([n.index('left_s0'), n.index('left_s1'),n.index('left_e0'), n.index('left_e1'), n.index('left_w0'), n.index('left_w1'), n.index('left_w2')])
    thetas = np.asarray(message.position)[left_thetas_indices]
    # thetas = np.array([message.position[4], message.position[5], message.position[2], message.position[3], message.position[6], message.position[7], message.position[8]])
    # print(thetas.shape)
    print(lab3_skeleton.lab3(thetas))
    # print(message.header.stamp.secs)
    # print(message)

#Define the method which contains the node's main functionality
def listener(topic):

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber(topic, JointState, callback)

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()


#Python's syntax for a main() method
if __name__ == '__main__':

    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('listener', anonymous=True)
    try:
        listener(sys.argv[-1])
    except rospy.ROSInterruptException:
        pass