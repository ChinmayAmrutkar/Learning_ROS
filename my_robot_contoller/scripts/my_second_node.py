#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
    rospy.init_node("test_node")                    #node initialization
    rospy.loginfo("Test node has been started")     #Good practice to start with

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():      #loop runs until node is shutdown or killed
        rospy.loginfo("Hello")
        rate.sleep()                    #Keeps the loop running at above mentioned frequency (rate) in Hz for now its 0.1 seconds