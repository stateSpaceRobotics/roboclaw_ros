#!/usr/bin/env python
import rospy
from roboclaw_ros import RoboclawManager

if __name__=="__main__":
    rospy.init_node("roboclaw_node")
    params = rospy.get_param("~")
    rc = RoboclawManager(params["managers"][0])
    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
        rc.update()
        rate.sleep()