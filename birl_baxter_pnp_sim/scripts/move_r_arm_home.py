#!/usr/bin/env python

import baxter_interface
import rospy
from joint_action_client import move_to_start
import ipdb

if __name__ == '__main__':
    rospy.init_node("set_right_arm_py")
    rospy.sleep(0.2)
    joints = [ 0.03681553890924993, -0.9836651802315216,0.2515728492132078, 1.1443496677625187,
     -0.1054611791671222, 1.3698448435816746, -0.5744758050630875]
    move_to_start(joints)
    print "Done"




