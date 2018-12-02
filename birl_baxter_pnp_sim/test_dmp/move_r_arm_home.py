#!/usr/bin/env python

import baxter_interface
import rospy

if __name__ == '__main__':
    rospy.init_node("set_right_arm_py")
    names = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    joints = [-0.10776215034895031, 0.8736020587007431, 0.3670049035015852, -0.749733110078996, 0.0782330201821561, 1.4511458253396015, -0.5733253194721734]
    combined = zip(names, joints)
    command = dict(combined)
    right = baxter_interface.Limb('right')
    right.move_to_joint_positions(command)
    print "Done"




