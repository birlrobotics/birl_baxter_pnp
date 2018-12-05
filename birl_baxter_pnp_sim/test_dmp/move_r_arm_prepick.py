#!/usr/bin/env python

import baxter_interface
import rospy

if __name__ == '__main__':
    rospy.init_node("set_right_arm_py")
    names = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    joints = [0.23354857495555426, 1.458432234082057, 0.6991117440787773, -1.1083011192472114, -0.09817477042466648, 1.2187477359749612, 0.17180584824316633]
    combined = zip(names, joints)
    command = dict(combined)
    right = baxter_interface.Limb('right')
    right.move_to_joint_positions(command)
    print "Done"




