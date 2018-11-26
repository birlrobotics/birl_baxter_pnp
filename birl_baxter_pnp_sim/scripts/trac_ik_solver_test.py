import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from trac_ik_baxter.srv import GetConstrainedPositionIK,GetConstrainedPositionIKRequest
import numpy as np
import os, sys
import ipdb
from trac_ik_solver import convert_pose_to_joint_plan
limb_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

def main():
    rospy.init_node("test_trac_ik_py")
    limb = "right"
    dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
    demonstration_dir = os.path.join(dir_of_this_script, '..', 'data', 'demonstrations')
    home_to_pre_pick = np.load(open(os.path.join(demonstration_dir, 'home_to_pre_pick.npy'), 'r'))
    dmp_pose_plan = home_to_pre_pick
    convert_pose_to_joint_plan(dmp_pose_plan,limb)

if __name__ == '__main__':
    sys.exit(main())