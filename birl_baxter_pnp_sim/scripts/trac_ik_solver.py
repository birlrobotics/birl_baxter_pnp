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
limb_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

def convert_pose_to_joint_plan(dmp_pose_plan,limb):
    service_name = "/trac_ik_"+limb
    # ipdb.set_trace()
    server_up = rospy.wait_for_service(service_name,timeout=5)
    
    ik_client = rospy.ServiceProxy(service_name, GetConstrainedPositionIK)
    req = GetConstrainedPositionIKRequest()
    for row in dmp_pose_plan:
        test_point = PoseStamped()
        test_point.pose.position.x = row[0]
        test_point.pose.position.y = row[1]
        test_point.pose.position.z = row[2]
        test_point.pose.orientation.x = row[3]
        test_point.pose.orientation.y = row[4]
        test_point.pose.orientation.z = row[5]
        test_point.pose.orientation.w = row[6]
        req.pose_stamp.append(test_point)             

    res = ik_client(req)  
    return res



def main():
    rospy.init_node("test_trac_ik_py")
    limb = "right"
    dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
    demonstration_dir = os.path.join(dir_of_this_script, '..', 'data', 'demonstrations')
    home_to_pre_pick = np.load(open(os.path.join(demonstration_dir, 'home_to_pre_pick.npy'), 'r'))
    # ipdb.set_trace()
    dmp_pose_plan = home_to_pre_pick
    convert_pose_to_joint_plan(dmp_pose_plan,limb)

if __name__ == '__main__':
    sys.exit(main())