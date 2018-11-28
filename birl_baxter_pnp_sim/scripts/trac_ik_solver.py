import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from trac_ik_baxter.srv import GetConstrainedPositionIK,GetConstrainedPositionIKRequest
from sensor_msgs.msg import JointState
import numpy as np
import os, sys

import ipdb


def convert_pose_to_joint_plan(dmp_pose_plan,limb="right"):
    service_name = "/trac_ik_"+limb
    server_up = rospy.wait_for_service(service_name,timeout=5)
    
    ik_client = rospy.ServiceProxy(service_name, GetConstrainedPositionIK)
    req = GetConstrainedPositionIKRequest()
    jointstate_list = [] 
    for idx, row in enumerate(dmp_pose_plan):
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
        if res.isValid[0] == False:
            continue
        else:
            req = GetConstrainedPositionIKRequest()
            seed = JointState()
            seed.position = res.joints[0].position
            seed.name = res.joints[0].name
            req.seed_angles.append(seed)
            jointstate_list.append(res.joints[0].position)
            
    return jointstate_list




def main():
    rospy.init_node("test_trac_ik_py")
    limb = "right"
    dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
    demonstration_dir = os.path.join(dir_of_this_script, '..', 'data', 'demonstrations')
    home_to_pre_pick = np.load(open(os.path.join(demonstration_dir, 'home_to_pre_pick.npy'), 'r'))
    # ipdb.set_trace()
    dmp_pose_plan = home_to_pre_pick
    joint_state = convert_pose_to_joint_plan(dmp_pose_plan,limb)

    print "Done"
    

if __name__ == '__main__':
    sys.exit(main())