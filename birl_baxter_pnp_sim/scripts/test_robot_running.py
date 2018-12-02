import rospy
import os,sys
import ipdb
from dmp_util import get_dmp_joint_plan
from joint_action_client import robot_run_trajectory
import numpy as np
from _constant import limb_name,limb,t_step,dir_of_this_script,demonstration_model_dir
from utils import list_to_pose
from gazebo_model_util import add_gazebo_models


def main():
    
    rospy.init_node("test_robot_running",anonymous=True)
    rospy.sleep(0.2) # wait for server to start up
    # ipdb.set_trace()
    demo = np.load(open(os.path.join(demonstration_model_dir, 'home_to_pre_pick.npy'), 'r'))
    start = demo[0]
    end = demo[-1]
    dmp_command_angle = get_dmp_joint_plan(start,end,demo,limb)
    robot_run_trajectory(limb,dmp_command_angle)


if __name__ == '__main__':
    sys.exit(main())