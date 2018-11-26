import rospy
import dill
import os,sys
import ipdb
import baxter_interface
from birl_dmp.dmp_training.util import generalize_via_dmp
from quaternion_interpolation import interpolate_pose_using_slerp
import numpy
from arm_move.srv_client import *
from arm_move import srv_action_client
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from trac_ik_solver import convert_pose_to_joint_plan,add_points_to_traj
dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
dmp_model_dir = os.path.join(dir_of_this_script, '..', 'data', 'dmp_models')

limb_interface = None

def main():
    
    rospy.init_node("test_robot_running")
    # rospy.wait_for_message("/robot/sim/started", Empty)
    limb = 'right'
    ipdb.set_trace()
    traj = srv_action_client.Trajectory(limb)
    global limb_interface
    limb_interface = baxter_interface.limb.Limb(limb)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]


    pick_object_pose = Pose()
    pick_object_pose.position.x = 0.7
    pick_object_pose.position.y = -0.4
    pick_object_pose.position.z = -0.115
    pick_object_pose.orientation.x = 0.0
    pick_object_pose.orientation.y = 1.0
    pick_object_pose.orientation.z = 0.0
    pick_object_pose.orientation.w = 0.0

    hover_pick_object_pose = copy.deepcopy(pick_object_pose)
    hover_distance = 0.15
    hover_pick_object_pose.position.z = hover_pick_object_pose.position.z + hover_distance
    
    start = [0.81421265, -0.45082216,  0.0866994 ,  0.99948816, -0.02984393,0.005424  ,0.01016594]
    end = [0.77519381, -0.09838225, -0.00453884,  0.98071834,  0.19285799,0.01934342, 0.0249633]
    dmp_model = dill.load(open(os.path.join(dmp_model_dir, 'home_to_pre_pick'), 'r'))
    command_matrix = generalize_via_dmp(start, end, dmp_model)
    command_matrix = interpolate_pose_using_slerp(command_matrix)
    # dmp_pose_plan =  covert_matix_to_pose(command_matrix)
    # ipdb.set_trace()
    dmp_angle_plan = convert_pose_to_joint_plan(command_matrix,limb)
    traj.clear('right')
    traj = add_points_to_traj(dmp_angle_plan,traj)
    traj.start()
    traj.wait(100.0)



def get_current_pose():
    global limb_interface
    current_pose_dic = limb_interface.endpoint_pose()
    current_pose_list = [ current_pose_dic['position'].x, 
                    current_pose_dic['position'].y,
                    current_pose_dic['position'].z,
                    current_pose_dic['orientation'].x,
                    current_pose_dic['orientation'].y,
                    current_pose_dic['orientation'].z,
                    current_pose_dic['orientation'].w]  
    return current_pose_list

def covert_matix_to_pose(command_matrix):
    list_of_poses = []
    for row in command_matrix:
        _pose = PoseStamped() 
        _pose.pose.position.x = row[0]
        _pose.pose.position.y = row[1]
        _pose.pose.position.z = row[2]
        _pose.pose.orientation.x = row[3]
        _pose.pose.orientation.y = row[4]
        _pose.pose.orientation.z = row[5]
        _pose.pose.orientation.w = row[6]
        list_of_poses.append(_pose)
    return list_of_poses

if __name__ == '__main__':
    sys.exit(main())