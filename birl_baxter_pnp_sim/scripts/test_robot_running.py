import rospy
import dill
import os,sys
import ipdb
import baxter_interface
from util import generalize_via_dmp,get_dmp_model, filter_static_points
from quaternion_interpolation import interpolate_pose_using_slerp
import numpy as np
from arm_move.srv_client import *
from arm_move import srv_action_client
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from trac_ik_solver import convert_pose_to_joint_plan

limb_name = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
demonstration_model_dir = os.path.join(dir_of_this_script, '..', 'data', 'demonstrations')

limb_interface = None

def main():
    
    rospy.init_node("test_robot_running")
    limb = 'right'
    ipdb.set_trace()
    rospy.sleep(0.5)
    traj = srv_action_client.Trajectory(limb)
    limb_interface = baxter_interface.limb.Limb(limb)

    demo = np.load(open(os.path.join(demonstration_model_dir, 'home_to_pre_pick.npy'), 'r'))
    demo = filter_static_points(demo)
    dmp_model = get_dmp_model(demo)
    start = demo[0]
    end = demo[-1]
    command_matrix = generalize_via_dmp(start, end, dmp_model)
    command_matrix = interpolate_pose_using_slerp(command_matrix)
    dmp_command_angle = convert_pose_to_joint_plan(command_matrix)

    robot_start_command = dict(zip(limb_name,dmp_command_angle[0]))
    robot_end_command = dict(zip(limb_name,dmp_command_angle[-1]))
    limb_interface.move_to_joint_positions(robot_start_command)
    # limb_interface.move_to_joint_positions(robot_end_command)

    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    traj.clear(limb)
    traj.add_point(current_angles,0.0)
    t_step = 0.15
    for idx, command in enumerate(dmp_command_angle):
        # robot_command = dict(zip(limb_name,command))
        # limb_interface.move_to_joint_positions(robot_command)
        wait_time =  t_step * idx
        traj.add_point(command,wait_time)
    traj.start()
    traj.wait(wait_time)


    # pick_object_pose = Pose()
    # pick_object_pose.position.x = 0.7
    # pick_object_pose.position.y = -0.4
    # pick_object_pose.position.z = -0.115
    # pick_object_pose.orientation.x = 0.0
    # pick_object_pose.orientation.y = 1.0
    # pick_object_pose.orientation.z = 0.0
    # pick_object_pose.orientation.w = 0.0

    # hover_pick_object_pose = copy.deepcopy(pick_object_pose)
    # hover_distance = 0.15
    # hover_pick_object_pose.position.z = hover_pick_object_pose.position.z + hover_distance
    
    # start = [0.81421265, -0.45082216,  0.0866994 ,  0.99948816, -0.02984393,0.005424  ,0.01016594]
    # end = [0.77519381, -0.09838225, -0.00453884,  0.98071834,  0.19285799,0.01934342, 0.0249633]

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