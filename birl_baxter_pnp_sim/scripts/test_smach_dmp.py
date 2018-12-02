#!/usr/bin/env python
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
import smach
import smach_ros


limb_name = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'] 
limb = 'right' 
t_step = 0.15
dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
demonstration_model_dir = os.path.join(dir_of_this_script, '..', 'data', 'demonstrations') # get demonstration direction





traj = None
limb_interface = None

class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        demo = np.load(open(os.path.join(demonstration_model_dir, 'home_to_pre_pick.npy'), 'r'))
        demo = filter_static_points(demo)
        dmp_model = get_dmp_model(demo)
        start = demo[0]
        end = demo[-1]
        command_matrix = generalize_via_dmp(start, end, dmp_model)
        command_matrix = interpolate_pose_using_slerp(command_matrix)
        dmp_command_angle = convert_pose_to_joint_plan(command_matrix)

        robot_start_command = dict(zip(limb_name,dmp_command_angle[0]))
        limb_interface.move_to_joint_positions(robot_start_command)

        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        traj.clear(limb)
        traj.add_point(current_angles,0.0)

        for idx, command in enumerate(dmp_command_angle):
            wait_time =  t_step * idx
            traj.add_point(command,wait_time)
        traj.start()
        traj.wait(wait_time)
        rospy.loginfo("finshed")
        return "outcome1"

def main():
    global traj, limb_interface
    rospy.init_node('smach_example_state_machine')
    rospy.sleep(0.5) # wait for server to start up
    traj = srv_action_client.Trajectory(limb)
    limb_interface = baxter_interface.limb.Limb(limb)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Done'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(Foo.__name__, Foo(), 
                               transitions={'outcome1':'Done'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    sys.exit(main())
