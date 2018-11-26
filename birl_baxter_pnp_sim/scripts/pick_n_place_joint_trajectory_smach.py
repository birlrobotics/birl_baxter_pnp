#!/usr/bin/env python
"""
pick and place service smach server

prereqursite:

!!Please rosrun baxter_interface Joint_trajectory_server first!
"""

import baxter_interface
from birl_sim_examples.srv import *

import sys
import rospy
import copy

from arm_move.srv_client import *
from arm_move import srv_action_client

import smach
import smach_ros


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import ipdb
import dill
from  pnp_util import get_dmp_joint_plan
import os


dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
dmp_model_dir = os.path.join(dir_of_this_script, '..', 'data', 'dmp_models')


class Go_to_Start_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        
    def execute(self, userdata):
        rospy.loginfo('executing Go to Start position...')
        global limb
        global traj
        global limb_interface
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]

        starting_joint_order_angles = [0.3144660615165098, -0.7094661143970039, 0.4663301595171658, 
        0.9234564343070191,-0.37160684586524145, 1.4066603824909245, -0.03259709174256504]
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_point(starting_joint_order_angles, 5.0)
        traj.start()
        traj.wait(10.0)
        traj.gripper_open()
        return 'Succeed'
        
class Setting_Start_and_End_Pose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'],
                             output_keys=['pick_object_pose', 'place_object_pose'])
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        rospy.loginfo('executing Setting_Start_and_End_Pose... ')
        self.pick_object_pose = Pose()
        self.place_object_pose = Pose()
        
        self.pick_object_pose.position.x = 0.7
        self.pick_object_pose.position.y = -0.4
        self.pick_object_pose.position.z = -0.115
        #RPY = 0 pi 0
        self.pick_object_pose.orientation = Quaternion(
            x=0.0,
            y=1.0,
            z=0.0,
            w=6.123233995736766e-17)
      
        self.place_object_pose.position.x = 0.7
        self.place_object_pose.position.y = -0.1
        self.place_object_pose.position.z = -0.115
        self.place_object_pose.orientation = Quaternion(
            x=0.0,
            y=1.0,
            z=0.0,
            w=6.123233995736766e-17)

        userdata.pick_object_pose = copy.deepcopy(self.pick_object_pose)
        userdata.place_object_pose = copy.deepcopy(self.place_object_pose)
        
        return 'Succeed'

class Add_Box_Gazebo_Model(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'],
                             input_keys=['pick_object_pose'])
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        srv_action_client.delete_gazebo_models()
        srv_action_client.load_gazebo_models(model_name="box_female",
                           model_pose=Pose(position=Point(x=userdata.pick_object_pose.position.x,
                                                          y=userdata.pick_object_pose.position.y,
                                                          z=userdata.pick_object_pose.position.z),
                                           orientation=Quaternion(x=0,y=0,z=0,w=1)),
                           model_reference_frame="base")
        return 'Succeed'

class Go_to_PrePick_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['pick_object_pose','hover_distance'])
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface

        pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
        hover_pick_object_pose = copy.deepcopy(pick_object_pose)
        hover_pick_object_pose.position.z = hover_pick_object_pose.position.z + userdata.hover_distance
        start = get_current_pose_list()
        end = get_end_pose_list(hover_pick_object_pose)
        dmp_model = dill.load(open(os.path.join(dmp_model_dir, 'home_to_pre_pick'), 'r'))
        dmp_angle_plan = get_dmp_joint_plan(start,end,dmp_model,limb)
        run_traj(dmp_angle_plan)
        rospy.sleep(1)
        return 'Succeed'

class Go_to_Pick_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['pick_object_pose','hover_distance'])
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
        start = get_current_pose_list()
        end = get_end_pose_list(pick_object_pose)
        dmp_model = dill.load(open(os.path.join(dmp_model_dir, 'pick_to_pre_pick'), 'r'))
        dmp_angle_plan = get_dmp_joint_plan(start,end,dmp_model,limb)
        run_traj(dmp_angle_plan)
        rospy.sleep(1)
        traj.gripper_close()
        rospy.sleep(1)
        return 'Succeed'

class MoveToPrePickPoseWithClosedHand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['pick_object_pose','hover_distance'])
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
        hover_pick_object_pose = copy.deepcopy(pick_object_pose)
        hover_pick_object_pose.position.z = hover_pick_object_pose.position.z + userdata.hover_distance

        start = get_current_pose_list()
        end = get_end_pose_list(hover_pick_object_pose)

        dmp_model = dill.load(open(os.path.join(dmp_model_dir, 'pick_to_pre_pick'), 'r'))
        dmp_angle_plan = get_dmp_joint_plan(start,end,dmp_model,limb)
        run_traj(dmp_angle_plan)
        return 'Succeed'

class Go_to_PrePlace_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['place_object_pose','hover_distance'])
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        place_object_pose = copy.deepcopy(userdata.place_object_pose)
        hover_place_object_pose = copy.deepcopy(place_object_pose)
        hover_place_object_pose.position.z = hover_place_object_pose.position.z + userdata.hover_distance
        start = get_current_pose_list()
        end = get_end_pose_list(hover_place_object_pose)
        dmp_model = dill.load(open(os.path.join(dmp_model_dir, 'pre_pick_to_pre_place'), 'r'))
        dmp_angle_plan = get_dmp_joint_plan(start,end,dmp_model,limb)
        run_traj(dmp_angle_plan)
        return 'Succeed'


class Go_to_Place_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['place_object_pose','hover_distance'])
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        place_object_pose = copy.deepcopy(userdata.place_object_pose)
        start = get_current_pose_list()
        end = get_end_pose_list(place_object_pose)
        dmp_model = dill.load(open(os.path.join(dmp_model_dir, 'pre_place_to_place'), 'r'))
        dmp_angle_plan = get_dmp_joint_plan(start,end,dmp_model,limb)
        run_traj(dmp_angle_plan)
        rospy.sleep(1)
        traj.gripper_open()
        return 'Succeed'

class MoveToPrePlacePoseWithOpenHand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['place_object_pose','hover_distance'])
        
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        place_object_pose = copy.deepcopy(userdata.place_object_pose)
        hover_place_object_pose = copy.deepcopy(place_object_pose)
        hover_place_object_pose.position.z = hover_place_object_pose.position.z + userdata.hover_distance
        start = get_current_pose_list()
        end = get_end_pose_list(hover_place_object_pose)
        dmp_model = dill.load(open(os.path.join(dmp_model_dir, 'place_to_pre_place'), 'r'))
        dmp_angle_plan = get_dmp_joint_plan(start,end,dmp_model,limb)
        run_traj(dmp_angle_plan)
        return 'Succeed'


def get_current_pose_list():
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

def get_end_pose_list(pose):
    pose_list = [ pose.position.x, 
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w]  
    return pose_list

def run_traj(dmp_angle_plan,t_step=0.3):
    global traj,limb_interface
    end_command = dict(zip(dmp_angle_plan.joints[-1].name,dmp_angle_plan.joints[0].position))
    # limb_interface.move_to_joint_positions(end_command)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    traj.clear(limb)
    traj.add_point(current_angles,0.0)
    for idx in range(len(dmp_angle_plan.isValid)):
        if dmp_angle_plan.isValid[idx] == False:
            rospy.loginfo("no valid ik")
            continue
        else: 
            
            wait_time = t_step*idx
            traj.add_point(dmp_angle_plan.joints[idx].position,wait_time)
    traj.start()
    traj.wait(wait_time)

def shutdown():
    global limb
    global traj
    global lintimb_erface
    rospy.loginfo("Stopping the node...")
    srv_action_client.delete_gazebo_models()
    traj.clear('right')
    traj.stop()

        
def main():
    rospy.init_node("pick_n_place_joint_trajectory")
    rospy.on_shutdown(shutdown)
    rospy.wait_for_message("/robot/sim/started", Empty)
    #ipdb.set_trace()
 
    sm = smach.StateMachine(outcomes=['Done'])

    sm.userdata.sm_pick_object_pose = Pose()
    sm.userdata.sm_place_object_pose = Pose()
    sm.userdata.sm_hover_distance = 0.15

    global traj
    global limb_interface
    global limb
    
    limb = 'right'
    traj = srv_action_client.Trajectory(limb)
    limb_interface = baxter_interface.limb.Limb(limb)
    
    with sm:
        smach.StateMachine.add('Go_to_Start_Position',Go_to_Start_Position(),
                               transitions={'Succeed':'Setting_Start_and_End_Pose'})
        smach.StateMachine.add('Setting_Start_and_End_Pose',Setting_Start_and_End_Pose(),
                               transitions={'Succeed':'Add_Box_Gazebo_Model'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                               'place_object_pose':'sm_place_object_pose'})
        smach.StateMachine.add('Add_Box_Gazebo_Model',Add_Box_Gazebo_Model(),
                               transitions={'Succeed':'Go_to_PrePick_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose'})

        smach.StateMachine.add('Go_to_PrePick_Position',Go_to_PrePick_Position(),
                               transitions={'Succeed':'Go_to_Pick_Position',
                                            'IK_Fail':'Go_to_Start_Position',
                                            'Time_Out':'Go_to_Start_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})      

        smach.StateMachine.add('Go_to_Pick_Position',Go_to_Pick_Position(),
                               transitions={'Succeed':'MoveToPrePickPoseWithClosedHand',
                                            'IK_Fail':'Go_to_Start_Position',
                                            'Time_Out':'Go_to_Start_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})

        smach.StateMachine.add('MoveToPrePickPoseWithClosedHand',MoveToPrePickPoseWithClosedHand(),
                               transitions={'Succeed':'Go_to_PrePlace_Position',
                                            'IK_Fail':'Go_to_Start_Position',
                                            'Time_Out':'Go_to_Start_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})

        smach.StateMachine.add('Go_to_PrePlace_Position',Go_to_PrePlace_Position(),
                               transitions={'Succeed':'Go_to_Place_Position',
                                            'IK_Fail':'Go_to_Start_Position',
                                            'Time_Out':'Go_to_Start_Position'},
                               remapping={'place_object_pose':'sm_place_object_pose',
                                          'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})

        smach.StateMachine.add('Go_to_Place_Position',Go_to_Place_Position(),
                               transitions={'Succeed':'MoveToPrePlacePoseWithOpenHand',
                                            'IK_Fail':'Go_to_Start_Position',
                                            'Time_Out':'Go_to_Start_Position'},
                               remapping={'place_object_pose':'sm_place_object_pose',
                                          'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})
        smach.StateMachine.add('MoveToPrePlacePoseWithOpenHand',MoveToPrePlacePoseWithOpenHand(),
                               transitions={'Succeed':'Done',
                                            'IK_Fail':'Go_to_Start_Position',
                                            'Time_Out':'Go_to_Start_Position'},
                               remapping={'place_object_pose':'sm_place_object_pose',
                                          'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})                                            
    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')

    sis.start()
    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())


