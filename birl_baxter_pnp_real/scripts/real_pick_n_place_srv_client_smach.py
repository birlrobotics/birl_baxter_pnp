#!/usr/bin/env python
"""
pick and place service smach server
"""

from arm_move import pick_and_place
from birl_sim_examples.srv import *

import sys
import rospy
import copy

from arm_move.srv_client import *

import smach
import smach_ros

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import ipdb

import random

class Go_to_Start_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'])
        self.state = 0
        
    def execute(self, userdata):
        rospy.loginfo('executing Go to Start position...')
        hmm_state_switch_client(self.state)
        go_to_start_position_client()
        return 'Succeed'
        
class Setting_Start_and_End_Pose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed'],
                             output_keys=['pick_object_pose', 'place_object_pose'])
    def execute(self, userdata):
        rospy.loginfo('executing Setting_Start_and_End_Pose... ')
        self.pick_object_pose = Pose()
        self.place_object_pose = Pose()
        
        self.pick_object_pose.position.x = 0.78370181675
        self.pick_object_pose.position.y = -0.296405272911
        self.pick_object_pose.position.z = -0.03448412355




        #RPY = 0 pi 0
        self.pick_object_pose.orientation = Quaternion(
            x= -0.0600070131966,
            y= 0.997894836048,
            z=  0.0199740264787,
            w=  0.0143559333636)

      
        self.place_object_pose.position.x = self.pick_object_pose.position.x
        self.place_object_pose.position.y = self.pick_object_pose.position.y + 0.4
        self.place_object_pose.position.z = self.pick_object_pose.position.z
        self.place_object_pose.orientation = self.pick_object_pose.orientation

        userdata.pick_object_pose = copy.deepcopy(self.pick_object_pose)
        userdata.place_object_pose = copy.deepcopy(self.place_object_pose)
        
        return 'Succeed'

# class Add_Box_Gazebo_Model(smach.State):
#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['Succeed'],
#                              input_keys=['pick_object_pose'])
        
#     def execute(self, userdata):
#         pick_and_place.delete_gazebo_models()
#         add_gazebo_model_client(_model_name="box_female",
#                                 _model_pose=Pose(position=Point(x=userdata.pick_object_pose.position.x, y=userdata.pick_object_pose.position.y, z=-0.115),
#                                                  orientation=Quaternion(x=0,y=0,z=0,w=1)),
#                                 _model_reference_frame="base")
#         return 'Succeed'

class Go_to_Pick_Hover_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['pick_object_pose','hover_distance'])
        self.state = 1
        
    def execute(self, userdata):
        self.pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
        self.pick_object_pose.position.z = self.pick_object_pose.position.z + userdata.hover_distance
        hmm_state_switch_client(self.state)
        (ik_flag, action_flag) = go_to_position_client(pose = self.pick_object_pose)
        if not ik_flag:
            return 'IK_Fail'
        elif not action_flag:
            return 'Time_Out'
        else:
            return 'Succeed'


class Pick_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail', 'Time_Out'],
                             input_keys=['pick_object_pose','hover_distance'])
        self.state = 2
        
    def execute(self, userdata):
        gripper_move_client(is_move_close = False)
        self.pick_object_pose = copy.deepcopy(userdata.pick_object_pose)
        (ik_flag, action_flag) = go_to_position_client(pose = self.pick_object_pose)
        gripper_move_client(is_move_close = True)
        self.pick_object_pose.position.z =self.pick_object_pose.position.z+ userdata.hover_distance
        hmm_state_switch_client(self.state)
        (ik_flag, action_flag) = go_to_position_client(pose = self.pick_object_pose)
        if not ik_flag:
            return 'IK_Fail'
        elif not action_flag:
            return 'Time_Out'
        else:
            return 'Succeed'


class Go_to_Place_Hover_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['place_object_pose','hover_distance'])
        self.state = 3
        
    def execute(self, userdata):
        self.place_object_pose = copy.deepcopy(userdata.place_object_pose)
        self.place_object_pose.position.z = self.place_object_pose.position.z + userdata.hover_distance
        hmm_state_switch_client(self.state)
        (ik_flag, action_flag) = go_to_position_client(pose = self.place_object_pose)
        if not ik_flag:
            return 'IK_Fail'
        elif not action_flag:
            return 'Time_Out'
        else:
            return 'Succeed'


class Place_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Succeed','IK_Fail','Time_Out'],
                             input_keys=['place_object_pose','hover_distance'])
        self.state = 4
        
    def execute(self, userdata):
        self.place_object_pose = copy.deepcopy(userdata.place_object_pose)
        (ik_flag, action_flag) = go_to_position_client(pose = self.place_object_pose)
        gripper_move_client(is_move_close = False)
        self.place_object_pose.position.z =self.place_object_pose.position.z+ userdata.hover_distance
        hmm_state_switch_client(self.state)
        (ik_flag, action_flag) = go_to_position_client(pose = self.place_object_pose)        
        if not ik_flag:
            return 'IK_Fail'
        elif not action_flag:
            return 'Time_Out'
        else:
            return 'Succeed' 
    
def main():
    """RSDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("pick_n_place_client")
    #rospy.on_shutdown(shutdown)
    #rospy.wait_for_message("/robot/sim/started", Empty)
    #ipdb.set_trace()
 
    sm = smach.StateMachine(outcomes=['Done'])

    sm.userdata.sm_pick_object_pose = Pose()
    sm.userdata.sm_place_object_pose = Pose()
    sm.userdata.sm_hover_distance = 0.15
    with sm:
        smach.StateMachine.add('Go_to_Start_Position',Go_to_Start_Position(),
                               transitions={'Succeed':'Setting_Start_and_End_Pose'})
        smach.StateMachine.add('Setting_Start_and_End_Pose',Setting_Start_and_End_Pose(),
                               transitions={'Succeed':'Go_to_Pick_Hover_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                               'place_object_pose':'sm_place_object_pose'})                      
        smach.StateMachine.add('Go_to_Pick_Hover_Position',Go_to_Pick_Hover_Position(),
                               transitions={'Succeed':'Pick_Object',
                                            'IK_Fail':'Go_to_Start_Position',
                                            'Time_Out':'Go_to_Start_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})

        smach.StateMachine.add('Pick_Object',Pick_Object(),
                               transitions={'Succeed':'Go_to_Place_Hover_Position',
                                            'IK_Fail':'Go_to_Start_Position',
                                            'Time_Out':'Go_to_Start_Position'},
                               remapping={'pick_object_pose':'sm_pick_object_pose',
                                          'hover_distance':'sm_hover_distance'})

        smach.StateMachine.add('Go_to_Place_Hover_Position',Go_to_Place_Hover_Position(),
                               transitions={'Succeed':'Place_Object',
                                            'IK_Fail':'Go_to_Start_Position',
                                            'Time_Out':'Go_to_Start_Position'},
                               remapping={'place_object_pose':'sm_place_object_pose',
                                          'hover_distance':'sm_hover_distance'})

        smach.StateMachine.add('Place_Object',Place_Object(),
                               transitions={'Succeed':'Done',
                                            'IK_Fail':'Go_to_Start_Position',
                                            'Time_Out':'Go_to_Start_Position'},
                               remapping={'place_object_pose':'sm_place_object_pose',
                                          'hover_distance':'sm_hover_distance'})
  
    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')

    sis.start()
    outcome = sm.execute()

    hmm_state_switch_client(0)

    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())


