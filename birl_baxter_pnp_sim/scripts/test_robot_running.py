import rospy
import os,sys
import ipdb
from dmp_util import get_dmp_joint_plan
from joint_action_client import robot_run_trajectory, move_to_start,get_current_pose_list
import numpy as np
from _constant import limb,demonstration_model_dir, place_pose
from utils import list_to_pose,pose_to_list,handle_object_in_gazebo_offset
from gazebo_model_util import add_gazebo_models, get_model_pose
import smach
import smach_ros

class MoveToReadyPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        ready_pose = [ 0.03681553890924993, -0.9836651802315216,0.2515728492132078, 1.1443496677625187, -0.1054611791671222, 1.3698448435816746, -0.5744758050630875]
        move_to_start(ready_pose)
        return "succuss"

class MoveToHoverPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        demo = np.load(open(os.path.join(demonstration_model_dir,'home_to_prepick', '2.npy'), 'r'))
        model_pose = get_model_pose("box_male",hover_flag=True)
        start = get_current_pose_list()
        end = model_pose
        # end = demo[-1]
        rospy.loginfo("dmp start pose is %s\n" %start[0:3])
        rospy.loginfo("dmp end pose is %s\n" %end[0:3] )
        dmp_command_angle = get_dmp_joint_plan(start,end,demo,limb)
        rospy.loginfo(dmp_command_angle[0])
        robot_run_trajectory(limb,dmp_command_angle,gripper_state="open")
        return "succuss"

class MoveToPickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        demo = np.load(open(os.path.join(demonstration_model_dir,'prepick_to_pick', '2.npy'), 'r'))
        model_pose = get_model_pose("box_male")
        start = get_current_pose_list()
        end = model_pose
        # end = model_pose[-1] 
        rospy.loginfo("dmp start pose is %s\n" %start[0:3])
        rospy.loginfo("dmp end pose is %s\n" %end[0:3] )
        dmp_command_angle = get_dmp_joint_plan(start,end,demo,limb)
        robot_run_trajectory(limb,dmp_command_angle,gripper_state="close",point_mode=True)
        return "succuss"
        
class BackToPickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        demo = np.load(open(os.path.join(demonstration_model_dir,'prepick_to_pick', '2.npy'), 'r'))
        model_pose = get_model_pose("box_male",hover_flag=True)
        start = get_current_pose_list()
        end = model_pose
        # end = model_pose[-1] 
        rospy.loginfo("dmp start pose is %s\n" %start[0:3])
        rospy.loginfo("dmp end pose is %s\n" %end[0:3] )
        dmp_command_angle = get_dmp_joint_plan(start,end,demo,limb)
        robot_run_trajectory(limb,dmp_command_angle,gripper_state="close",point_mode=True)
        return "succuss"  

class MoveToPrePlacePosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        demo = np.load(open(os.path.join(demonstration_model_dir,'pre_place_to_place.npy'), 'r'))
        # model_pose = get_model_pose("box_male",hover_flag=True)
        start = get_current_pose_list()
        # start = demo[0]
        # end = demo[-1]
        end = place_pose
        rospy.loginfo("dmp start pose is %s\n" %start[0:3])
        rospy.loginfo("dmp end pose is %s\n" %end[0:3] )
        dmp_command_angle = get_dmp_joint_plan(start,end,demo,limb)
        robot_run_trajectory(limb,dmp_command_angle,gripper_state="close",point_mode=True)
        return "succuss"


def main():
    rospy.init_node("test_robot_running",anonymous=True)
    rospy.sleep(0.2) # wait for server to start up

    sm = smach.StateMachine(outcomes=['Done'])
    with sm:
        smach.StateMachine.add(MoveToReadyPose.__name__, MoveToReadyPose(), 
                               transitions={'succuss':MoveToHoverPosition.__name__})     

        smach.StateMachine.add(MoveToHoverPosition.__name__, MoveToHoverPosition(), 
                               transitions={'succuss':MoveToPickPosition.__name__})    

        smach.StateMachine.add(MoveToPickPosition.__name__, MoveToPickPosition(), 
                               transitions={'succuss':BackToPickPosition.__name__}),      

        smach.StateMachine.add(BackToPickPosition.__name__, BackToPickPosition(), 
                               transitions={'succuss':MoveToPrePlacePosition.__name__})    
                                                                                   
        smach.StateMachine.add(MoveToPrePlacePosition.__name__, MoveToPrePlacePosition(), 
                               transitions={'succuss':'Done'})
                                   
    outcome = sm.execute()


if __name__ == '__main__':
    sys.exit(main())