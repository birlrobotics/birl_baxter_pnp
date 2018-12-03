import rospy
import os,sys
import ipdb
from dmp_util import get_dmp_joint_plan
from joint_action_client import robot_run_trajectory
import numpy as np
from _constant import limb,demonstration_model_dir
from utils import list_to_pose
from gazebo_model_util import add_gazebo_models
import smach
import smach_ros

class MoveToHoverPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        demo = np.load(open(os.path.join(demonstration_model_dir, 'home_to_pre_pick.npy'), 'r'))
        start = demo[0]
        end = demo[-1]
        dmp_command_angle = get_dmp_joint_plan(start,end,demo,limb)
        robot_run_trajectory(limb,dmp_command_angle)
        return "succuss"

class MoveToPickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        demo = np.load(open(os.path.join(demonstration_model_dir, 'pre_pick_to_pick.npy'), 'r'))
        start = demo[0]
        end = demo[-1]
        dmp_command_angle = get_dmp_joint_plan(start,end,demo,limb)
        robot_run_trajectory(limb,dmp_command_angle)
        return "succuss"

def main():
    rospy.init_node("test_robot_running",anonymous=True)
    rospy.sleep(0.2) # wait for server to start up

    sm = smach.StateMachine(outcomes=['Done'])
    with sm:
        smach.StateMachine.add(MoveToHoverPosition.__name__, MoveToHoverPosition(), 
                               transitions={'succuss':'MoveToPickPosition'})
        
        smach.StateMachine.add(MoveToPickPosition.__name__, MoveToPickPosition(), 
                               transitions={'succuss':'Done'})
    ipdb.set_trace()                           
    outcome = sm.execute()


if __name__ == '__main__':
    sys.exit(main())