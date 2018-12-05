#!/usr/bin/env python
"""
service client module. create a class 
which contains a variety of service client fucntions
"""

import sys
import rospy
import rospkg
import copy
import struct
import numpy as np
import actionlib

from birl_sim_examples.srv import *

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
import operator
import baxter_interface
import ipdb
from _constant import robot_runing_speed,limb_name

class Trajectory(object):
    def __init__(self, limb,verbose = False):
        # set up the action client
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        
        ## check up if the joint_trajectory action server setup
        ## please rosrun baxter_interface joint_trajectory_server
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)

        #clear all  JointTrajectoryPoint
        self.clear(limb)
        self._verbose = verbose
        # enable the IK Service
        ik_ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_ns, SolvePositionIK)
        rospy.wait_for_service(ik_ns, 5.0)
        
        self._gripper = baxter_interface.Gripper(limb)
        self._limb = baxter_interface.limb.Limb(limb)
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        if self._init_state != True:
            print("Enabling robot... ")
            self._rs.enable()

    def find_start_offset(self,start_angle, cur_angle,speed):
        dflt_vel = [speed] * 7
        diffs = map(operator.sub, start_angle, cur_angle)
        diffs = map(operator.abs, diffs)
        #determine the largest time offset necessary across all joints
        t_offset = max(map(operator.div, diffs, dflt_vel))
        return t_offset

    def find_offset(self, dmp_angle_plans,speed):
        dflt_vel = [speed]*7
        mat = np.array(dmp_angle_plans)
        last = mat[0]
        new_mat = [last]
        t_offset = []
        for idx in range(mat.shape[0]):
            diffs = map(operator.sub, mat[idx], last)
            diffs = map(operator.abs, diffs)
            t_offset_ = max(map(operator.div, diffs, dflt_vel))
            last = mat[idx]
            t_offset.append(t_offset_)
        return np.array(t_offset).cumsum()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy.copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        return self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def add_pose_point(self,pose,time):
        angles = self.ik_request(pose)
        if not angles:
            return 0
        else:
            self.add_point(angles,time)
            return 1

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = limb_name

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)
  
    def ik_request(self,pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

        limb_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
        limb_angles = [limb_joints[joint] for joint in limb_names]
        return limb_angles

def hmm_state_switch_client(state):
    rospy.wait_for_service('hmm_state_switch')
    try:
        hmm_state_switch_proxy = rospy.ServiceProxy('hmm_state_switch',
                                                    State_Switch)
        req = State_SwitchRequest()
        req.state = state
        resp = hmm_state_switch_proxy(req)
        if resp.finish.data:
            print "Hmm State switch to %d succesfully" %state
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def get_current_angle(limb="right"):
    traj = Trajectory(limb)
    limb_interface = traj._limb 
    cur_angle = [limb_interface.joint_angle(joint) for joint in limb_name]
    return cur_angle


def get_current_pose_list(limb="right"):
    traj = Trajectory(limb)
    limb_interface = traj._limb 
    current_pose_dic = limb_interface.endpoint_pose()
    current_pose_list = [ current_pose_dic['position'].x, 
                    current_pose_dic['position'].y,
                    current_pose_dic['position'].z,
                    current_pose_dic['orientation'].x,
                    current_pose_dic['orientation'].y,
                    current_pose_dic['orientation'].z,
                    current_pose_dic['orientation'].w]  
    return current_pose_list

def move_to_start(start_angle,limb="right"):
    traj = Trajectory(limb)
    cur_angle = get_current_angle()
    start_wait_time = traj.find_start_offset(start_angle,cur_angle,speed=robot_runing_speed)
    traj.clear(limb)
    traj.add_point(cur_angle, 0.0)
    traj.add_point(start_angle,start_wait_time) 
    traj.start()
    rospy.loginfo("MOve to dmp trajectory start\n")
    traj.wait(start_wait_time)

def robot_run_trajectory(limb,dmp_command_angle,gripper_state="open"): 
    traj = Trajectory(limb)
    cur_angle = get_current_angle()
    move_to_start(dmp_command_angle[0])
    traj_wait_time = traj.find_offset(dmp_command_angle,speed=robot_runing_speed)
    for idx, command in enumerate(dmp_command_angle):
        wait_time =  traj_wait_time[idx]
        traj.add_point(command,wait_time)
    traj.start()
    traj.wait(wait_time)
    rospy.loginfo("Finish dmp trajectory\n")
    if gripper_state == "open":
        traj.gripper_open()
    elif gripper_state == "close":
        rospy.sleep(1)
        traj.gripper_close()


