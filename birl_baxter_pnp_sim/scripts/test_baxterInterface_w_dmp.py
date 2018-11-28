import rospy
import baxter_interface
from arm_move import srv_action_client
import os, sys
import dill
from pnp_util import get_dmp_joint_plan
import ipdb

limb_interface = None
traj = None
limb = 'right'
dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
dmp_model_dir = os.path.join(dir_of_this_script, '..', 'data', 'dmp_models')
pick_pose_list = [0.7,-0.4,-0.2,0,1,0,0]
prepick_pose_list = [0.7,-0.4,0.0,0,1,0,0]

starting_joint_order_angles = [0.3144660615165098, -0.7094661143970039, 0.4663301595171658, 
        0.9234564343070191,-0.37160684586524145, 1.4066603824909245, -0.03259709174256504]
limb_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']        
start_pose = dict(zip(limb_names,starting_joint_order_angles))

def main():
    rospy.init_node("test_baxter_interface_w_dmp")   
    
    global traj,limb_interface
    limb_interface = baxter_interface.limb.Limb(limb)
    limb_interface.move_to_joint_positions(start_pose)

    traj = srv_action_client.Trajectory(limb)
    current_pose = get_current_pose_list()
    # start = get_current_pose_list()
    # end = pick_pose_list
    start = None
    end = None    
    dmp_model = dill.load(open(os.path.join(dmp_model_dir, 'pre_pick_to_pick'), 'r'))
    ipdb.set_trace()
    dmp_angle_plan = get_dmp_joint_plan(start,end,dmp_model,limb)
    
    run_traj(dmp_angle_plan)




def run_traj(dmp_angle_plan,t_step=0.3):
    global traj,limb_interface,limb
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    # ready_command = dict(zip(dmp_angle_plan.joints[0].name,dmp_angle_plan.joints[0].position))
    end_command = dict(zip(dmp_angle_plan.joints[-1].name,dmp_angle_plan.joints[0].position))
    # limb_interface.move_to_joint_positions(ready_command)
    limb_interface.move_to_joint_positions(end_command)
    rospy.loginfo("done")
    # traj.clear(limb)
    # traj.add_point(current_angles,0.0)
    # ipdb.set_trace()
    # for idx in range(len(dmp_angle_plan.isValid)):
    #     if dmp_angle_plan.isValid[idx] == False:
    #         rospy.loginfo("no valid ik")
    #         continue
    #     else: 
    #         wait_time = t_step*idx
    #         traj.add_point(dmp_angle_plan.joints[idx].position,wait_time)
    # traj.start()
    # traj.wait(wait_time)

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


if __name__ == '__main__':
    sys.exit(main())