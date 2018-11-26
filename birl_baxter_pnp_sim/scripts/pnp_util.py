from birl_dmp.dmp_training.util import generalize_via_dmp
from quaternion_interpolation import interpolate_pose_using_slerp
from trac_ik_solver import convert_pose_to_joint_plan,add_points_to_traj


excute_dmp(start,end,dmp_model,traj,limb_interface):
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    command_matrix = generalize_via_dmp(start, end, dmp_model)
    command_matrix = interpolate_pose_using_slerp(command_matrix)
    dmp_angle_plan = convert_pose_to_joint_plan(dmp_pose_plan,limb)
    traj.clear('right')
    traj.add(current_angles,0.0)
    traj, wait_time = add_points_to_traj(dmp_angle_plan,traj)
    traj.start()
    print "robot start running"
    traj.wait(wait_time)