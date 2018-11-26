from birl_dmp.dmp_training.util import generalize_via_dmp
from quaternion_interpolation import interpolate_pose_using_slerp
from trac_ik_solver import convert_pose_to_joint_plan


def get_dmp_joint_plan(start,end,dmp_model,limb):
    command_matrix = generalize_via_dmp(start, end, dmp_model)
    command_matrix = interpolate_pose_using_slerp(command_matrix)
    dmp_angle_plan = convert_pose_to_joint_plan(command_matrix,limb)
    return dmp_angle_plan