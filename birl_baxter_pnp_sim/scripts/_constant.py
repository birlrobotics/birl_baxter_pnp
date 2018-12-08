import os

limb_name = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'] 
limb = 'right' 
dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
demonstration_model_dir = os.path.join(dir_of_this_script, '..', 'dmp_data', 'processed_demonstrations','demos') # get demonstration direction
gazebo_model_dir = os.path.join(dir_of_this_script, '..', 'model')
robot_runing_speed = 0.3 # 0.65 max speed
hover_height = 0.15
model_pose = [[0.6,-0.25,-0.115, 1, 0, 0, 0]]
pick_pose = [0.6,-0.25,-0.115, 1, 0, 0, 0]
place_pose = [-0.07, -0.8, 0.5, 1, 0, 0, 0]
model_name = ["box_male"]