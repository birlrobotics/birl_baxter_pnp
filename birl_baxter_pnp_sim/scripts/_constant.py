import os

limb_name = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'] 
limb = 'right' 
dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
demonstration_model_dir = os.path.join(dir_of_this_script, '..', 'dmp_data', 'demonstrations') # get demonstration direction
gazebo_model_dir = os.path.join(dir_of_this_script, '..', 'model')
robot_runing_speed = 0.1 # 0.65 max speed
pick_list = [0.6,-0.35,2,0,1,0,0]
models = ["box"]