# Description

  This repo is for pick and place experiment of HMM analysing the state_classification and fault recovery in Birl research group.

  birl_baxter_pnp_sim is pick and place in Gazebo simulation 
  birl_baxter_pnp_real is pick and place in real experiment.
  
  
There are three folder in scripts folder:
baxter_PNP_endpose              unimodal data for endpoint pose 
baxter_PNP_endpose_N_wrench     multimodal data for both endpoint pose and wrench
baxter_PNP_wrench               unimodal data for wrench
  
  
# Running Pick and Place Code


     roslaunch birl_baxter_description pick_n_place_box_gazebo.launch 
     
     
using the joint trajactory method

     roslaunch birl_baxter_pnp_sim pick_n_place_joint_trajactory.launch
using the move to angle method

     roslaunch birl_baxter_pnp_sim pick_n_place_move_angle.launch 
     
     

# HMM Online Analysis
## Recoding success trails


In order to train your HMM model, you are required to collect several success trals data when executing pick and place.

You should create some folder like this

/XXX/success/01
/XXX/success/02
/XXX/success/03
...

and record the data using rosbag:

    rosbag record -O /XXX/success/01/0X.bag /tag_multimodal

Then we should convert .bag file to .csv file. In my case, I use the rosbag_to_csv repo:

Download: https://github.com/AtsushiSakai/rosbag_to_csv

After catkin_make you can run the converting code like this:

     rosrun rosbag_to_csv rosbag_to_csv.py
     
## Trainning HMM models

Then we can start to train HMM model.

Due to depending on Pandas and HMMlearn module, we recommend you to use anaconda environment. In my case, I can type the command before running code in every terminal which needs this environment:

    export PATH="/home/ben/anaconda2/bin:$PATH"
    
Then we trainning the models:

    rosrun birl_baxter_pnp_sim hmm_model_train_real_baxter_PNP_multisequence_endpose.py
    
After that, you are able to train the threshold for fault detection:

    rosrun birl_baxter_pnp_sim hmm_model_train_threshold_real_baxter_PNP_multisequence_endpose.py 
    
## Online HMM Anaylyse


