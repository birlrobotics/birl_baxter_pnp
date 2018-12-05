from util import get_dmp_model, generalize_via_dmp
import os,sys
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
demonstration_dir = os.path.join(dir_of_this_script, '..', 'dmp_data', 'new_demonstrations',"prepick_to_pick")
fig_saving_path = os.path.join(dir_of_this_script, '..', 'dmp_data', 'processed_demonstrations',"figures","prepick_to_pick")
new_demo_saving_path = os.path.join(dir_of_this_script, '..', 'dmp_data', 'processed_demonstrations',"demos","prepick_to_pick")


def process_human_demonstration(demo, start_index, end_index):
    orig_start = demo[0]
    orig_end = demo[-1]
    proc_data_dmpModel = get_dmp_model(demo)
    data_proc_ = generalize_via_dmp(start=orig_start,end=orig_end,model=proc_data_dmpModel)
    data_proc = np.copy(data_proc_)
    new_data = data_proc[start_index:end_index,:]
    return new_data

def plot(demo,fig_idx=0, title_="human_demonstration",saving_fig_name=None ):
    fig = plt.figure(fig_idx)
    ax = fig.gca(projection='3d')
    ax.plot(demo[:,0],demo[:,1],demo[:,2])
    ax.scatter(demo[0,0],demo[0,1],demo[0,2], label="start", color="g",alpha=1)
    ax.scatter(demo[-1,0],demo[-1,1],demo[-1,2], label="end", color="r",alpha=1)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    min_lim = -0.5  
    max_lim = 0.5
    ax.set_xlim(min_lim,max_lim)
    ax.set_ylim(min_lim,max_lim)
    ax.set_zlim(min_lim,max_lim)
    ax.set_title(title_)
    ax.legend()
    if saving_fig_name!= None:
        fig.savefig( os.path.join(fig_saving_path,saving_fig_name), format='png',dpi=300,bbox_inches='tight')

def traning_dmp(demo):
    orig_start = demo[0]
    orig_end = demo[-1]
    dmpModel = get_dmp_model(demo)
    dmp_traj = generalize_via_dmp(start=orig_start,end=orig_end,model=dmpModel)
    return dmp_traj


def main():
    demo_name = "5"
    human_demonstration = np.load(open(os.path.join(demonstration_dir, demo_name)+".npy", 'r'))
    start_index = 20
    end_index = -20
    new_data = process_human_demonstration(human_demonstration,start_index, end_index)
    dmp_traj = traning_dmp(new_data)
    plot(human_demonstration,fig_idx=0, title_="human_demonstration")
    plot(new_data,fig_idx=1, title_="dmp_proc_demonstration")
    plot(dmp_traj,fig_idx=2, title_="dmp_generalization_w_orig_start_end",saving_fig_name=demo_name)
    np.save(os.path.join(new_demo_saving_path,demo_name), new_data)
    plt.show()

if __name__ == '__main__':
    sys.exit(main())