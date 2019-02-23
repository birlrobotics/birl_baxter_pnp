import dtw
import numpy as np
import sys, os
import glob
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from scipy.ndimage.filters import gaussian_filter1d
import ipdb
from scipy.interpolate import griddata

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
demonstration_model_dir = os.path.join(dir_of_this_script, '..', 'dmp_data', 'new_demonstrations','home_to_prepick') 

def plot_demo_list(demo_list,fig_idx = 1,label="raw data",_color="grey"):
    fig = plt.figure(fig_idx)
    # ax = fig.gca(projection='3d')
    demo_list = np.array(demo_list)
    for demo in demo_list:
        # ax.plot(demo[:,0],demo[:,1],demo[:,2],color=_color)
        plt.plot(demo,color=_color)
    # ax.set_xlabel("x")
    # ax.set_ylabel("y")
    # plt.set_title(label)

def plot_demo_after_dtw(demo_list,fig_idx=4,_color="r",label="after dtw"):
    fig = plt.figure(fig_idx)
    # ax = fig.gca(projection='3d')
    # ipdb.set_trace()
    for demo_idx,demo in enumerate(demo_list):
        demo_np = np.array(demo)
        # ax.plot(demo_np[:,0],demo_np[:,1],demo_np[:,2],color=_color)
        plt.plot(demo_np[:,0],color=_color)
    # ax.set_title(label)
        
def compare_two_demos(demo1, demo2):
    fig = plt.figure(10)
    ax = fig.gca(projection='3d')
    demo1 = np.array(demo1)
    demo2 = np.array(demo2)
    # 
    ax.plot(demo1[:,0],demo1[:,1],demo1[:,2],color='g')
    ax.plot(demo2[:,0],demo2[:,1],demo2[:,2],color='r')


def convert_array_to_list(array):
    return [array[0]]

def main():
    demo_raw_list = []
    demo_filter_list = []
    demo_norm_list = []

    demo_path_list = glob.glob(os.path.join(demonstration_model_dir, '*.npy'))
    demo_path_list = sorted(demo_path_list)
    for demo_path in demo_path_list:
        demo = np.load(demo_path, 'r')
        demo_1d = np.copy(demo[:,0])
        filter_demo = gaussian_filter1d(demo_1d, sigma=1)
        demo_gird = np.linspace(0,1,len(filter_demo))
        norm_grid = np.linspace(0,1,200)
        norm_demo = griddata(demo_gird, filter_demo, norm_grid, method='linear')
        demo_raw_list.append(demo_1d)
        demo_filter_list.append(filter_demo)
        demo_norm_list.append(norm_demo)
   

    orig_demo = np.array(demo_norm_list[0]).reshape(-1, 1)
    after_dtw_demo_list = []
    after_dtw_demo_list_orig = []

    for demo in demo_norm_list:
        after_dtw_demo = []
        after_dtw_demo_orig = []
        
        demo = np.array(demo).reshape(-1, 1)
        dist, cost, acc, path = dtw.dtw(orig_demo,demo, dist=lambda x, y: np.linalg.norm(x - y, ord=1))
        ipdb.set_trace()
        for idx in path[0]:
            list_ = convert_array_to_list(orig_demo[idx])
            lisT = np.copy(list_)
            after_dtw_demo_orig.append(lisT)
        
        for idx in path[1]:
            list_ = convert_array_to_list(demo[idx])
            lisT = np.copy(list_)
            after_dtw_demo.append(lisT)
        after_dtw_demo_list.append(after_dtw_demo)
        after_dtw_demo_list_orig.append(after_dtw_demo_orig)
    
    plot_demo_list(demo_raw_list)
    plot_demo_list(demo_filter_list,fig_idx=2,label="filterd data",_color="g")
    plot_demo_list(demo_norm_list,fig_idx=3,label="normed data",_color="b")
    plot_demo_after_dtw(after_dtw_demo_list)
    # plot_demo_after_dtw(after_dtw_demo_list_orig,fig_idx=5,_color = "black")
    # demo1 = after_dtw_demo_list_orig[1]
    # demo2 = after_dtw_demo_list[1]
    # compare_two_demos(demo1, demo2)
    plt.show()


if __name__ == '__main__':
    sys.exit(main())


