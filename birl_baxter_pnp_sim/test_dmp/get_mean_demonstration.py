import numpy as np
import ipdb
import os,sys
from scipy.ndimage.filters import gaussian_filter1d
from scipy.interpolate import griddata
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import glob


sigma = 3
dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
demonstration_model_dir = os.path.join(dir_of_this_script, '..', 'dmp_data', 'new_demo','move_to_prepick') # get demonstration direction

demo_path_list = glob.glob(os.path.join(demonstration_model_dir, 'home_to_pre_pick_*'))
demo_path_list = sorted(demo_path_list)

data_list = []
len_norm = 200

def plot(raw_data,filtered_data,norm_data):
    fig = plt.figure(1)
    ax = fig.gca(projection='3d')
    ax.plot(raw_data[:,0],raw_data[:,1],raw_data[:,2])
    ax.plot(filtered_data[:,0],filtered_data[:,1],filtered_data[:,2])
    ax.plot(norm_data[:,0],norm_data[:,1],norm_data[:,2])
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_ylabel("z")

def main():
    for demo_path in demo_path_list:
        raw_data = np.load(demo_path) 
        filtered_data = gaussian_filter1d(raw_data.T, sigma=sigma).T
        grid = np.linspace(0, 1, len_norm)
        time_stamp = np.linspace(0, 1, len(filtered_data))
        norm_data = griddata(time_stamp, filtered_data, grid, method='linear')
        # plot(raw_data,filtered_data,norm_data)
        # plt.show()
        data_list.append(demo)
    aa = np.array(data_list) 
    mean = aa.mean(axis=0)

if __name__ == '__main__':
    sys.exit(main())