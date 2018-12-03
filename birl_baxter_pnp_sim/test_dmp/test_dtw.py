from dtw import dtw
from numpy import array
from numpy import linalg as LA
import matplotlib.pyplot as plt
import sys, os

def main():
    x = array([0, 0, 1, 1, 2, 4, 2, 1, 2, 0]).reshape(-1, 1)
    y = array([1, 1, 1, 2, 2, 2, 2, 3, 2, 0, 1, 1, 3]).reshape(-1, 1)

    dist, cost, acc, path = dtw(x, y, dist=lambda x, y: LA.norm(x - y, ord=1))
    dtw_path = (path)   
    new_traj_1 = []
    new_traj_2 = []
    for idx in dtw_path[0]:
        new_traj_1.append(x[idx])
    for idx in dtw_path[1]:
        new_traj_2.append(y[idx])

    plt.plot(x,label="demo1",linewidth=1)
    # plt.plot(y,label="demo2",linewidth=1)
    plt.plot(new_traj_1,label="dtw_1",linewidth=2.5)
    # plt.plot(new_traj_2,label="dtw_2",linewidth=2.5)
    plt.legend()
    plt.show()






if __name__ == '__main__':
    sys.exit(main())


