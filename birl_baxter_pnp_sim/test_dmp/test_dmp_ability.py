from util import get_dmp_model, generalize_via_dmp
import os
import numpy as np


dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
demonstration_dir = os.path.join(dir_of_this_script, '..', 'dmp_data', 'new_demo')
demo = np.load(open(os.path.join(demonstration_dir, 'move_to_hover_position.npy'), 'r'))

orig_start = demo[0]
orig_end = demo[-1]
dmp_model_1 = get_dmp_model(demo)
y_track_1 = generalize_via_dmp(start=orig_start,end=orig_end,model=dmp_model_1)


from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
fig = plt.figure(1)
ax = fig.gca(projection='3d')
ax.plot(demo[:,0],demo[:,1],demo[:,2])
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_title("orig_start_end")

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
fig = plt.figure(2)
ax = fig.gca(projection='3d')
ax.plot(y_track_1[:,0],y_track_1[:,1],y_track_1[:,2])
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_title("dmp_orig_start_end")


plt.show()




