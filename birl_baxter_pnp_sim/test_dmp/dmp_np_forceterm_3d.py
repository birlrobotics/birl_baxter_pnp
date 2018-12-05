from pydmps.dmp_discrete import DMPs_discrete
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

dmp = DMPs_discrete(dt=.01, n_dmps=7, n_bfs=20, w=np.zeros((7, 20)))
y_track, dy_track, ddy_track = dmp.rollout()
fig = plt.figure(1)
ax = fig.gca(projection='3d')
ax.plot(y_track[:,0],y_track[:,1],y_track[:,2])

plt.figure(2, figsize=(6, 3))
# plt.plot(np.ones(len(y_track))*dmp.goal, 'r--', lw=2)
plt.plot(y_track[:,1], lw=2)
plt.title('DMP system - no forcing term')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend(['goal', 'system state'], loc='lower right')
plt.tight_layout()
plt.show()
# --------------------------------------------------------------
# No force term DMP
