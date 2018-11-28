from pydmps.dmp_discrete import DMPs_discrete
import matplotlib.pyplot as plt
import numpy as np

dmp = DMPs_discrete(dt=.01, n_dmps=1, n_bfs=10, w=np.zeros((1, 10)))
y_track, dy_track, ddy_track = dmp.rollout()
plt.figure(1, figsize=(6, 3))
plt.plot(np.ones(len(y_track))*dmp.goal, 'r--', lw=2)
plt.plot(y_track, lw=2)
plt.title('DMP system - no forcing term')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend(['goal', 'system state'], loc='lower right')
plt.tight_layout()

# --------------------------------------------------------------
# No force term DMP
