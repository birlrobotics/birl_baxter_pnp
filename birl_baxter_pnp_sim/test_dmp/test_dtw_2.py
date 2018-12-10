import dtw
import numpy as np
import matplotlib.pyplot as plt

x = np.array([0, 0, 1, 1, 2, 4, 2, 1, 2, 0]).reshape(-1, 1)
y = np.array([1, 1, 1, 2, 2, 2, 2, 3, 2, 0,1,5,9]).reshape(-1, 1)

plt.plot(x)
plt.plot(y)

dist, cost, acc, path = dtw.dtw(x, y, dist=lambda x, y: np.linalg.norm(x - y, ord=1))
# print path[0]
# print path[1]
x_list = []
y_list = []
for idx in path[0]:
    x_list.append(x[idx])

for idx in path[1]:
    y_list.append(y[idx])

plt.plot(x_list)
plt.plot(y_list)
plt.show()