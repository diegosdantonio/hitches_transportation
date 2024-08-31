from class_hitches import Collinear_cables
import numpy as np

def trajectory(t, k, x0, y0):
    xd_log = np.array([k * np.cos(t) + x0, k * np.sin(t) + y0, 0.])
    return xd_log

Z = 1

p1 = np.array([0.0, -0.2, 0.])
p2 = np.array([-0.3, 0.25, 0.])
p3 = np.array([0.3, 0.4, 0.])
r = 0.2

t = 0.

xp1 = trajectory(t, r, p1[0], p1[1])

xd = np.array([xp1, p2, p3])

C1 = Collinear_cables(xd=xd, ell=2)
posaux = C1.all



for i in range(len(posaux)):
    pos = np.array(posaux[i]) + np.array([0, 0, Z])
    print(pos)
