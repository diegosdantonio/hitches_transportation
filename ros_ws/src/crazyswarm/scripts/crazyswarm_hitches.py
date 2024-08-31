#!/usr/bin/env python
import time

import numpy as np
from pycrazyswarm import *
from class_hitches import Quasi_satic_class


Z = 0.5

def trajectory(t, k, x0, y0):
    xd_log = np.array([k * np.cos(t) + x0, k * np.sin(t) + y0])
    return xd_log

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    TIMESCALE = 0.2

    allcfs.takeoff(targetHeight=Z, duration=2.0)
    timeHelper.sleep(1.5+Z)

    p1 = [0.0, -0.2]
    p2 = [-0.3, 0.25]
    p3 = [0.3, 0.4]
    r = 0.2

    t_step =  0.01
    t = 0.01

    swarm.input.waitUntilButtonPressed()

    while not swarm.input.checkIfButtonIsPressed():
        xp1 = trajectory(t, r, p1[0], p1[1])
        C1 = Quasi_satic_class(np.array([xp1, p2, p3]), ell=2.0)
        C1.quasi_static()

        print(t)
        posaux = C1.all
        for i, cf in enumerate(allcfs.crazyflies):
            pos = np.array(np.append(posaux[i], Z))
            cf.goTo(pos, 0, 1.0)
            print(pos)
        timeHelper.sleep(TIMESCALE)
        t += t_step

    print("press button to continue...")

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
