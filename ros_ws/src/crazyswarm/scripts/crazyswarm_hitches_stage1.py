#!/usr/bin/env python
import time

import numpy as np
from numpy import ones,vstack
from numpy.linalg import lstsq
from pycrazyswarm import *
from class_hitches import Quasi_satic_class
from class_circle_trajz import circle_traj

Z = 0.8


def trajectory(t, k, x0, y0):
    xd_log = np.array([k * np.cos(t) + x0, k * np.sin(t) + y0])
    return xd_log





if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    TIMESCALE = 2

    allcfs.takeoff(targetHeight=Z, duration=2.0)
    timeHelper.sleep(1.5+Z)

    p1 = [0., -1., 0.0]
    p2 = [-0.9, 0.4, 0.0]
    p3 = [0.9, 0.4, 0.0]
    r = 0.6

    xd = np.array([p1, p2, p3])
    xd_theta = np.array([0, np.pi / 2 + np.pi / 4, - np.pi / 2 - np.pi / 4])

    r = np.array([0.25, 0.25, 0.25])

    C1 = circle_traj(xd, xd_theta, r)

    t_step =  0.08
    t = 0.01

    C1.update(0)
    posaux = C1.all

    for i, cf in enumerate(allcfs.crazyflies):
        pos = np.array(posaux[i]) + np.array([0, 0, Z])
        cf.goTo(pos, 0, 2)

    timeHelper.sleep(0.1)

    swarm.input.waitUntilButtonPressed()
    ban = False
    while t < 2*np.pi and not swarm.input.checkIfButtonIsPressed():

        C1.update(t)
        posaux = C1.all

        for i, cf in enumerate(allcfs.crazyflies):
            pos = np.array(posaux[i]) + np.array([0, 0, Z])
            cf.goTo(pos, 0, 2)

        timeHelper.sleep(0.1)
        t += t_step

    swarm.input.waitUntilButtonPressed()
    while t >= 0 and not swarm.input.checkIfButtonIsPressed():


        C1.update(t)
        posaux = C1.all

        for i, cf in enumerate(allcfs.crazyflies):
            pos = np.array(posaux[i]) + np.array([0, 0, Z])
            cf.goTo(pos, 0, 2)

        timeHelper.sleep(0.1)
        t -= t_step

    swarm.input.waitUntilButtonPressed()
    print("press button to continue...")

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
