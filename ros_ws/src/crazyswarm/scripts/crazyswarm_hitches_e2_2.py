#!/usr/bin/env python
import time

import numpy as np
from numpy import ones,vstack
from numpy.linalg import lstsq
from pycrazyswarm import *
from class_hitches import Collinear_cables
import argparse

Z = 0.6

def trajectory(t, k, x0, y0):
    xd_log = np.array([k * np.cos(t) + x0, k * np.sin(t) + y0, 0.])
    return xd_log

def linear_traj(p1, p2, t):
    points = [p1, p2]
    x_coords, y_coords = zip(*points)
    A = vstack([x_coords, ones(len(x_coords))]).T
    m, c = lstsq(A, y_coords)[0]

    x = p1[0] + t + 0.2*np.sin(t) #+ 0.05 * np.sign(t)
    # print(x)
    y = m * x + c
    # print(y)
    return x, y


if __name__ == "__main__":
    # Crazyswarm's inner parser must add help to get all params.
    parser = argparse.ArgumentParser(add_help=False)
    #
    group = parser.add_argument_group("Collision avoidance", "")
    group.add_argument(
        "--noavoid",
        help="Disable collision avoidance.",
        action="store_true"
    )
    group.add_argument(
        "--assign",
        help=("Use optimal start-goal assignment instead of random assignment."),
        action="store_true"
    )
    group.add_argument(
        "--loops",
        type=int,
        default=1,
        help="Repeat the experiment this many times, without resetting start positions.",
    )
    args, unknown = parser.parse_known_args()


    swarm = Crazyswarm(parent_parser=parser)
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    TIMESCALE = 2

    allcfs.takeoff(targetHeight=Z, duration=2.0)
    timeHelper.sleep(1.5+Z)



    swarm.input.waitUntilButtonPressed()

    xy_radius = 0.1
    radii = 1.0 * xy_radius * np.array([1.0, 1.0, 1.0])
    cfs = swarm.allcfs.crazyflies


    if not args.noavoid:
        for i, cf in enumerate(cfs):
            others = cfs[:i] + cfs[(i+1):]
            cf.enableCollisionAvoidance(others, radii)


#########################################################################################
#########################################################################################
    p1 = np.array([0., -.5, 0.0])
    p2 = np.array([-0.4, 0.5, 0.0])
    p3 = np.array([0.4, 0.5, 0.0])

    p1aux = np.array([0., -.5, 0.0])
    p2aux = np.array([-0.4, 0.5, 0.0])
    p3aux = np.array([0.4, 0.5, 0.0])

    r = 0.2

    t_step =  0.01
    t = 0.01

    ban = True
    cont = 0

    ell = 1.9

    taux = 0.

    while not swarm.input.checkIfButtonIsPressed():

        if ban == False and swarm.input.checkIfAnyButtonIsPressed():
            ban = True
            print(ban)
            # timeHelper.sleep(1)

        elif ban == True and swarm.input.checkIfAnyButtonIsPressed():
            ban = False
            # print(ban)
            # timeHelper.sleep(1)

        if ban == True and p2[1] <= 0.6:
            taux -= 0.002
            # print(taux)
            Dp2 = linear_traj(p2aux[0:2], p1aux[0:2], taux)
            Dp3 = linear_traj(p3aux[0:2], p1aux[0:2], -taux)
            p2[0:2] = Dp2
            p3[0:2] = Dp3
        else:
            ban = False

        if ban == False and p2[1] >= -0.45:
            taux += 0.002
            # print(taux)
            Dp2 = linear_traj(p2aux[0:2], p1aux[0:2], taux)
            Dp3 = linear_traj(p3aux[0:2], p1aux[0:2], -taux)
            p2[0:2] = Dp2
            p3[0:2] = Dp3
        else:
            ban = True

        # taux += 0.05
        # Dp2 = linear_traj(p2aux[0:2], p1aux[0:2], taux)
        # Dp3 = linear_traj(p3aux[0:2], p1aux[0:2], -taux)
        # p2[0:2] = Dp2
        # p3[0:2] = Dp3


        xd = np.array([p1, p2, p3])

        C1 = Collinear_cables(xd=xd, ell=ell)
        posaux = C1.all

        for i, cf in enumerate(allcfs.crazyflies):
            pos = np.array(posaux[i]) + np.array([0, 0, Z])
            # print(pos)
            cf.goTo(pos, 0, 1)

        timeHelper.sleep(0.2)
        t += t_step
        cont += 1
        if cont >= 2:
            cont = 0
            # print(p1)
            # print(p2)
            # print(p3)
            # print(taux)

    # timeHelper.sleep(1.5 + Z)
    swarm.input.waitUntilButtonPressed()

    print("press button to continue...")

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
