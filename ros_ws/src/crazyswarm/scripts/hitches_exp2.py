import time

import matplotlib; matplotlib.use("TkAgg")
import numpy as np
import matplotlib.pyplot as plt
from celluloid import Camera
from scipy import optimize
# from class_root import RootFinder


class Quasi_satic_class:
    def __init__(self, xd, ell):

        self.xd1 = xd[0]
        self.xd2 = xd[1]
        self.xd3 = xd[2]

        self.ell = ell


        xd1_xd2 = self.xd2 - self.xd1
        xd3_xd1 = self.xd3 - self.xd1
        xd3_xd2 = self.xd3 - self.xd2
        # print(xd1_xd2)

        # distance between fixed points and desired intersection of cables
        self.l_xd1_xd2 = np.linalg.norm(xd1_xd2)
        self.l_xd3_xd1 = np.linalg.norm(xd3_xd1)
        self.l_xd3_xd2 = np.linalg.norm(xd3_xd2)

        # computation indeed triagle shape
        self.theta1 = np.pi - np.arctan2(xd1_xd2[1], xd1_xd2[0])
        self.theta2 = np.arctan2(xd3_xd1[1], xd3_xd1[0])
        self.theta5 = np.arctan2(xd3_xd2[1], xd3_xd2[0])

        # print(np.rad2deg(self.theta1), np.rad2deg(self.theta2), np.rad2deg(self.theta5))

        self.p1 = np.array([0, 0])
        self.p2 = np.array([0, 0])
        self.p3 = np.array([0, 0])
        self.p4 = np.array([0, 0])
        self.p5 = np.array([0, 0])
        self.p6 = np.array([0, 0])


    def f1(self, alpha, T1, T2, thetaC1, thetaC2):
        c1 = -np.cos(thetaC1) + np.cos(thetaC2)
        c2 = np.sin(thetaC1) + np.sin(thetaC2)

        return (c1**2 + c2**2
                + 2 * c1 * T1 * np.cos(alpha)
                - 2 * c2 * T1 * np.sin(alpha))

    def f2(self, alpha, T1, T2, thetaC1, thetaC2):
        c1 = np.cos(thetaC1) + np.cos(thetaC2)
        c2 = np.sin(thetaC1) - np.sin(thetaC2)

        return (c1**2 + c2**2
                - 2 * c1 * T1 * np.cos(alpha)
                + 2 * c2 * T1 * np.sin(alpha))

    def f3(self, alpha, T1, T2, thetaC1, thetaC2):
        c1 = - np.cos(thetaC1) - np.cos(thetaC2)
        c2 = - np.sin(thetaC1) - np.sin(thetaC2)

        return (c1**2 + c2**2
                + 2 * c1 * T1 * np.cos(alpha)
                + 2 * c2 * T1 * np.sin(alpha))

    def quasi_static(self):

        # calculate upper lines

        # calculate upper angles
        theta9 = optimize.bisect(self.f1, 0, np.pi/2, args=(1, 1, self.theta1, self.theta2))
        # print(np.rad2deg(theta9))
        theta4 = optimize.bisect(self.f2, 0, np.pi/2, args=(1, 1, self.theta5, self.theta1))
        # print(np.rad2deg(theta4))
        theta6 = optimize.bisect(self.f3, 0, np.pi/4, args=(1, 1, self.theta5, self.theta2))
        # print(np.rad2deg(theta6))

        theta3 = np.arccos(-np.cos(theta4) - np.cos(self.theta5) + np.cos(self.theta1))
        theta7 = np.arccos(-np.cos(theta6) + np.cos(self.theta5) + np.cos(self.theta2))
        theta8 = np.arccos(np.cos(theta9) - np.cos(self.theta1) + np.cos(self.theta2))

        # theta2 = np.arccos(np.cos(theta1) - (np.cos(alpha) - np.cos(beta)))

        l_p1_p2 = (self.ell - self.l_xd1_xd2) / 2
        l_p3_p1 = (self.ell - self.l_xd3_xd1) / 2
        l_p2_p3 = (self.ell - self.l_xd3_xd2) / 2

        # calculate points
        self.A = np.array([l_p3_p1 * np.cos(theta9), -l_p3_p1 * np.sin(theta9)]) + self.xd1
        self.B = np.array([-l_p1_p2 * np.cos(theta8), -l_p1_p2 * np.sin(theta8)]) + self.xd1

        self.C = np.array([l_p1_p2 * np.cos(theta3), -l_p1_p2 * np.sin(theta3)]) + self.xd2
        self.D = np.array([-l_p2_p3 * np.cos(theta4), l_p2_p3 * np.sin(theta4)]) + self.xd2

        self.E = np.array([l_p2_p3 * np.cos(theta7), l_p2_p3 * np.sin(theta7)]) + self.xd3
        self.F = np.array([l_p3_p1 * np.cos(theta6), l_p3_p1 * np.sin(theta6)]) + self.xd3

        self.all = np.array([self.A, self.B, self.C, self.D, self.E, self.F])




    def ploting(self,axes, camera):
        # plotting
        axes.plot([self.xd1[0], self.xd2[0]], [self.xd1[1], self.xd2[1]], 'r')
        axes.plot([self.xd2[0], self.xd3[0]], [self.xd2[1], self.xd3[1]], 'b')
        axes.plot([self.xd3[0], self.xd1[0]], [self.xd3[1], self.xd1[1]], 'k')

        axes.plot([self.A[0], self.xd1[0]], [self.A[1], self.xd1[1]], 'k')
        axes.plot([self.B[0], self.xd1[0]], [self.B[1], self.xd1[1]], 'r')
        axes.plot([self.C[0], self.xd2[0]], [self.C[1], self.xd2[1]], 'r')
        axes.plot([self.D[0], self.xd2[0]], [self.D[1], self.xd2[1]], 'b')
        axes.plot([self.E[0], self.xd3[0]], [self.E[1], self.xd3[1]], 'b')
        axes.plot([self.F[0], self.xd3[0]], [self.F[1], self.xd3[1]], 'k')

        axes.set_xlabel('x(m)')
        axes.set_ylabel('y(m)')

        # axes[0].plot(xd_log[0, :], xd_log[1, :], '--r')

        camera.snap()
        axes.axis('equal')

def trajectory(t, k, x0, y0):
    xd_log = np.array([k * np.cos(t) + x0, k * np.sin(t) + y0])
    return xd_log

fig, axes = plt.subplots()
camera = Camera(fig)

# time
Time = np.linspace(0,2*np.pi,100)

p1_1 = [ 0.0, -0.2]
p2_1 = [-0.3,  0.25]
p3_1 = [ 0.3,  0.4]

C1 = Quasi_satic_class(np.array([p1_1, p2_1, p3_1]), ell=2.0)
C1.quasi_static()
C1.ploting(axes,camera)


time.sleep(10)
k = 0.01
p1_2 = [ 0.0 * k, -0.2 * k]
p2_2 = [-0.3 * k,  0.25* k]
p3_2 = [ 0.3 * k,  0.4 * k]

C1 = Quasi_satic_class(np.array([p1_2, p2_2, p3_2]), ell=2.0)
C1.quasi_static()
C1.ploting(axes,camera)
time.sleep(10)

# for t, i in zip(Time, range(len(Time))):
#     xp1 = trajectory(t, r, p1[0], p1[1])
#     xp2 = trajectory(t, 1, p2[0], p2[1])
#     xp3 = trajectory(t, 1, p3[0], p3[1])
#
#     C1 = Quasi_satic_class(np.array([xp1, p2, p3]), ell=2.0)
#     C1.quasi_static()
#     C1.ploting(axes, camera)

animation = camera.animate()

# animation.save('hitch.gif')
plt.show()