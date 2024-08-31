import matplotlib; matplotlib.use("TkAgg")
import numpy as np
# import matplotlib.pyplot as plt
# from celluloid import Camera
from scipy import optimize
# from class_root import RootFinder

class Collinear_cables:
    def __init__(self, xd, ell, taux=0):
        # Initialization of collinear cables class with positions, a length ell
        self.xd1 = xd[0]
        self.xd2 = xd[1]
        self.xd3 = xd[2]

        self.ell = ell

        self.A, self.B = self.intersection_static1(self.xd1, self.xd2, self.xd3)
        self.C, self.D = self.intersection_static2(self.xd2, self.xd3, self.xd1)
        self.E, self.F = self.intersection_static3(self.xd3, self.xd1, self.xd2)

        self.all = np.array([self.A, self.B, self.C, self.D, self.E, self.F])




    def intersection_static1(self, p0, p1, p2):
        theta1 = np.arctan2(p0[1] - p1[1], p0[0] - p1[0])
        theta2 = np.arctan2(p0[1] - p2[1], p0[0] - p2[0])

        self.ellA = 0.5
        # print("ell", self.ellA)
        self.ellB = 0.5

        A = np.array([self.ellA * np.cos(theta1), self.ellA * np.sin(theta1), 0.]) + p0
        B = np.array([self.ellB * np.cos(theta2), self.ellB * np.sin(theta2), 0.]) + p0

        return [A, B]

    def intersection_static2(self, p0, p1, p2):
        theta1 = np.arctan2(p0[1] - p1[1], p0[0] - p1[0])
        theta2 = np.arctan2(p0[1] - p2[1], p0[0] - p2[0])

        self.ellC = (self.ell - self.ellB - np.linalg.norm(self.xd1 - self.xd2))
        self.ellD = (self.ell - np.linalg.norm(self.xd2 - self.xd3))/2 - self.taux

        A = np.array([self.ellC * np.cos(theta1), self.ellC * np.sin(theta1), 0.]) + p0
        B = np.array([self.ellD * np.cos(theta2), self.ellD * np.sin(theta2), 0.]) + p0
        return [A, B]

    def intersection_static3(self, p0, p1, p2):
        theta1 = np.arctan2(p0[1] - p1[1], p0[0] - p1[0])
        theta2 = np.arctan2(p0[1] - p2[1], p0[0] - p2[0])

        self.ellF = (self.ell - self.ellA - np.linalg.norm(self.xd1 - self.xd3))
        self.ellE = (self.ell - np.linalg.norm(self.xd2 - self.xd3))/2 + self.taux
        # print(self.taux)

        # self.ellC = (self.ell - self.ellB - np.linalg.norm(p0 - p1) + self.taux)

        A = np.array([self.ellE * np.cos(theta1), self.ellE * np.sin(theta1), 0.]) + p0
        B = np.array([self.ellF * np.cos(theta2), self.ellF * np.sin(theta2), 0.]) + p0

        return [A, B]


    def intersection(self, p0, p1, p2):
        theta1 = np.arctan2(p0[1] - p1[1], p0[0] - p1[0])
        theta2 = np.arctan2(p0[1] - p2[1], p0[0] - p2[0])

        ellA = (self.ell - np.linalg.norm(p0 - p1)) / 2
        ellB = (self.ell - np.linalg.norm(p0 - p2)) / 2

        A = np.array([ellA * np.cos(theta1), ellA * np.sin(theta1), 0.]) + p0
        B = np.array([ellB * np.cos(theta2), ellB * np.sin(theta2), 0.]) + p0

        return [A, B]


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
