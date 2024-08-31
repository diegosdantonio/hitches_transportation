import numpy as np
from scipy import optimize

# function catenary
def f(c,l,x_bar):
    return (- l/2 + c * np.sinh(x_bar/(2*c)))  # only one real root at x = 1

# Rotation matrix
def Rotz(th):
    Rot = np.array([[np.cos(th), -np.sin(th), 0],
                  [np.sin(th), np.cos(th), 0],
                  [0, 0, 1]])
    return Rot

# Hat_map
def hat_map(aux):
    A = np.array([[0, -aux[2], aux[1]], [aux[2], 0, -aux[0]], [-aux[1], aux[0], 0]])
    return A


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
        l3_8 = (self.ell - self.l_xd1_xd2) / 2
        l9_6 = (self.ell - self.l_xd3_xd1) / 2
        l4_7 = (self.ell - self.l_xd3_xd2) / 2

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


# function for compute the position of the robots
# here the minimum point is fixed and we are rotating
# and increasing and decreasing the span.




# def trajectoryA(t):
#     t=2*t
#     # minimum point fixed

#     # Gustavo
#     dst_xCd = np.array([1., 0., 0.3], dtype='f')
#     # dst_xCd = np.array([0.5 + 0.9*np.cos(t*0.7), 0., 0.0], dtype='f')
#     dst_dxCd = np.array([0., 0., 0.], dtype='f')
#     dst_ddxCd = np.array([0., 0., 0.], dtype='f')

#     # rotating with the time
#     dst_yaw = t/10  #np.pi/2 # 

#     dst_Omega = np.array([0, 0, 0], dtype='f')

#     dst_R = Rotz(dst_yaw)
#     dst_dR = dst_R.dot(hat_map(dst_Omega))

#     ## span desired
#     # Diego
#     dst_x_bar_d = 0.15*np.sin(t/1) + 0.5

#     ## Gustavo, 
#     # ex1: 
#     # dst_x_bar_d = 0.5 
#     # ex2: 
#     # dst_x_bar_d = 0.6

#     dst_dx_bar_d = 0

#     # solution using bisection
#     sol = optimize.bisect(f, 0.01, 10, args=(1., dst_x_bar_d))  #(f, x0=[0.15], args=(1.5, dst_x_bar_d), method='hybr', tol=1e-6)
#     #print(sol)
#     dst_c = np.absolute(sol)
#     dst_dc = 0

#     # equation for compute the slag
#     dst_zABd = dst_c*(np.cosh(dst_x_bar_d/(2*dst_c)) - 1)

#     # relative positions
#     xAdv = np.array([-dst_x_bar_d/2, 0, dst_zABd], dtype='f')
#     xBdv = np.array([dst_x_bar_d/2, 0, dst_zABd], dtype='f')

#     ## Desired positions to vectors

#     dst_xAd = dst_xCd + dst_R.dot(xAdv)
#     dst_xBd = dst_xCd + dst_R.dot(xBdv)

#    # dst_dxAd = dst_dxCd + dst_dR.dot(xAdv) + dst_R.dot(dxAdv)
#    # dst_dxBd = dst_dxCd + dst_dR.dot(xBdv) + dst_R.dot(dxBdv)

#     return dst_xAd, dst_xBd, dst_yaw


def trajectory(t, k, x0, y0):
    xd_log = np.array([k * np.cos(t) + x0, k * np.sin(t) + y0])
    return xd_log

def trajectory_robots_exp1(t, ell, yaw):

    p1 = [0 ,-0.5]
    p2 = [-.5 ,0.1]
    p3 = [0.5 ,0.5]

    xp1 = trajectory(t, 0.1, p1[0], p1[1])
    xp2 = trajectory(t, 1, p2[0], p2[1])
    xp3 = trajectory(t, 1, p3[0], p3[1])

    C1 = Quasi_satic_class(xd=np.array([xp1, p2, p3]), ell=ell)
    C1.quasi_static()

    ## Desired positions to vectors
    z = 0.5

    dst_xAd = np.array([C1.A[0], C1.A[1], z])
    dst_xBd = np.array([C1.B[0], C1.B[1], z])
    dst_xCd = np.array([C1.C[0], C1.C[1], z])
    dst_xDd = np.array([C1.D[0], C1.D[1], z])
    dst_xEd = np.array([C1.E[0], C1.E[1], z])
    dst_xFd = np.array([C1.F[0], C1.F[1], z])

    dst_yaw = yaw

   # dst_dxAd = dst_dxCd + dst_dR.dot(xAdv) + dst_R.dot(dxAdv)
   # dst_dxBd = dst_dxCd + dst_dR.dot(xBdv) + dst_R.dot(dxBdv)

    return dst_xAd, dst_xBd, dst_xCd, dst_xDd, dst_xEd, dst_xFd, dst_yaw



