import matplotlib; matplotlib.use("TkAgg")
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np

class circle_traj:
    def __init__(self, xd, theta, r):
        self. xd = xd 
        self.theta = theta
        self.xcw1 = [] 
        self.xcw2 = []  
        self.r = r  
        self.ell = 2

    def Rz(self, theta):
       # Returns the rotation matrix for a rotation around the z-axis by 'theta' radians
      return np.array([[ np.cos(theta), -np.sin(theta), 0],
                        [ np.sin(theta),  np.cos(theta), 0],
                        [ 0           , 0            , 1 ]])

    def circle(self, t, r=1):
        # Returns a point on a circle at parameter 't' (angle) with radius 'r'
        # The circle is parallel to the xz-plane
        return np.array([r * np.sin(t),
                         0*t,
                         r * np.cos(t)])

    def intersection(self, p0, p1, p2):
        theta1 = np.arctan2(p0[2] - p1[2], p0[0] - p1[0])
        theta2 = np.arctan2(p0[2] - p2[2], p0[0] - p2[0])

        ellA = (self.ell - np.linalg.norm(p0, p1)) / 2
        ellB = (self.ell - np.linalg.norm(p0, p2)) / 2

        A = np.array([ellA * np.cos(theta1), 0, ellA * np.sin(theta1)]) + p0
        B = np.array([ellB * np.cos(theta2), 0, ellB * np.sin(theta2)]) + p0

        C = A + (B - A)/2
        return [C]

    def update(self, t, ax=[]):
        self.xcw1 = []
        self.xcw2 = []

        # print(xc2)
        # print(len(self.theta))
        for i in range(len(self.theta)):
            xc1 = self.circle(t + np.pi / 2, self.r[i])
            xc2 = self.circle(t + 3 * np.pi / 2, self.r[i])
            self.xcw1.append(self.xd[i] + xc1.dot(self.Rz(self.theta[i])))
            self.xcw2.append(self.xd[i] + xc2.dot(self.Rz(self.theta[i])))



        self.xcw1 = np.asarray(self.xcw1)
        self.xcw2 = np.asarray(self.xcw2)

        self.all = np.array([self.xcw1[0], self.xcw2[0]
                            ,self.xcw1[1], self.xcw2[1]
                            ,self.xcw1[2], self.xcw2[2]])

        # self.all = []
        #
        # for xcw1, xcw2 in zip(self.xcw1,self.xcw2):
        #     self.all.append(xcw1)
        #     self.all.append(xcw2)


fig = plt.figure(figsize = (10,10))
ax = plt.axes(projection='3d')

# Time = np.arange(0, np.pi/2, np.pi/50)
#
# xd = np.array([[1., 0., 0.],
#                [0., 1., 0.],
#                [1., 1., 0.]])
#
# xd_theta = np.array([0, np.pi/4, 2*np.pi/4 ])
# C = circle_traj(xd, xd_theta)
#
# for t in Time:
#     C.update(t, ax=ax)
#     ax.plot3D(C.xcw1[:, 0], C.xcw1[:, 1], C.xcw1[:, 2], '.')
#     ax.plot3D(C.xcw2[:, 0], C.xcw2[:, 1], C.xcw2[:, 2], '.')
#
#
#
# plt.show()