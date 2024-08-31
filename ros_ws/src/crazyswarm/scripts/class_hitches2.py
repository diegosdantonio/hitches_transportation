import matplotlib; matplotlib.use("TkAgg")
import numpy as np
import matplotlib.pyplot as plt
from celluloid import Camera


class Quasi_satic_class:
    def __init__(self, pd):

        self.pd = pd
        self.lines = self.pd[1:] - self.pd[:-1]



    def f1(self, alpha, T1, T2, thetaC1, thetaC2):
        c1 = np.cos(thetaC1) + np.cos(thetaC2)
        c2 = np.sin(thetaC1) + np.sin(thetaC2)

        return (c1**2 + c2**2
                + 2 * c1 * T1 * np.cos(alpha)
                + 2 * c2 * T1 * np.sin(alpha))


    def ploting(self,axes, camera, xd_log = None):

        for i, x in enumerate(self.pd):
            # pd = np.asarray(pd[0])
            # axes.plot(self.pd[i, 0], self.pd[i, 1], 'ok')

            if i < len(self.pd) - 1:
                axes.plot([self.pd[i, 0], self.pd[i + 1, 0]], [self.pd[i, 1], self.pd[i + 1, 1]], '-k')

        for i, p in enumerate(self.pd[:-2]):
            p = self.pd[i:i + 3]
            self.In = intersection(p)
            self.In.ploting(axes)

        axes.plot([self.pd[0, 0], 5.5], [self.pd[0, 1], -1], '-k')
        axes.plot([self.pd[4, 0], 4.5], [self.pd[4, 1], 6], '-k')
        axes.plot(5.5, -1, 'ok')
        axes.plot(4.5, 6, 'ok')

        axes.set_xlabel('x(m)')
        axes.set_ylabel('y(m)')

        axes.plot(xd_log[0, :], xd_log[1, :], '--g', alpha=0.3)

        camera.snap()
        axes.axis('equal')


class intersection:
    def __init__(self, p, l=5):

        self.p0 = p[1]
        self.p1 = p[0]
        self.p2 = p[2]

        self.p = p

        self.theta1 = np.arctan2(self.p0[1] - self.p1[1],
                               self.p0[0] - self.p1[0])

        self.theta2 = np.arctan2(self.p0[1] - self.p2[1],
                               self.p0[0] - self.p2[0])

        # self.theta3 = optimize.bisect(self.f1, -np.pi / 2, np.pi / 2, args=(1, 1, self.theta1, self.theta2))
        # self.theta4 = np.arccos(np.pi - (np.cos(self.theta1) + np.cos(self.theta2) + np.cos(self.theta3)))

        self.A = np.array([((l - np.linalg.norm(self.p0[1] - self.p2[1]))/2) * np.cos(self.theta1),
                           ((l - np.linalg.norm(self.p0[1] - self.p2[1]))/2) * np.sin(self.theta1)]) + self.p0

        self.B = np.array([((l - np.linalg.norm(self.p0[1] - self.p1[1]))/2) * np.cos(self.theta2),
                            ((l - np.linalg.norm(self.p0[1] - self.p1[1]))/2) * np.sin(self.theta2)]) + self.p0

        print(self.A, self.B)



    def f1(self, alpha, T1, T2, thetaC1, thetaC2):
        c1 = -np.cos(thetaC1) + np.cos(thetaC2)
        c2 = np.sin(thetaC1) + np.sin(thetaC2)

        return (c1**2 + c2**2
                + 2 * c1 * T1 * np.cos(alpha)
                - 2 * c2 * T1 * np.sin(alpha))

    def ploting(self, axes):
        axes.plot([self.A[0], self.p0[0]], [self.A[1], self.p0[1]], 'k')
        axes.plot([self.B[0], self.p0[0]], [self.B[1], self.p0[1]], 'k')
        # axes.plot(self.p0[0], self.p0[1], 'ok')
        axes.plot(self.A[0], self.A[1], 'ok')
        axes.plot(self.B[0], self.B[1], 'ok')






def trajectory(t, k, x0, y0):
    xd_log = np.array([k * np.cos(t) + x0, k * np.sin(t) + y0])
    return xd_log

fig, axes = plt.subplots()
plt.grid(True)
camera = Camera(fig)

# time
Time = np.linspace(0,2*np.pi,100,endpoint=False)

p1 = [0, 2]
# xp1 = trajectory(0, 0.5, p1[0], p1[1])
# pn = np.array([[6 ,1], [3 ,0], xp1, [2 ,5], [5, 4]])
#
#
# # pn = np.array([[6 ,1], [3 ,0], [0 ,2]])
#
#
#
#
# C1 = Quasi_satic_class(pn)
# C1.ploting(axes,camera)

xd_log = trajectory(Time, 0.5, p1[0], p1[1])

for t, i in zip(Time, range(len(Time))):
    xp1 = trajectory(t, 0.5, p1[0], p1[1])
    # xp2 = trajectory(t, 1, p2[0], p2[1])
    # xp3 = trajectory(t, 1, p3[0], p3[1])

    pn = np.array([[6, 1], [3, 0], p1, [2, 5], [5, 4]])

    C1 = Quasi_satic_class(pn)
    C1.ploting(axes, camera, xd_log)

animation = camera.animate()

animation.save('hitch.gif')
plt.show()