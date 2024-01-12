#! /usr/bin/python
# -*- coding: utf-8 -*-
u"""
Cubic Spline library on python

author Atsushi Sakai

usage: see test codes as below

license: MIT
"""
import math
import numpy as np
import bisect
import matplotlib.pyplot as plt

class Spline:
    u"""
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                 (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        u"""
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
                self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0
        return result

    def calcd(self, t):
        u"""
        Calc first derivative

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        u"""
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def calcddd(self, t):
        u"""
        Calc third derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        result = 6.0 * self.d[i]
        return result

    def __search_index(self, x):
        u"""
        search data segment index
        """
        res = bisect.bisect(self.x, x)
        if res == len(self.x):
            res -= 1
        return res - 1

    def __calc_A(self, h):
        u"""
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        u"""
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                       h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        #  print(B)
        return B


class Spline2D:
    u"""
    2D Cubic Spline class

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        print(self.s)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        u"""
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        u"""
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / (dx ** 2 + dy ** 2) ** (3 / 2)

        return k

    def calc_curvature_d(self, s):
        u"""
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dddx = self.sx.calcddd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        dddy = self.sy.calcddd(s)

        k_d = (
                      (dddy * dx - dddx * dy) * (dx ** 2 + dy ** 2) -
                      3 * (dx * ddy - ddx * dy) * (dx * ddx + dy * ddy)
              ) / ((dx ** 2 + dy ** 2) ** (5 / 2))

        return k_d

    def calc_yaw(self, s):
        u"""
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw


class Spline3D:
    """
    3D Cubic Spline class
    """

    def __init__(self, x, y, z):
        self.s = self.__calc_s(x, y, z)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)
        self.sz = Spline(self.s, z)

    def __calc_s(self, x, y, z):
        dx = np.diff(x)
        dy = np.diff(y)
        dz = np.diff(z)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2 + idz ** 2) for (idx, idy, idz) in zip(dx, dy, dz)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        u"""
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)
        z = self.sz.calc(s)
        return x, y, z

    def calc_curvature(self, s):
        u"""
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / (dx ** 2 + dy ** 2) ** (3 / 2)
        return k

    def calc_yaw(self, s):
        u"""
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw

    def calc_pitch(self, s):
        """
        calc pitch - this function needs to be double checked
        """
        dx = self.sx.calcd(s)
        dz = self.sz.calcd(s)
        pitch = math.atan2(dz, dx)
        return pitch


def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s


if __name__ == "__main__":
    x = [568.3755267385674, 568.069599096251, 567.7159641407602, 567.2861605946557, 566.7899372548937, 566.2019180955712, 565.4791297930967, 564.6235010096929, 563.6662434368087, 562.6386162335656, 561.5523682848572, 560.4192338245213, 559.2626695090979, 558.0748682443741, 556.8792722794622, 555.6758782765503, 554.4646933880451, 553.2535154083188, 552.0501604843065, 550.8468124725637]
    y = [20.612875450616073, 20.573559010645532, 20.556466834289118, 20.540362382904746, 20.522129601854225, 20.504376658664313, 20.486896473731946, 20.46503756242896, 20.43371328674828, 20.394060506376896, 20.350152773291583, 20.30484396462284, 20.2606651469128, 20.21864386304, 20.17960368544594, 20.142567573074246, 20.108512539055837, 20.07592233742271, 20.044745470815784, 20.015033913418506]
    csp = Spline2D(x, y)
    horizon = 50
    s = np.arange(0, csp.s[-1], csp.s[-1] / horizon)
    spline_x = []
    spline_y = []
    for idx, ss in enumerate(s):
        ox, oy = csp.calc_position(ss)
        oyaw = csp.calc_yaw(ss)
        spline_x.append(ox)
        spline_y.append(oy)
    plt.plot(x, y, "xb", label="input")
    plt.plot(spline_x, spline_y, "-r", label="spline")
    plt.show()
    
    # s = [0, 0.30844368170743924, 0.6624914528309723, 1.0925966046234032, 1.5891547958048178, 2.1774418844973567, 2.9004415297663946, 3.756349484509496, 4.714119431360494, 5.742511385741932, 6.8296463797044895, 7.963686323156087, 9.121094109394331, 10.309638442929188, 11.505871633732847, 12.70983542050712, 13.921498979208648, 15.133115345810102, 16.336874071699977, 17.5405888277815]
    # print(bisect.bisect(s, 17.5405888277815))