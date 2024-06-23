#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np


class TrapezoidalAcceleration:
    def __init__(self):
        self._ts = 0
        self._t1 = 0
        self._t2 = 0
        self._te = 0

        self._vs = 0
        self._v1 = 0
        self._v2 = 0
        self._ve = 0
        
        self._xs = 0
        self._x1 = 0
        self._x2 = 0
        self._xe = 0

    def update(self, jerk_max, acc_max, vel_s, vel_e):
        """ Update variables to connect two speeds with trapezoidal acceleration
        Args:
            jerk_max (float): the maximum jerk
            acc_max (float): the maximum acceleration
        """
        # adjust the sign of jerk_max and acc_max
        self._jerk_max = np.sign(vel_e - vel_s) * jerk_max
        self._acc_max = np.sign(vel_e - vel_s) * acc_max
        self._vs = vel_s
        self._ve = vel_e

        # cruising time
        time_c = (self._ve - self._vs) / self._acc_max - (self._acc_max / self._jerk_max)

        # Positive or negative cruise time to check if trapezoidal acceleration is possible
        if time_c > 0:
            # Trapezoidal acceleration possible
            time_a = self._acc_max / self._jerk_max

            self._ts = 0
            self._t1 = self._ts + time_a
            self._t2 = self._t1 + time_c
            self._te = self._t2 + time_a

            self._v1 = self._vs + self._acc_max * time_a / 2
            self._v2 = self._v1 + self._acc_max * time_c

            self._x1 = self._vs * time_c + self._jerk_max * time_c * time_c / 6
            self._x2 = self._x1 + self._v1 * time_c
            self._xe = self._xs + (self._vs + self._ve) / 2 * (self._te - self._ts)

            self._xs = 0
            self._x1 = self._xs + self._vs * time_a + self._acc_max * time_a**2 / 6
            self._x2 = self._x1 + self._v1 * time_c
            self._xe = self._xs + (self._vs + self._ve) / 2 * (self._te - self._ts)
        else:
            time_m = np.sqrt((self._ve - self._vs) / self._jerk_max)

            self._t1 = self._t2 = self._ts + time_m
            self._te = self._t2 + time_m

            self._v1 = self._v2 = (self._vs + self._ve) / 2
            self._x1 = self._x2 = self._xs + self._v1 * time_m + self._jerk_max * time_m * time_m * time_m / 6

            self._xe = self._xs + 2 * self._v1 * time_m

            
    def get_jerk(self, t):
        """ Calculate the jerk $j$ at time $t$.
        Args:
            t (float): time [s]
        Returns:
            float: jerk [m/s/s/s]
        """
        if t <= self._ts:
            return 0
        elif t <= self._t1:
            return self._jerk_max
        elif t <= self._t2:
            return 0
        elif t <= self._te:
            return -self._jerk_max
        else:
            return 0

    def get_acceleration(self, t):
        """ Calculate the acceleration $a$ at time $t$.
        Args:
            t (float): time [s]
        Returns:
            float: acceleration [m/s/s]
        """
        if t <= self._ts:
            return 0
        elif t <= self._t1:
            return self._jerk_max * (t - self._ts)
        elif t <= self._t2:
            return self._acc_max
        elif t <= self._te:
            return -self._jerk_max * (t - self._te)
        else:
            return 0

    def get_velocity(self, t):
        """ Calculate the velocity $v$ at time $t$.
        Args:
            t (float): time [s]
        Returns:
            float: velocity [m/s]
        """
        if t <= self._ts:
            return self._vs
        elif t <= self._t1:
            return self._vs + self._jerk_max / 2 * (t - self._ts) ** 2
        elif t <= self._t2:
            return self._v1 + self._acc_max * (t - self._t1)
        elif t <= self._te:
            return self._ve - self._jerk_max / 2 * (t - self._te) ** 2
        else:
            return self._ve

    def get_position(self, t):
        """ Calculate the position $x$ at time $t$.
        Args:
            t (float): time [s]
        Returns:
            float: position [m]
        """
        if t <= self._ts:
            return self._xs + self._vs * (t - self._ts)
        elif t <= self._t1:
            return self._xs + self._vs * (t - self._ts) + self._jerk_max / 6 * (t - self._ts) * (t - self._ts) * (t - self._ts)
        elif t <= self._t2:
            return self._x1 + self._v1 * (t - self._t1) + self._acc_max / 2 * (t - self._t1) * (t - self._t1)
        elif t <= self._te:
            return self._xe + self._ve * (t - self._te) - self._jerk_max / 6 * (t - self._te) * (t - self._te) * (t - self._te)
        else:
            return self._xe + self._ve * (t - self._te)


    def get_end_time(self):
        """ Get the end time of trapezoidal acceleration.
        Returns:
            float: end time [s]
        """
        return self._te


if __name__ == '__main__':
    jerk_max = 10
    acc_max = 10
    vel_s = 0
    vel_e = 1

    ta = TrapezoidalAcceleration()
    ta.update(jerk_max, acc_max, vel_s, vel_e)
    req_time = ta.get_end_time()

    t = np.linspace(0, req_time, 100)
    j = [ta.get_jerk(tt) for tt in t]
    a = [ta.get_acceleration(tt) for tt in t]
    v = [ta.get_velocity(tt) for tt in t]
    x = [ta.get_position(tt) for tt in t]

    fig, axs = plt.subplots(4, 1, figsize=(6, 8))
    axs[0].plot(t, j, color='C0')
    axs[0].set_title('Jark')
    axs[1].plot(t, a, color='C1')
    axs[1].set_title('Accelaration')
    axs[2].plot(t, v, color='C2')
    axs[2].set_title('Velocity')
    axs[3].plot(t, x, color='C3')
    axs[3].set_title('Position')
    plt.tight_layout()
    plt.show()