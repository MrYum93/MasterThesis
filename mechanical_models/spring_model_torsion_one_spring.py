#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This spring used: https://www.thespringstore.com/pe028-375-94768-mw-3160-mh-n-in.html?unit_measure=me

Revision
YYYY-MM-DD
2018-11-08 MB First
2018-11-09 MW Added spring parameters
2018-11-25 MW Now the spring catches the plane and dampens it by some random number
              that is later to be the motors of the system
'''

import math
import numpy as np
import matplotlib.pyplot as plt
import sys

# defines
EPS = sys.float_info.min


# We're using SI units
class ar_model(object):
    def __init__(self):
        # lists
        self.plane_ang_pos_l = []
        self.plane_ang_vel_l = []
        self.plane_ang_acc_l = []
        self.arm_ang_l = []
        self.rope_len_l = []
        self.time_l = []
        self.spring_u_e_l = []
        self.plane_k_e_l = []

        self.spring_init_vel = 0
        self.plane_init_vel = 17

        self.time = 0.0
        self.delta_t = 0.0001
        self.plane_mass = 2.3
        # self.plane_pos = 0.105  # where the mass / plane is hooked on
        # self.plane_vel = 17.0
        # self.plane_acc = 0
        self.plane_ang_pos = math.pi/2
        self.plane_ang_vel = 17
        self.plane_ang_acc = 0
        self.plane_displacement = 0
        self.plane_on_spring_angle = math.pi/2  # in rad
        self.spring_k = 42
        self.angle_equilibrium = math.pi/2  # in rad
        self.spring_n = 26*0.1 # 26 springs has a max load of 308
        self.spring_radius = 0.05 # 5 cm
        self.spring_arm_inner = 0.105
        self.spring_arm_outer = 0.2165
        self.spring_start_stretch = self.spring_arm_outer - self.spring_arm_inner
        self.plane_k_e = 0 #1 / 2 * self.plane_mass * self.spring_radius ** 2 * self.plane_ang_vel ** 2
        self.spring_u_e = 1 / 2 * self.spring_k * self.plane_ang_pos ** 2
        self.max_spring_load = 12.900
        self.e_plane = 0
        self.e_spring = 0
        self.e_loss = 10  # percent. Might not be usaable
        self.dist_between_arms = 1

    def spring_forcediagram(self):
        start_f = 0
        end_f = self.max_spring_load
        curr_f = 0
        f_list = []
        curr_dist = 0
        dist_list = []
        while curr_f < end_f:
            curr_f = self.spring_k * curr_dist
            curr_dist += 0.01
            f_list.append(curr_f)
            dist_list.append(curr_dist)

        fig = plt.figure()
        plt.plot(dist_list, f_list, 'r')
        fig.suptitle('Spring force / displacement', fontsize=20)
        plt.xlabel('Distance[m]', fontsize=18)
        plt.ylabel('Force', fontsize=16)
        plt.show()

    def pos_to_vel_acc(self, time_list, pos_list, title=""):
        x = time_list
        y = pos_list

        dy = np.zeros(y.__len__(), np.float)
        dyy = np.zeros(y.__len__(), np.float)
        dy[0:-1] = np.diff(y) / np.diff(x)
        dyy[0:-1] = np.diff(dy) / np.diff(x)
        dy[-1] = (y[-1] - y[-2]) / (x[-1] - x[-2])
        dyy[-1] = (dy[-1] - dy[-2]) / (x[-1] - x[-2])

        # Remove the last two points bc of spike in acceleration
        x = x[:-2]
        y = y[:-2]
        dy = dy[:-2]
        dyy = dyy[:-2]

        plt.subplot(3, 1, 1)
        plt.title(title)
        plt.plot(x, y, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Pos', fontsize=16)

        plt.subplot(3, 1, 2)
        plt.plot(x, dy, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Vel', fontsize=16)

        plt.subplot(3, 1, 3)
        plt.plot(x, dyy, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Acc', fontsize=16)

        plt.show()

    def total_energy(self, kinetic_list, potential_list, time_list):
        # First step is "wierd" bc there is no energy in the system before the first time-step!
        plt.subplot(3, 1, 1)
        plt.title('Energy in system')
        plt.plot(time_list, kinetic_list, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Kinetic', fontsize=16)

        plt.subplot(3, 1, 2)
        plt.plot(time_list, potential_list, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Potential', fontsize=16)

        s = [sum(s) for s in zip(potential_list, kinetic_list)]
        # print(len(s))

        plt.subplot(3, 1, 3)
        plt.plot(time_list, s, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Total', fontsize=16)

        plt.show()

    def plot(self, one, two, time, title):
        x = time

        x = x[1:-1]
        one = one[1:-1]
        two = two[1:-1]

        plt.subplot(2, 1, 1)
        plt.title(title)
        plt.plot(x, one, 'r')

        plt.subplot(2, 1, 2)
        plt.plot(x, two, 'r')
        plt.xlabel('Time since hooking', fontsize=18)

        plt.show()

    def khan_acadamy_spring_system(self):
        # https://www.khanacademy.org/partner-content/pixar/simulation/hair-simulation-code/pi
        # /step-3-damped-spring-mass-system
        gravity = 0
        mass = 2.3
        positionY = 200
        velocityY = 17
        timeStep = self.delta_t
        anchorX = 210
        anchorY = 100
        k = 42
        damping = 2

        while self.time < 10:
            springForceY = -k * (positionY - anchorY)
            dampingForceY = damping * velocityY
            forceY = springForceY + mass * gravity - dampingForceY
            accelerationY = forceY / mass
            velocityY = velocityY + accelerationY * timeStep
            positionY = positionY + velocityY * timeStep

            print(positionY)

            self.time += self.delta_t

            self.plane_ang_pos_l.append(self.plane_ang_pos)
            self.plane_ang_vel_l.append(self.plane_ang_vel)
            self.plane_ang_acc_l.append(self.plane_ang_acc)
            self.time_l.append(self.time)

        self.pos_to_vel_acc(self.time_l, self.plane_pos_l, "Plane")

    def main(self):
        # Append once to get discrete differentiation
        self.plane_k_e_l.append(self.plane_k_e)
        self.spring_u_e_l.append(self.spring_u_e)
        self.plane_ang_pos_l.append(self.plane_ang_pos)
        self.plane_ang_vel_l.append(self.plane_ang_vel)
        self.plane_ang_acc_l.append(self.plane_ang_acc)
        self.time_l.append(self.time)

        damping = 0#.5

        cnt = 0
        while self.time < 5:
            # First we have an acceleration from prev time-step then a new vel and finally a new pos,
            # lastly increment the time

            # print("pos old vs new", self.plane_ang_pos_l[cnt-1], self.plane_ang_pos_l[cnt])
            # print("vel old vs new", self.plane_ang_vel_l[cnt-1], self.plane_ang_vel_l[cnt])
            # print("acc old vs new", self.plane_ang_acc_l[cnt-1], self.plane_ang_acc_l[cnt])

            tau_spring = -self.spring_k * (self.plane_ang_pos - self.angle_equilibrium)  # The torsion spring contrib
            f_spring = tau_spring/(self.spring_radius * self.plane_on_spring_angle)
            f_damper = - damping * self.plane_ang_vel  # the dampening
            f_spring_system = f_spring + f_damper

            self.plane_ang_acc = (f_spring_system/self.plane_mass)/self.spring_radius
            self.plane_ang_vel = self.plane_ang_vel + self.plane_ang_acc * self.delta_t
            self.plane_ang_pos = self.plane_ang_pos + self.plane_ang_vel * self.delta_t

            self.plane_k_e = tau_spring * self.plane_ang_pos #1/2 * self.plane_mass * self.spring_radius**2 * self.plane_ang_vel**2
            self.spring_u_e = 1/2 * self.spring_k * self.plane_ang_pos**2

            print("ke", self.plane_k_e)
            print("ue", self.spring_u_e)
            # print("tau_sp", tau_spring)
            # print("F_dm", f_damper)
            # print("F_ss", f_spring_system)

            self.time += self.delta_t

            self.plane_k_e_l.append(self.plane_k_e)
            self.spring_u_e_l.append(self.spring_u_e)
            self.plane_ang_pos_l.append(self.plane_ang_pos)
            self.plane_ang_vel_l.append(self.plane_ang_vel)
            self.plane_ang_acc_l.append(self.plane_ang_acc)
            self.time_l.append(self.time)

            print("")

            cnt += 1
    
        self.pos_to_vel_acc(self.time_l, self.plane_ang_pos_l, "One spring advanced")
        self.total_energy(self.plane_k_e_l, self.spring_u_e_l, self.time_l)


if __name__ == "__main__":
    model = ar_model()
    model.main() # spring_forcediagram()
