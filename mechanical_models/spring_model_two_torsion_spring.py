#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This spring used: https://www.thespringstore.com/pe028-375-94768-mw-3160-mh-n-in.html?unit_measure=me

Revision
YYYY-MM-DD
2018-11-08 MB First
2018-11-09 MW Added spring parameters
2018-02-12 MW Created new document with two torsion springs
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
        self.plane_pos_l = []
        self.plane_vel_l = []
        self.plane_acc_l = []
        self.arm_ang_l = []
        self.rope_len_l = []
        self.time_l = []
        self.spring_l_u_e_l = []
        self.spring_r_u_e_l = []
        self.plane_k_e_l = []
        self.total_force_l = []
        self.all_purpose_l = []

        self.spring_init_vel = 0

        self.time = 0.0
        self.delta_t = 0.01
        self.rope_len = 0.01
        self.plane_dir = np.array([0, -1])
        self.plane_mass = 1.2
        self.plane_pos = np.array([0.15, 0.05])
        self.plane_vel = np.array([0, -17])
        self.plane_acc = np.array([0, 0])
        self.plane_displacement = 0
        self.plane_k_e = 1/2 * self.plane_mass * self.plane_vel**2
        self.spring_k = 42
        self.spring_equilibrium = math.pi/2
        self.spring_n = 2#26*0.1 # 26 springs has a max load of 308

        self.spring_l_ang_rope = math.pi/2
        self.spring_l_arm = 0.35  # this should be from the middle of the cylinder to the pulley
        self.spring_l_anchor = np.array([0, 0])
        self.spring_l_center_pos = np.array([0.05, 0])
        self.spring_l_end_point = np.array([0.05, 0.05])
        self.spring_l_eq = math.pi/2

        self.spring_r_ang_rope = math.pi/2
        self.spring_r_arm = 0.35  # meters
        self.spring_r_anchor = np.array([0.3, 0])
        self.spring_r_center_pos = np.array([0.25, 0])
        self.spring_r_end_point = np.array([0.25, 0.05])
        self.spring_r_eq = math.pi/2

        # self.spring_l_u_e = 1 / 2 * self.spring_k * self.spring_l_angle_to_rope ** 2
        # self.spring_r_u_e = 1 / 2 * self.spring_k * self.spring_r_angle_to_rope ** 2
        # self.max_spring_stretch = self.max_spring_stretch_method()
        self.max_spring_load = 12.900
        self.e_plane = 0
        self.e_spring = 0
        self.e_loss = 10  # percent. Might not be usaable

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

    # def max_spring_stretch_method(self):
    #     l1 = self.spring_arm_inner * math.sin(math.pi/4)
    #     l2 = self.spring_arm_outer * math.sin(math.pi/4)
    #     hyp = math.sqrt((l1 + l2)**2 + l1**2)
    #     return hyp

    def pos_to_vel_acc(self, time_list, pos_list, title=""):
        x = time_list
        # y = pos_list
        y = [item[1] for item in pos_list]
        # print("y", y)

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
        x = time_list
        p = potential_list
        k = kinetic_list

        plt.subplot(3, 1, 1)
        plt.title('Energy in system')
        plt.plot(x, k, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Plane', fontsize=16)

        plt.subplot(3, 1, 2)
        plt.plot(x, p, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Spring', fontsize=16)

        s = [sum(s) for s in zip(p, k)]
        # print(len(s))

        plt.subplot(3, 1, 3)
        plt.plot(x, s, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Total', fontsize=16)

        plt.show()

    def connect_rope(self, naturel_length, spring_k, anchor_point, end):
        '''
        We need to calc the direction of the rope, which we do by using the start and endpoint

        :param naturel_length:
        :param spring_k:
        :param start_anchor:
        :param end_anchor:
        :return:
        '''
        stretch = end - anchor_point
        lhat = stretch / (stretch**2).sum()**0.5
        stretch = np.linalg.norm(stretch) - naturel_length
        force = -spring_k*stretch*lhat
        return force

    def main(self):
        damping = 0.001

        # Append twice to get descrete acceleration
        self.plane_k_e_l.append(self.plane_k_e)
        # self.spring_l_u_e_l.append(self.spring_l_u_e)
        # self.spring_r_u_e_l.append(self.spring_r_u_e)
        self.plane_pos_l.append(self.plane_pos)
        self.plane_vel_l.append(self.spring_init_vel)
        self.plane_acc_l.append(self.plane_acc)
        self.total_force_l.append(0)
        self.all_purpose_l.append(0)
        self.time_l.append(self.time)

        damping = 0.5

        cnt = 0
        while self.time < 0.03:
            # First we have an acceleration from prev time-step then a new vel and finally a new pos,
            # lastly increment the time

            # f_external = self.plane_mass * self.plane_acc # the plane contrib is 0
            # print("start stretch:! ", self.spring_start_stretch)

            # here we calculate the angle of the connection between the spring and the rope
            ab_l = self.spring_l_center_pos - self.spring_l_anchor
            bc_l = self.spring_l_end_point - self.spring_l_center_pos
            self.spring_l_ang_rope = math.acos((np.dot(ab_l, bc_l))/(np.linalg.norm(ab_l)*np.linalg.norm(bc_l)))

            ab_r = self.spring_r_center_pos - self.spring_r_anchor
            bc_r = self.spring_r_end_point - self.spring_r_center_pos
            self.spring_r_ang_rope = math.acos((np.dot(ab_r, bc_r))/(np.linalg.norm(ab_r)*np.linalg.norm(bc_r)))

            # The hook point is where the plane hooks onto the rope
            hook_point = self.plane_pos

            f_rope_l = model.connect_rope(self.rope_len, 100, self.spring_l_end_point, hook_point)
            f_rope_r = model.connect_rope(self.rope_len, 100, self.spring_r_end_point, hook_point)

            f_damping_left = -self.plane_vel*damping
            f_damping_right = -self.plane_vel*damping

            tau_l_spring = -self.spring_k * (self.spring_l_ang_rope - self.spring_l_eq)  # The torsion spring contrib
            tau_r_spring = -self.spring_k * (self.spring_r_ang_rope - self.spring_r_eq)  # The torsion spring contrib

            f_l_spring = tau_l_spring/(self.spring_l_arm * self.spring_l_ang_rope)
            f_r_spring = tau_r_spring/(self.spring_r_arm * self.spring_r_ang_rope)

            f_spring_system = f_rope_l + f_rope_r + \
                              f_damping_left + f_damping_right + \
                              f_l_spring + f_r_spring

            print("l end", self.spring_l_end_point)
            print("r end", self.spring_r_end_point)
            print("pos", self.plane_pos)
            # print(self.plane_pos)
            self.plane_acc = f_spring_system / self.plane_mass
            self.plane_vel = self.plane_vel + self.plane_acc * self.delta_t
            self.plane_pos = self.plane_pos + self.plane_vel * self.delta_t

            # print("plane pos", self.plane_pos[1])

            new_l_x_length = (abs(self.plane_pos[1]**2 - self.rope_len**2))**0.5
            print("new_l_x_length ", new_l_x_length )
            self.spring_l_end_point[0] = self.spring_l_center_pos[0] + new_l_x_length - self.rope_len
            self.spring_l_end_point[1] = abs((self.spring_l_end_point[0] ** 2 - self.spring_l_arm ** 2)) ** 0.5
            new_r_x_length = (abs(self.rope_len**2 - self.plane_pos[1]**2))**0.5
            self.spring_r_end_point[0] = self.spring_r_center_pos[0] - (self.rope_len - new_r_x_length)
            self.spring_r_end_point[1] = abs((self.spring_r_end_point[0] ** 2 - self.spring_l_arm ** 2)) ** 0.5
            # self.spring_r_end_point
            # print("new l rope len:", new_l_x_length)
            # print("new r rope len:", new_r_x_length)


            # self.plane_k_e = 0.5 * self.plane_mass * self.plane_vel**2
            # self.spring_u_e = 0.5 * self.spring_k * self.plane_pos**2

            self.time += self.delta_t

            # self.plane_k_e_l.append(self.plane_k_e)
            # self.spring_u_e_l.append(self.spring_u_e)
            self.plane_pos_l.append(self.plane_pos)
            self.plane_vel_l.append(self.plane_vel)
            self.plane_acc_l.append(self.plane_acc)
            self.total_force_l.append(f_spring_system)
            self.time_l.append(self.time)

            cnt += 1

        # self.pos_to_vel_acc(self.time_l, self.plane_pos_l, "Two springs")
        # self.pos_to_vel_acc(self.time_l, self.all_purpose_l, "spring stretch disregard two last plots")
        # self.total_energy(self.plane_k_e_l, self.spring_u_e_l, self.time_l) # k_u is not correct
        # self.spring_forcediagram()


if __name__ == "__main__":
    model = ar_model()
    model.main() # spring_forcediagram()