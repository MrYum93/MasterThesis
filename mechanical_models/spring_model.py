#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This spring used: https://www.thespringstore.com/pe028-375-94768-mw-3160-mh-n-in.html?unit_measure=me

Revision
YYYY-MM-DD
2018-11-08 MB First
2018-11-09 MW Added spring parameters
'''

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

#We're using SI units
class ar_model(object):
    def __init__(self):
        self.time = 0
        self.plane_init_speed = 17.0
        self.plane_speed = 17.0
        self.plane_weight = 2.3
        self.spring_constant = 42
        self.spring_n = 26 # 26 springs has a max load of 308
        self.plane_speed_list = []
        self.time_list = []
        self.displacement_plane = 0
        self.init_plane_e = 0
        self.inner_spring_arm = 0.105
        self.outer_spring_arm = 0.2165
        self.spring_start_stretch = self.outer_spring_arm - self.inner_spring_arm
        self.equilibrium = 0.080 # spring resting pos
        self.max_spring_stretch = 0.279400
        self.max_spring_load = 12.900
        self.half_distance_arms = 1
        self.gen_cnt = 0
        self.old_time = 0
        self.old_vel = 0

    def spring_forcediagram(self):
        start_f = 0
        end_f = self.max_spring_load
        curr_f = 0
        f_list = []
        curr_dist = 0
        dist_list = []
        while curr_f < end_f:
            curr_f = self.spring_constant * curr_dist
            curr_dist += 0.01
            f_list.append(curr_f)
            dist_list.append(curr_dist)

        fig = plt.figure()
        plt.plot(dist_list, f_list, 'r')
        fig.suptitle('Spring force / displacement', fontsize=20)
        plt.xlabel('Distance[m]', fontsize=18)
        plt.ylabel('Force', fontsize=16)
        plt.show()

    def spring_force(self, displacement):
        pass

    def force_to_acc(self, force, weight):
        return force/weight

    def spring_energy(self, displacement):
        return 0.5*self.spring_constant*(displacement)**2

    def plane_energy(self, plane_speed):
        return 0.5*self.plane_weight*(plane_speed)**2

    def angle_to_spring_stretch(self, angle):
        return math.sqrt((self.inner_spring_arm*math.sin(angle))**2+(-self.inner_spring_arm*math.cos(angle)+self.outer_spring_arm)**2)

    def displacement_to_angle(self, displacement):
        return math.atan(displacement/self.half_distance_arms)

    def pos_to_acc(self, pos):
        return -(self.spring_constant / self.plane_weight) * pos

    def mass_spring_odeint(self, init_state, t):
        # unpack the state vector
        x = init_state[0]
        xd = init_state[1]

        # these are our constants
        k = -self.spring_constant  # Newtons per metre
        m = self.plane_weight  # Kilograms
        g = self.plane_init_speed #plane_acc  # metres per second

        # compute acceleration of plane xdd
        xdd = ((k * x) / m) + g

        # return the two state derivatives
        self.old_time = t
        return [xd, xdd]

    def main(self):
        pass
        # state0 = [0.0, 17.0]#self.plane_init_speed] # location, vel
        # t = np.arange(0.0, 10.0, 0.1)
        #
        # state = odeint(self.mass_spring_odeint, state0, t)
        #
        # print("What is the state?: ", state)
        # plt.plot(t, state)
        # plt.xlabel('TIME (sec)')
        # plt.ylabel('STATES')
        # plt.title('Mass-Spring System')
        # plt.legend(('$x$ (m)', '$\dot{x}$ (m/sec)'))
        # plt.show()


def main():
    print("### Main begin ###")
    model = ar_model()
    model.init_plane_e = model.plane_energy(model.plane_speed)
    counter = 0
    while counter < 400:
        if(model.displacement_plane >= 5):
            print("plane is more than 5 meters out")
            break
        delta_time = 0.01
        delta_displacement_plane = model.plane_speed*delta_time
        model.displacement_plane += delta_displacement_plane

        acc = model.pos_to_acc(0 - delta_displacement_plane)
        print("acceleration: ", acc)
        model.time += delta_time
        # model.displacement += 0.01*model.plane_speed
        print("Plane displacement[m]: ", model.displacement_plane)
        current_arm_angle = model.displacement_to_angle(model.displacement_plane)
        print("Current arm angle[degrees]: ", current_arm_angle*180/3.14)
        displacement_spring = model.angle_to_spring_stretch(current_arm_angle)
        print("Spring_displacement[m]: ", displacement_spring)
        e_spring = model.spring_energy(displacement_spring) * model.spring_n * 2
        print("E_spring[J]: ", e_spring)
        e_plane = model.plane_energy(model.plane_speed)
        new_e_plane = model.init_plane_e - e_spring
        print("E_plane[J]: ", new_e_plane)
        try:
            new_plane_speed = math.sqrt(new_e_plane/(0.5*model.plane_weight))
            print("Old plane speed[v/m]: ", model.plane_speed)
            print("New plane speed[v/m]: ", new_plane_speed)
        except:
            print("can't do sqrt to a negative number, exiting model loop")
            break
        else:
            plane_acc = (model.plane_speed - new_plane_speed) / delta_time
            print(plane_acc)
            model.plane_speed = new_plane_speed
            model.plane_speed_list.append(new_plane_speed)
            model.time_list.append(model.time)
        print("")
        counter += 1

    fig = plt.figure()
    plt.plot(model.time_list, model.plane_speed_list, 'ro')
    fig.suptitle('Plane speed', fontsize=20)
    plt.xlabel('Time since hooking', fontsize=18)
    plt.ylabel('Plane speed', fontsize=16)
    plt.show()
    # if KeyboardInterrupt():
    #     exit()
    print("### Main end ###")


if __name__ == "__main__":
    # model = ar_model(); model.main() # spring_forcediagram()
    main()

