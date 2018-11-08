#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import matplotlib.pyplot as plt

#We're using SI units
class ar_model(object):
    def __init__(self):
        self.time = 0
        self.plane_init_speed = 17.0
        self.plane_speed = 17.0
        self.plane_weight = 2.3
        self.spring_constant = 42
        self.plane_speed_list = []
        self.time_list = []
        self.displacement = 0
        self.init_plane_e = 0
        self.inner_spring_arm = 0.105
        self.outer_spring_arm = 0.2165
        self.half_distance_arms = 1

    def spring_energy(self, displacement):
        return 0.5*self.spring_constant*(displacement)**2

    def plane_energy(self, plane_speed):
        return 0.5*self.plane_weight*(plane_speed)**2

    def angle_to_spring_stretch(self, angle):
        return math.sqrt((self.inner_spring_arm*math.sin(angle))**2+(-self.inner_spring_arm*math.cos(angle)+self.outer_spring_arm)**2)

    def displacement_to_angle(self, displacement):
        return math.atan(displacement/self.half_distance_arms)

def main():
    model = ar_model()
    model.init_plane_e = model.plane_energy(model.plane_speed)
    counter = 0
    while counter < 40:
        delta_time = 0.01
        delta_displacement = model.plane_speed*delta_time

        model.displacement += delta_displacement



        model.time += delta_time
        #model.displacement += 0.01*model.plane_speed
        print model.displacement
        current_angle = model.displacement_to_angle(model.displacement)
        print "Current angle[degrees]: ", current_angle*180/3.14
        spring_displacement = model.angle_to_spring_stretch(current_angle)
        print "Spring_displacement: ", spring_displacement
        e_spring = model.spring_energy(model.displacement)
        print "E_spring: ", e_spring
        e_plane = model.plane_energy(model.plane_speed)
        new_e_plane = model.init_plane_e - e_spring
        print "E_plane: ", new_e_plane
        try:
            new_plane_speed = math.sqrt(new_e_plane/(0.5*model.plane_weight))
            print "Plane speed: ", new_plane_speed
        except:
            print "can't do sqrt to a negative number, exiting model loop"
            break
        else:
            model.plane_speed = new_plane_speed
            model.plane_speed_list.append(new_plane_speed)
            model.time_list.append(model.time)
        counter += 1


    fig = plt.figure()
    plt.plot(model.time_list, model.plane_speed_list, 'ro')
    fig.suptitle('Plane speed', fontsize=20)
    plt.xlabel('Time since hooking', fontsize=18)
    plt.ylabel('Plane speed', fontsize=16)
    plt.show()

if __name__ == "__main__":
    main()

