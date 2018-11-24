#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import matplotlib.pyplot as plt

class simple_model(object):
    def __init__(self):
        #This model assumes that the plane has an initial speed and is all ready hooked to the springs at t=0
        self.time = 0
        self.delta_t = 0.001

    def plane_x_to_spring_stretch(self, plane_x, distance_to_mount_point):
        return math.sqrt(plane_x**2+distance_to_mount_point**2)



class fixed_wing(object):
    def __init__(self):
        self.weight = 2.6
        self.init_speed = 17
        self.init_ke = 0.5*self.weight*self.init_speed**2
        self.speed = self.init_speed
        self.x = 0
class spring(object): #https://www.thespringstore.com/pe028-375-94768-mw-3160-mh-n-in.html?unit_measure=me
    def __init__(self):
        self.neutral_length = 80/1000
        self.spring_constant = 42

        self.pre_stretched = 0.1
        self.current_stretch = self.pre_stretched
        self.spring_force = -self.spring_constant*self.current_stretch
        self.energy_stored = 0.5*self.spring_constant*self.current_stretch**2
        self.spring_angle = 0




if __name__ == "__main__":


    model = simple_model()

    l_s = spring()
    r_s = spring()
    plane = fixed_wing()
    while model.time < 6:
        #Looking at the planes movement in delta t
        delta_x = plane.speed*model.delta_t
        plane.x += delta_x
        current_spring_stretch = model.plane_x_to_spring_stretch(plane.x, 0.1+80/1000)
        l_s.current_stretch = current_spring_stretch
        r_s.current_stretch = current_spring_stretch
        total_spring_force = l_s.spring_force + r_s.spring_force

        #This force will accelerate the plane
        a = total_spring_force/plane.weight
        delta_v = a*model.delta_t
        plane.speed += delta_v
        model.time += model.delta_t
        print "a", a, "speed", plane.speed, "x", plane.x




