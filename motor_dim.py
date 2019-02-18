#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import matplotlib.pyplot as plt

def fw_x_to_motor_r(fw_x):
    #Lets define the distance between the two arms to 2 meter
    half_d = 1
    #Then the amount of rope unwinded can be found by pythagoras
    rope_unwinded =  math.sqrt(half_d**2+fw_x**2)
    max_rope_unwinded = 5.09901951359
    #The amount of rope left coiled up is then
    coiled_rope = max_rope_unwinded - rope_unwinded
    #By approximating how many times the rope is winded it is approximated by circles stacked on top of each other with the distance equal to the ropes diameter
    rope_d = 0.005
    start_d = 0.02
    diameter = start_d
    approx = 0
    #print("Coiled rope", coiled_rope)
    while approx < coiled_rope:
        approx += math.pi*diameter
        diameter += 2*rope_d

    return diameter

fw_mass = 1
fw_speed = 10.0
braking_time = 0.001
braking_time = 5.0/fw_speed

fw_momentum = fw_speed*fw_mass

braking_force_two_motors = fw_momentum/braking_time

braking_force_one_motor = braking_force_two_motors/2

fw_position = 0

#Now lets step through the process with small time steps

delta_t = 0.001
position_list = []
speed_list = []
torque_list = []
time_list = []
time = 0

while(fw_speed > 0):
    fw_position += fw_speed*delta_t
    position_list.append(fw_position)
    motor_diameter = fw_x_to_motor_r(fw_position)
    motor_radius = motor_diameter/2
    torque = braking_force_one_motor*motor_radius
    time += delta_t
    time_list.append(time)
    #As each motor is expected to change the momentum by this amount we multiply by two
    delta_momentum = (braking_force_one_motor*delta_t)*2

    fw_momentum -= delta_momentum
    fw_speed = fw_momentum/fw_mass
    speed_list.append(fw_speed)
    torque_list.append(torque)

fig = plt.figure()
plt.plot(time_list, speed_list, 'ro')
fig.suptitle('fw speed', fontsize=20)
plt.xlabel('Time since hooking', fontsize=18)
plt.ylabel('Fw speed', fontsize=16)
plt.show()

fig = plt.figure()
plt.plot(time_list, position_list, 'ro')
fig.suptitle('fw position', fontsize=20)
plt.xlabel('Time since hooking', fontsize=18)
plt.ylabel('Fw position', fontsize=16)
plt.show()

fig = plt.figure()
plt.plot(time_list, torque_list, 'ro')
fig.suptitle('motor torque', fontsize=20)
plt.xlabel('Time since hooking', fontsize=18)
plt.ylabel('motor torque', fontsize=16)
plt.show()