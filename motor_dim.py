#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import matplotlib.pyplot as plt

last_rope_unwinded = 0
last_omega = 0
alpha_list = []
omega_list = []
acceleration_torque_list = []
RPM_list = []
radius_list = []
length_list = []
rope_speed_list = []

def pre_compute_spiral():
    start_r = 0.015
    distance_windings = 0.005/(2*math.pi)
    theta = 0.0
    delta_theta = 0.001
    l = 0.0
    global radius_list
    global length_list
    last_r = start_r
    r = start_r + distance_windings * theta
    #The equation of the spiral will then be r = start_r+distance_windings*thetha
    #The length of the spiral is calculated as the length of a curve in polar coordinates https://www.intmath.com/blog/mathematics/length-of-an-archimedean-spiral-6595
    while l<5:
        theta += delta_theta
        r = start_r + distance_windings * theta
        radius_list.append(r)
        l += math.sqrt(r**2+((r-last_r)/delta_theta)**2)*delta_theta
        radius_list.append(r)
        length_list.append(l)
        last_r = r

def l_to_r(l):
    global radius_list
    global length_list
    #print("Went here")
    #print("Radius 0", radius_list[len(radius_list)-1], "length 0", length_list[len(length_list)-1])
    counter = 0
    for i in length_list:
        if i <= l+0.0001 and i >= l-0.0001:

            return radius_list[counter]
        counter += 1
        #print("Length", l, "Has radius", radius_list[i])

    #for i in [i for i, x in enumerate(testlist) if x == 1]:

#if x <= l+0.00000000001 or x >= l-0.00000000001]

def fw_x_to_motor_r(fw_x):
    #Lets define the distance between the two arms to 2 meter
    half_d = 1
    #Then the amount of rope unwinded can be found by pythagoras
    rope_unwinded =  math.sqrt(half_d**2+fw_x**2)
    max_rope_unwinded = 5.0000292261074595
    global length_list
    #As there is one meter rope to the point where the FW is hooked which is not supposed to ever get winded on the coil
    #this meter then needs to be subtracted
    rope_unwinded -= 1
    #The amount of rope left coiled up is then
    coiled_rope = max_rope_unwinded - rope_unwinded
    #print("Rope unwinded", rope_unwinded)
    #By approximating how many times the rope is winded it is approximated by circles stacked on top of each other with the distance equal to the ropes diameter

    r = l_to_r(coiled_rope)
    #print("Biggest length", length_list[len(length_list)-1])

    rope_d = 0.005
    start_d = 0.03
    diameter = start_d
    approx = 0
    #print("Coiled rope", coiled_rope)
    #while approx < coiled_rope:
    #    approx += math.pi*diameter
    #    diameter += 2*rope_d
    #print("diameter", diameter)

    global last_rope_unwinded
    global alpha_list
    global omega_list
    global acceleration_torque_list
    global RPM_list
    global rope_speed_list

    if last_rope_unwinded != 0:
        delta_r_u = rope_unwinded - last_rope_unwinded
        #print("Delta RU", delta_r_u)
        rope_speed = delta_r_u / 0.001
        rope_speed_list.append(rope_speed)
        #r = diameter/2
        RPM = rope_speed /(r*0.10472)
        RPM_list.append(RPM)
        #print("RPM", RPM, "Diameter", diameter)
        global last_omega
        if last_omega != 0:
            omega = rope_speed/(r)
            omega_list.append(omega)
            #print("Omega", omega)
            delta_omega = omega - last_omega
            alpha = delta_omega/0.001
            alpha_list.append(alpha)
            #print("Acceleration torque required at load", 0.008*alpha) #See the document in calculating inertia
            acceleration_torque_list.append(0.008*alpha)
            #print("Alpha", alpha)

        last_omega = rope_speed/(r)

    last_rope_unwinded = rope_unwinded


    return r

fw_mass = 1
fw_speed = 17.0

braking_time = 10.0/fw_speed

fw_momentum = fw_speed*fw_mass

braking_force_two_motors = fw_momentum/braking_time

braking_force_one_motor = braking_force_two_motors/2

fw_position = -17



#Now lets step through the process with small time steps

delta_t = 0.001
position_list = []
speed_list = []
torque_list = []
time_list = []
time = -1
pre_compute_spiral()
torque = 1
while(fw_speed > 0):

    fw_position += fw_speed*delta_t
    position_list.append(fw_position)
    if time >= 0:
        motor_radius = fw_x_to_motor_r(fw_position)
        #motor_radius = motor_diameter/2
        braking_force_one_motor = torque/motor_radius
        print("Braking force", braking_force_one_motor)
        #torque = braking_force_one_motor*motor_radius
        delta_momentum = (braking_force_one_motor*delta_t)*2
        fw_momentum -= delta_momentum
        fw_speed = fw_momentum/fw_mass
    else:
        alpha_list.append(0)
        omega_list.append(0)
        acceleration_torque_list.append(0)
        RPM_list.append(0)

        rope_speed_list.append(0)
    time += delta_t
    time_list.append(time)
    #As each motor is expected to change the momentum by this amount we multiply by two

    speed_list.append(fw_speed)
    torque_list.append(torque)



plt.style.use('fivethirtyeight')
fig = plt.figure()
plt.plot(time_list, speed_list)
fig.suptitle('fw speed', fontsize=20)
plt.xlabel('Time since hooking', fontsize=18)
plt.ylabel('Fw speed', fontsize=16)
plt.show()

fig = plt.figure()
plt.plot(time_list, position_list)
fig.suptitle('fw position', fontsize=20)
plt.xlabel('Time since hooking', fontsize=18)
plt.ylabel('Fw position', fontsize=16)
plt.show()

fig = plt.figure()
plt.plot(time_list, torque_list)
fig.suptitle('motor torque', fontsize=20)
plt.xlabel('Time since hooking', fontsize=18)
plt.ylabel('motor torque', fontsize=16)
plt.show()
del time_list[0]

fig = plt.figure()
plt.plot(time_list, RPM_list)
fig.suptitle('motor RPM', fontsize=20)
plt.xlabel('Time since hooking', fontsize=18)
plt.ylabel('motor RPM', fontsize=16)
plt.show()

fig = plt.figure()
plt.plot(time_list, rope_speed_list)
fig.suptitle('Rope speed', fontsize=20)
plt.xlabel('Time since hooking', fontsize=18)
plt.ylabel('Rope speed', fontsize=16)
plt.show()

del time_list[0]

fig = plt.figure()
plt.plot(time_list, omega_list)
fig.suptitle('motor angular speed', fontsize=20)
plt.xlabel('Time since hooking', fontsize=18)
plt.ylabel('motor angular speed', fontsize=16)
plt.show()




fig = plt.figure()
plt.plot(time_list, alpha_list)
fig.suptitle('motor angular acceleration', fontsize=20)
plt.xlabel('Time since hooking', fontsize=18)
plt.ylabel('motor angular acceleration', fontsize=16)
plt.show()

fig = plt.figure()
plt.plot(time_list, acceleration_torque_list)
fig.suptitle('motor acceleration torque', fontsize=20)
plt.xlabel('Time since hooking', fontsize=18)
plt.ylabel('motor acceleration torque', fontsize=16)
plt.show()

del RPM_list[0]
fig = plt.figure()
plt.plot(acceleration_torque_list, RPM_list)
fig.suptitle('Torque /RPM plot', fontsize=20)
plt.xlabel('Acceleration torque', fontsize=18)
plt.ylabel('RPM', fontsize=16)
plt.show()


def old_fw_x_to_motor_r(fw_x):
    #Lets define the distance between the two arms to 2 meter
    half_d = 1
    #Then the amount of rope unwinded can be found by pythagoras
    rope_unwinded =  math.sqrt(half_d**2+fw_x**2)
    max_rope_unwinded = 5.09901951359-1

    #As there is one meter rope to the point where the FW is hooked which is not supposed to ever get winded on the coil
    #this meter then needs to be subtracted
    rope_unwinded -= 1
    #The amount of rope left coiled up is then
    coiled_rope = max_rope_unwinded - rope_unwinded
    #print("Rope unwinded", rope_unwinded)
    #By approximating how many times the rope is winded it is approximated by circles stacked on top of each other with the distance equal to the ropes diameter



    rope_d = 0.005
    start_d = 0.03
    diameter = start_d
    approx = 0
    #print("Coiled rope", coiled_rope)
    while approx < coiled_rope:
        approx += math.pi*diameter
        diameter += 2*rope_d
    print("diameter", diameter)

    global last_rope_unwinded
    global alpha_list
    global omega_list
    global acceleration_torque_list
    global RPM_list
    if last_rope_unwinded != 0:
        delta_r_u = rope_unwinded - last_rope_unwinded
        #print("Delta RU", delta_r_u)
        rope_speed = delta_r_u / 0.001
        r = diameter/2
        RPM = rope_speed /(r*0.10472)
        RPM_list.append(RPM)
        #print("RPM", RPM, "Diameter", diameter)
        global last_omega
        if last_omega != 0:
            omega = rope_speed/(diameter*0.5)
            omega_list.append(omega)
            #print("Omega", omega)
            delta_omega = omega - last_omega
            alpha = delta_omega/0.001
            alpha_list.append(alpha)
            #print("Acceleration torque required at load", 0.008*alpha) #See the document in calculating inertia
            acceleration_torque_list.append(0.008*alpha)
            #print("Alpha", alpha)

        last_omega = rope_speed/(diameter*0.5)

    last_rope_unwinded = rope_unwinded


    return diameter