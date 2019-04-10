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
from plot_system import PlotSystem
import time

# defines
EPS = sys.float_info.min
TO_RAD = math.pi / 180
TO_DEG = 180 / math.pi

# We're using SI units
class ar_model(object):
    def __init__(self):
        self.interactive = False
        self.plane_verbose = False
        self.cr_verbose = False
        self.right_arm_verbose = False
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
        self.end_pos_y_l = []
        self.end_pos_x_l = []
        
        self.rope_speed = 0
        self.spring_init_vel = 0

        self.time = 0.0
        self.delta_t = 0.005

        self.neutral_rope_length = 2.5
        self.rope_len = self.neutral_rope_length 
        self.rope_k = 2000

        self.plane_mass = 0.7
        self.plane_pos = np.array([0, -1.1, 0]) #The plane starts one meter before the docking station before it is hooked
        self.plane_vel = np.array([0, 10, 0])
        self.plane_acc = np.array([0, 0, 0])
        self.plane_displacement = 0
        self.plane_k_e = 1/2 * self.plane_mass * self.plane_vel**2
        self.spring_k = 42
        self.spring_n = 2#26*0.1 # 26 springs has a max load of 308
        self.spring_rad = 0.05
        self.half_distane_poles = 1
        self.spring_l_ang_rope = math.pi/2
        self.spring_l_arm = 0.35  # this should be from the middle of the cylinder to the pulley
        self.spring_l_anchor = np.array([0, 0])
        self.spring_l_center_pos = np.array([0.05, 0])
        self.spring_l_end_point = np.array([0.05, 0.05])
        self.spring_l_eq = -math.pi/2
        self.spring_l_theta = -math.pi/2

        self.spring_r_ang_rope = math.pi/2
        self.spring_r_arm = 0.35  # meters
        self.spring_r_anchor = np.array([self.rope_len*2 + 0.1, 0])
        self.spring_r_center_pos = np.array([self.spring_r_anchor[0]-0.05, 0])
        self.spring_r_end_point = np.array([self.spring_r_anchor[0]-0.05, self.spring_r_anchor[1]+0.05])
        self.spring_r_eq = -math.pi/2
        self.spring_r_theta = -math.pi/2

        # self.spring_l_u_e = 1 / 2 * self.spring_k * self.spring_l_angle_to_rope ** 2
        # self.spring_r_u_e = 1 / 2 * self.spring_k * self.spring_r_angle_to_rope ** 2
        # self.max_spring_stretch = self.max_spring_stretch_method()
        self.max_spring_load = 12.900
        self.e_plane = 0
        self.e_spring = 0
        self.e_loss = 10  # percent. Might not be usaable


        #Both arm kinematics
        #self.arm_dampening = 0.9
        self.torsion_K = 470
        #Right arm kinematics
        self.eqi_angle_right = 0
        self.omega_right = 0
        self.theta_right = 0
        self.right_arm_position_x_l = []
        self.right_arm_position_y_l = []
        self.right_rope_anchor_list = []#self.right_rope_anchor_l = np.empty([3, 1], dtype=float)
        self.right_rope_anchor_y = []
        
        #Left arm kinematis
        self.eqi_angle_left = 0
        self.omega_left = 0
        self.theta_left = 0
        self.left_arm_position_x_l = []
        self.left_arm_position_y_l = []
        self.left_arm_theta_list = []

        #Left rope
        self.left_rope_theta_list = []

        #Left motor
        self.motor_l_theta = 0
        self.motor_l_omega = 0
        self.motor_l_alpha = 0
        self.motor_l_radius = 0.2 #The radius where the rope are coiled
        self.motor_force = 0
        self.motor_holding_torque = 2#0.19 #ish

        self.motor_phase = 1
        self.motor_theta_list = []
        self.motor_omega_list = []
        self.motor_alpha_list = []
        #103H7823-1740 https://docs-emea.rs-online.com/webdocs/141d/0900766b8141d50f.pdf
        self.motor_inertia = 0.000084
        self.motor_load_inertia = 0.00803026571
        self.motor_pull_out_torque = 1  #This is a worst case estimate
        self.motor_max_alpha = self.motor_pull_out_torque/(self.motor_load_inertia+self.motor_inertia) #t = I * alpha <=> alpha = t/i
        #Right motor
        #self.motor_r_thetha
        #self.motor_r_omega
        #self.motor_r_alpha
        #self.motor_r_radius #The radius where the rope are coiled

    def append_motor_stuff(self):
        self.motor_theta_list.append(self.motor_l_theta)
        self.motor_omega_list.append(self.motor_l_omega)
        self.motor_alpha_list.append(self.motor_l_alpha)

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

    # def old_method_for_calc_ag_to_string(self):
    # # here we calculate the angle of the connection between the spring and the rope
    # ab_l = abs(self.spring_l_center_pos - self.spring_l_anchor)
    # bc_l = abs(self.spring_l_end_point - self.spring_l_center_pos)
    # dot_l = np.dot(ab_l, bc_l)
    # dot_l.astype(np.double)
    # linalg_l = np.linalg.norm(ab_l) * np.linalg.norm(bc_l)
    # linalg_l.astype(np.double)
    # tmp_l = np.around(math.acos(dot_l/linalg_l), decimals=13)
    # self.spring_l_ang_rope = tmp_l
    #
    # ab_r = abs(self.spring_r_center_pos - self.spring_r_anchor)
    # bc_r = abs(self.spring_r_end_point - self.spring_r_center_pos)
    # dot_r = np.dot(ab_r, bc_r)
    # dot_r.astype(np.double)
    # linalg_r = np.linalg.norm(ab_r) * np.linalg.norm(bc_r)
    # linalg_r.astype(np.double)
    # tmp_r = np.around(math.acos(dot_r/linalg_r), decimals=13)
    # self.spring_r_ang_rope = tmp_r
    
    #Start med P controller -> PI -> PD -PID i den rækkefølge
    #hvis de ikke accelere hurtigt nok er det PI, hvis de gør det for hurtigt vil et P led bremste det hele.
    #Papers med lineære controllers til ulineære systemer
    #Prøv at tegne loops op og se hvad der skal til
    #Start med at antage at flyet har en hastighed vi kender"
    #Der er måske et lineært forhold mellem rebets vinkel og armens, i forhold til hvo rmange newton der bliver trukket med.
    def plot_plane(self):
        '''
        x = time_list
        y = pos_list
        # y = [item[1] for item in pos_list]
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
        '''
        print("Post_list[1]", self.plane_pos_l[1])
        plt.subplot(3, 1, 1)
        plt.title("Plane movement along the y-axis")
        plt.plot(self.time_l, [item[1] for item in self.plane_pos_l], 'r')
        plt.xlabel('Time', fontsize=18)
        plt.ylabel('Plane y pos', fontsize=16)

        plt.subplot(3, 1, 2)
        plt.plot(self.time_l, [item[1] for item in self.plane_vel_l], 'r')
        plt.xlabel('Time', fontsize=18)
        plt.ylabel('Plane y velocity', fontsize=16)

        plt.subplot(3, 1, 3)
  
        plt.plot([item[0] for item in self.plane_pos_l], [item[1] for item in self.plane_pos_l], 'r')
        plt.xlabel('plane x pos', fontsize=18)
        plt.ylabel('Plane y pos', fontsize=16)

        plt.show()
    
    def plot_thetas(self):
        #This methods plots the left thetas in function of time
        
        #We need to make the rope theta consitent, therefore if the angle is negative we need to adjust it
        counter = 0
        '''for theta in self.left_rope_theta_list:
            if theta < 0:
                new_theta = math.pi + (math.pi+theta)
                self.left_rope_theta_list[counter] = new_theta #Adjustment equation
            #Lets also get the value down to approx zero by subtracting pi
            #self.left_rope_theta_list[counter] -= math.pi
            counter += 1
        '''
        fig = plt.figure
        plt.plot(self.time_l[1:], self.left_rope_theta_list, label="Rope theta")
        plt.plot(self.time_l[1:], self.left_arm_theta_list, "r--", label="Arm theta")
        title = "The two thetas of the left side"
        plt.ylabel('Thetha [rad]')
        plt.xlabel("Time [seconds]")
        plt.legend()
        plt.show()
        #print(self.refined_time)

    def plot_right_arm(self):
        #print(self.right_arm_position_l)
          
        
        plt.subplot(2, 1, 1)
        plt.title("Different vector value plots")
        plt.plot(self.right_arm_position_x_l, self.right_arm_position_y_l, 'ro')
        plt.xlabel('X values of position vector', fontsize=18)
        plt.ylabel('Y Values of position vector', fontsize=16)
        #print("Right rope anchor", self.right_rope_anchor_list)
        #for item in self.right_rope_anchor_list:
        #    print("Item", item)
        plt.subplot(2, 1, 2)
        plt.plot([item[0] for item in self.right_rope_anchor_list], [item[1] for item in self.right_rope_anchor_list], 'r')
        plt.xlabel('rra_x', fontsize=18)
        plt.ylabel('rra_y', fontsize=16)
        '''
        plt.subplot(3, 1, 3)
  
        plt.plot(self.time_l, [item[1] for item in self.plane_acc_l], 'r')
        plt.xlabel('Time', fontsize=18)
        plt.ylabel('Plane y acceleration', fontsize=16)
        '''
        plt.show()


    def total_energy(self, kinetic_list, potential_list, time_list): #Used with linear spring
        x = time_list
        p = potential_list
        k = kinetic_list

        plt.subplot(2, 2, 1)
        plt.title('Energy in system')
        plt.plot(x, k, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Plane', fontsize=16)

        plt.subplot(2, 2, 2)
        plt.plot(x, p, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Spring', fontsize=16)

        s = [sum(s) for s in zip(p, k)]
        # print(len(s))

        plt.subplot(2, 2, 3)
        plt.plot(x, s, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Total', fontsize=16)

        plt.subplot(2, 2, 3)
        plt.plot(x, s, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Total', fontsize=16)

        
        plt.show()

    def gen_plot(self, plot_list, time_list, title=""): #General plotting
        plt.title(title)
        plt.plot(time_list, plot_list, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Total', fontsize=16)

        plt.show()

    def connect_rope(self, naturel_length, spring_k, hook_point, anchor_point):
        '''
        We need to calc the direction of the rope, which we do by using the start and endpoint
        :param naturel_length:
        :param spring_k:
        :param hook point: Point where the plane touches 
        :param end_anchor: Point where the rope is anchored at the docking station
        :return:
        '''

        #Two different forces for the rope
        stretch = hook_point - anchor_point
        if self.cr_verbose:
            print("Rope stretch", stretch)
        lhat = stretch / (stretch**2).sum()**0.5 #Normalized direction vector
        stretch = np.linalg.norm(stretch) - naturel_length
        #print("Rope stretch", stretch)
        force = -spring_k*stretch*lhat
        if self.cr_verbose:
            print("Rope force", force)
        if stretch < 0:
            return np.array([0, 0, 0])
        return force

    def calc_end_point_from_angle(self, end_point, center_of_circle, radius, plane_pos):
        '''
        Description Model of the rotating arm
        :end point = where the rope is attached.
        :Center of circle = center of torsion sprring (point where rotation happens)
        :radius = length of the catching arm
        :Plane pos = plane (x, y) coordinate

        Returning (end point of rotating arm, the angle from center to arm)
        '''
        theta = math.atan2(end_point[1] - center_of_circle[1],
                           end_point[0] - center_of_circle[0])
        if theta >= math.pi:
            theta = -math.pi
        # print("theta", theta)

        # print("old end pos", end_point)
        end_point[0] = (math.cos(theta)) * radius + center_of_circle[0]
        end_point[1] = (math.sin(theta)) * radius + center_of_circle[1]
        # print("new end pos l", end_point)

        return end_point, theta

    def calc_end_point_from_rope_force(self, spring_k, rope_ang, spring_rad, rope_force, spring_center):
        end_point = np.array(np.zeros(2))
        '''
        spring_k: The spring constant of the rope
        rope_ang: There is an axis from the end of the arm going parallel with the y axis, the angle is measured from this axis to the rope CCW
        spring_rad: is the angle of the arm (where the spring is fixed to)
        rope_force: The force the rope pulls with.
        Spring center: (x, y) position of where the spring is mounted

        '''
        # rope_force = np.linalg.norm(rope_force)
        sign_r_f = np.sign(rope_force)
        print("rope f", rope_force)
        rope_force = sum(rope_force**2)**0.5
        if sign_r_f[0] < 0:
            rope_force *= -1
        print("sign", sign_r_f)

        theta_spring = (spring_rad * rope_force * math.sin(rope_ang)) / (- spring_k)
        # print("math.sin(rope_ang)", math.sin(rope_ang))
        # print("rope_force", rope_force)
        # print("spring_rad", spring_rad)
        #
        # print("theta_spring", theta_spring * TO_DEG)

        if theta_spring >= math.pi:
            theta_spring = -math.pi

        end_point[0] = (math.cos(theta_spring)) * spring_rad + spring_center[0]
        end_point[1] = (math.sin(theta_spring)) * spring_rad + spring_center[1]

        return end_point, theta_spring

    def right_arm_kin(self, force_vector, position_vector):
        '''
        The kinematics of the RIGHT arm is calculated and stored in this method
        
        *****Not 100 % sure******
        The force along the y axis from the rope will rotate the arm fo/llowing
        alpha = F_spring_y / M_arm
        *****Not 100 % sure******
        
        *****From www.maplesoft.com/content/EngineeringFundamentals/4/mapledocument_30/Rotation MI and Torque.pdf*****
        torque_vector = 
        r_vector crossproduct F_vector = Inertia * acceleration_vector
        *****From www.maplesoft.com/content/EngineeringFundamentals/4/mapledocument_30/Rotation MI and Torque.pdf*****
        From this i think we can say that we want to solve for the acceleration_vector in order to calculate how the arm moves, thus
        (r_vector (crossproduct) F_vector) / Inertia_arm = Acceleration_vector
        I = M*r^2 lets use the formula for a point mass rotating a distance r from the axis, thus the mass of the arm and the distance to the COM should be used 
        I = 0.15*0.2^2 = 0.006
        We expect to get angular acceleration about the z axis, therefore we probably want to use 3D vectors

        *****Adding torsion spring*******
        Torque = -k*theta (Spring constant times the angle between the equilibrum point and current angle of the torsion spring)
        Torque = r (cross) F
        We can calculate the torque the rope applies on the arm, then we can calculate the torque the spring does, then find the net torque and then solve for acceleration
        
        '''
        inertia_arm = 0.006
        L = 0.4 #The length of the arm
        
        self.eqi_angle_right = math.pi #Equilibrim angle of the torsion spring this angle is 135 degrees from the x axis cw
        spring_torque = (self.eqi_angle_right-self.theta_right)*self.torsion_K
        lever_torque = np.cross(position_vector, force_vector)
        #print("RIGHT Lever torque", lever_torque[2])
        #print("RIGHT Spring Torque", spring_torque)
        total_torque = spring_torque + lever_torque[2]
        acceleration = total_torque/inertia_arm
        
        #force_vector_3D = np.array([force_vector[0], force_vector[1], 0])
        #print("Start theta", math.atan2(position_vector[1], position_vector[0]))
        #The position vector of the point where the force is applied relative to teh axis of rotation
        #alpha_vec = np.cross(position_vector, force_vector)/0.05#/0.006 WHEN there is no spring
        #alpha_vec[2] = acceleration
        #This acceleration moves the arm
        self.omega_right += acceleration*self.delta_t
        self.theta_right += self.omega_right*self.delta_t
        
        position_vector[0] = L*math.cos(self.theta_right)
        position_vector[1] = L*math.sin(self.theta_right)
        if self.right_arm_verbose:
            print("Omega right", self.omega_right)
            print("Theta right", self.theta_right)
            print("Position arm right", position_vector)
            #print("Acceleration of arm", alpha_vec)
        self.right_arm_position_x_l.append(position_vector[0])
        self.right_arm_position_y_l.append(position_vector[1])
        #M = 0.200 #0.2 kg seems fair for a wooden beam as an arm
        #This vector should then move the arm, so r have to be changed, but for starters, lets just try to print how the acceleration is when the arm is moved.
        return position_vector

    def left_arm_kin(self, force_vector, position_vector):
        '''
        The kinematics of the RIGHT arm is calculated and stored in this method
        
        *****Not 100 % sure******
        The force along the y axis from the rope will rotate the arm fo/llowing
        alpha = F_spring_y / M_arm
        *****Not 100 % sure******
        
        *****From www.maplesoft.com/content/EngineeringFundamentals/4/mapledocument_30/Rotation MI and Torque.pdf*****
        torque_vector = r_vector crossproduct F_vector = Inertia * acceleration_vector
        *****From www.maplesoft.com/content/EngineeringFundamentals/4/mapledocument_30/Rotation MI and Torque.pdf*****
        From this i think we can say that we want to solve for the acceleration_vector in order to calculate how the arm moves, thus
        (r_vector (crossproduct) F_vector) / Inertia_arm = Acceleration_vector
        I = M*r^2 lets use the formula for a point mass rotating a distance r from the axis, thus the mass of the arm and the distance to the COM should be used 
        I = 0.15*0.2^2 = 0.006
        We expect to get angular acceleration about the z axis, therefore we probably want to use 3D vectors

        '''
        inertia_arm = 0.006
        L = 0.4 #The length of the arm
        self.torsion_K = 3 #The spring constant of the torsion spring
        self.eqi_angle_left = 0 #Equilibrim angle of the torsion spring this angle is 45 degrees from the x axis cw
        spring_torque = (self.eqi_angle_left-self.theta_left)*self.torsion_K
        lever_torque = np.cross(position_vector, force_vector)
        total_torque = spring_torque + lever_torque[2]
        acceleration = total_torque/inertia_arm
        #print("LEFT Lever torque", lever_torque[2])
        #print("LEFT Spring Torque", spring_torque)
        #force_vector_3D = np.array([force_vector[0], force_vector[1], 0])
        #print("Start theta", math.atan2(position_vector[1], position_vector[0]))
        #The position vector of the point where the force is applied relative to teh axis of rotation
        #alpha_vec = np.cross(position_vector, force_vector)/0.05#/0.006 WHEN there is no spring
        #alpha_vec = np.cross(position_vector, force_vector)/0.05#/0.006
        #alpha_vec[2] = acceleration
        #This acceleration moves the arm
        self.omega_left += acceleration*self.delta_t
        self.theta_left += self.omega_left*self.delta_t
        position_vector[0] = L*math.cos(self.theta_left)
        position_vector[1] = L*math.sin(self.theta_left)
        if self.right_arm_verbose:
            print("Omega left", self.omega_left)
            print("Theta left", self.theta_left)
            print("Position arm left", position_vector)
            #print("Acceleration of arm", alpha_vec)
        self.left_arm_position_x_l.append(position_vector[0])
        self.left_arm_position_y_l.append(position_vector[1])
        #M = 0.200 #0.2 kg seems fair for a wooden beam as an arm
        #This vector should then move the arm, so r have to be changed, but for starters, lets just try to print how the acceleration is when the arm is moved.
        return position_vector
    
    def plot_motor(self):
        plt.style.use('fivethirtyeight')
        fig = plt.figure()
        #plt.subplot(3,1,1)
        #plt.plot(self.time_l, self.motor_theta_list)
        #plt.title('Motor angular position - speed - acceleration', fontsize=20)
        #plt.xlabel('Time [s]', fontsize=18)
        #plt.ylabel('Theta [rad]', fontsize=16)
        #sda
        #fig = plt.figure()
        #plt.subplot(3,1,2)
        rounds_per_min_list = []
        for item in self.motor_omega_list:
            rounds_per_min_list.append(item*9.55)
        plt.plot(self.time_l, rounds_per_min_list)
        #plt.title('Motor angular speed', fontsize=20)
        plt.xlabel('Time [s]', fontsize=18)
        plt.ylabel('Motor speed [RPM]', fontsize=16)
        
        #fig = plt.figure()
        #plt.subplot(3,1,3)
        #plt.plot(self.time_l, self.motor_alpha_list)
        #plt.title('Motor theta', fontsize=20)
        #plt.xlabel('Time [s]', fontsize=18)
        #plt.ylabel('Alpha [rad/s^2]', fontsize=16)
        #plt.savefig("Motor_plots.png")
        plt.show()


    def left_motor(self, plane_hooked):
        accelerate = False
        de_accelerate = False
        #print("Motor acceleration max", self.motor_max_alpha)
        if self.motor_phase == 1:
            #new_len = (self.half_distane_poles**2 + self.plane_pos[1]**2)**0.5

            #camparable_speed = old_len - new_len /
            if plane_hooked:
                if self.rope_speed < math.sqrt(self.half_distane_poles**2+self.plane_vel[1]**2): #d/dy(sqrt(k^2+y^2)) #Prøv at differientiere positionen på hypotenusen.
                    accelerate = True
                elif self.rope_speed > math.sqrt(self.half_distane_poles**2+self.plane_vel[1]**2):
                    de_accelerate = True
                '''
                if self.rope_speed < self.plane_pos[1]/math.sqrt(self.half_distane_poles**2+self.plane_pos[1]**2): #d/dy(sqrt(k^2+y^2)) #Prøv at differientiere positionen på hypotenusen.
                    accelerate = True
                elif self.rope_speed > self.plane_pos[1]/math.sqrt(self.half_distane_poles**2+self.plane_pos[1]**2):
                    de_accelerate = True
                '''
                if accelerate: #The motor will accelerate forward
                    self.motor_l_alpha = self.motor_max_alpha
                    self.motor_l_omega += self.motor_l_alpha*self.delta_t
                    self.motor_l_theta += self.motor_l_omega*self.delta_t
                    self.rope_len = self.neutral_rope_length + self.motor_l_theta * self.motor_l_radius
                    print("Accelerating", self.rope_len)
                elif de_accelerate:
                    self.motor_l_alpha = -self.motor_max_alpha
                    self.motor_l_omega += self.motor_l_alpha*self.delta_t
                    self.motor_l_theta += self.motor_l_omega*self.delta_t
                    self.rope_len = self.neutral_rope_length + self.motor_l_theta * self.motor_l_radius
                    print("De accelerating")
                
                #old_len =new_len
            
            #print("Motor_l_theta", self.motor_l_theta)
            if self.rope_speed - math.sqrt(self.half_distane_poles**2+self.plane_vel[1]**2) > -0.1 and self.rope_speed - math.sqrt(self.half_distane_poles**2+self.plane_vel[1]**2) < 0.1:  # 1 - 1.1 = -0.1 : 1.1 - 1 = 0.1
                print("Target speed acquired")
                self.motor_phase = 2
        if self.motor_phase == 2:
            #In this phase the motor should de accelerate the plane and therefore add force to the two ropes        
            self.motor_l_alpha = -self.motor_max_alpha
            self.motor_l_omega += self.motor_l_alpha*self.delta_t        
            self.motor_l_theta += self.motor_l_omega*self.delta_t
            self.rope_len = self.neutral_rope_length + self.motor_l_theta * self.motor_l_radius
            #motor_torque = motor_radius * motor_force
            motor_torque = self.motor_holding_torque
            self.motor_force = motor_torque/self.motor_l_radius
            
            
            
            #self.plane_vel = self.plane_vel + braking_de_acc * self.delta_t
            #self.plane_pos = self.plane_pos + self.plane_vel * self.delta_t
            #if self.rope_len - self.neutral_rope_length > -0.1 and self.rope_len - self.neutral_rope_length < 0.1:
            #    self.motor_phase = 3
            
            if self.motor_l_omega < 0:
                self.motor_l_omega = 0
                self.motor_phase = 4
            
            
            #if self.plane_vel[1] - 0.0 > -0.3 and self.plane_vel[1] - 0.0  < 0.3:
            #    self.motor_phase = 3
        #Hvad skal der til før vi kan vælge en fornuftig motor?
            #Er det for lidt med constant pull out torque
        #Skal vi have kode der kan estimere hastigheden ud fra yaw - inden vi går videre med hardware til systemet?
            #Check
        #
        if self.motor_phase == 3:
            #This phase the motors tries to go to a home position, where the rope has the start length
            self.motor_force 
            if self.rope_len > self.neutral_rope_length:
                de_accelerate = True
            else:
                accelerate = True

            if accelerate: #The motor will accelerate forward
                self.motor_l_alpha = self.motor_max_alpha
                self.motor_l_omega += self.motor_l_alpha*self.delta_t
                self.motor_l_theta += self.motor_l_omega*self.delta_t
                self.rope_len = self.neutral_rope_length + self.motor_l_theta * self.motor_l_radius
                #print("Accelerating", self.rope_len)
            elif de_accelerate:
                self.motor_l_alpha = -self.motor_max_alpha
                self.motor_l_omega += self.motor_l_alpha*self.delta_t
                self.motor_l_theta += self.motor_l_omega*self.delta_t
                self.rope_len = self.neutral_rope_length + self.motor_l_theta * self.motor_l_radius
                    #print("De accelerating")

            if self.rope_len - self.neutral_rope_length > -0.05 and self.rope_len - self.neutral_rope_length < 0.05:
                self.motor_phase = 3

        if self.motor_phase == 4:
            self.motor_force = 0
            self.motor_l_alpha = 0
            self.motor_l_omega = 0   
        print("Motor state", self.motor_phase)
        
        #PID loop for the motor:
        '''run(speed):
            error = speed - self.motor_l_omega 

            P = 0.5 * error
            delta_error = last_error - error
            D = 0.001 * delta_error/self.delta_t
         '''   
            


        #print("Motor force", self.motor_force)

        #print("Motor phase", self.motor_phase, self.plane_vel)
        #The motor goes through different phases
            #PHASE 1
            #The motor accelerates so that it unravels the rope at the same speed as the plane is travelling
            #plane_speed =  EQ1 dx/dt ~~ delta_y / delta_t 
                #rope_length^2 = (half distance poles)^2 + plane y ^2    ###EXPERIMENT maybe speed can be used instead of distances
                #plane y = sqrt(rope_length² - half_distane_poles^2) EQ 1
                #Hvis vi differentiere ovenstående lignings venstreside med y og højre side med rope_length så har vi hvordan rope lengths hastighed hænger sammen med flyets y hastighed
                #rope_length speed = motor_omega*motor_radius #Rope length can be estimated by either encoders or steppers setpoint angle
                #
                #rope_length^2 = (half distance poles)^2 + plane y ^2    
                #Vi har en fixed wing speed, hvilken hastighed skal motoren have for at følge flyet
            #PHASE 2
            #The motor uses its torque to de accelerate the load to a halt

    

    def main(self):
        damping = 0.001

        # Append twice to get descrete acceleration
        self.plane_k_e_l.append(self.plane_k_e)
        # self.spring_l_u_e_l.append(self.spring_l_u_e)
        # self.spring_r_u_e_l.append(self.spring_r_u_e)
        self.plane_pos_l.append(self.plane_pos)
        self.plane_vel_l.append(self.plane_vel)
        self.plane_acc_l.append(self.plane_acc)
        self.total_force_l.append(0)
        self.all_purpose_l.append(0)
        self.end_pos_x_l.append(self.spring_l_end_point[0])
        self.end_pos_y_l.append(self.spring_l_end_point[1])
        self.time_l.append(self.time)
        self.append_motor_stuff()
        damping = 0.3

       
        cnt = 0
        plane_hooked = False
        f_spring_system = np.array([0.0, 0.0, 0.0])
        start_position_vector_right_arm = np.array([-0.4, 0.0, 0.0]) #NEED a REFERENCE vector for its position in the coordinate system
        vras_x = -0.4 #VectorRightArmStart
        vras_y = 0.0
        vlas_x = 0.4
        vlas_y = 0.0
        start_position_vector_left_arm = np.array(([vlas_x, vlas_y, 0.0]))
        position_vector_right_arm = start_position_vector_right_arm
        position_vector_left_arm = start_position_vector_left_arm
        hook_point = np.array([0.0, -0.0, 0.0]) #As the plane are one meter below the hook point (seen in the y direction)
        start_left_rope_anchor = np.array([-self.neutral_rope_length, -0.0, 0.0]) 
        start_right_rope_anchor = np.array([self.neutral_rope_length, -0.0, 0.0]) 
        self.theta_right = math.atan2(start_position_vector_right_arm[1], start_position_vector_right_arm[0])
        self.theta_left = math.atan2(start_position_vector_left_arm[1], start_position_vector_left_arm[0])
        #Interactive plot
        if self.interactive:
            plot = PlotSystem(-7.5, 7.5, -7.5, 7.5)
        while self.time < 3:
            '''
            Description: This version introduces the two arms, but with no springs attached
            '''
            plane_drag_force = 0.2*self.plane_vel[1]**2
            if self.plane_vel[1] > 0: #If positive velocity we need a negative drag force
                plane_drag_force = -plane_drag_force
            #print("Plane vel, drag force", self.plane_vel[1], plane_drag_force)
            rope_before = self.rope_len
            
            #MOTOR call
            #self.left_motor(plane_hooked)
            
            
            rope_after = self.rope_len
            self.rope_speed = (rope_after - rope_before)/self.delta_t
            #right_rope_anchor = np.array([start_right_rope_anchor[0]-start_position_vector_right_arm[0]+position_vector_right_arm[0],\
            #start_right_rope_anchor[1]-start_position_vector_right_arm[1]+position_vector_right_arm[1],\
            #start_right_rope_anchor[2]-start_position_vector_right_arm[2]+position_vector_right_arm[2]])
            rra_x = start_right_rope_anchor[0]-vras_x+ position_vector_right_arm[0]
            rra_y = start_right_rope_anchor[1]-vras_y+ position_vector_right_arm[1]
            right_rope_anchor = np.array([rra_x, rra_y, 0.0])
            self.right_rope_anchor_list.append(right_rope_anchor)#np.append(self.right_rope_anchor_l, right_rope_anchor)
            
            lra_x = start_left_rope_anchor[0]-vlas_x+ position_vector_left_arm[0]
            lra_y = start_left_rope_anchor[1]-vlas_y+ position_vector_left_arm[1]
            left_rope_anchor = np.array([lra_x, lra_y, 0.0])
            #print("Rope anchors r / l", right_rope_anchor, left_rope_anchor)
            #self.right_rope_anchor_x.append(right_rope_anchor[0])
            #self.right_rope_anchor_y.append(right_rope_anchor[1])
            #We have two ropes each going from their left anchor to the hooking point
            #These lines dont need to be in the while loop
            if not plane_hooked:
                start_sum_hook_points = np.add(start_right_rope_anchor, start_left_rope_anchor)
                position_arm_diff = np.subtract(position_vector_left_arm, position_vector_right_arm)
                hook_y = position_vector_left_arm[1]
                #print("hok y", hook_y)
                hook_point = np.array([0.0, 0, 0.0])
                #print("position_arm_diff", position_arm_diff)
                #print("Start_sum_hook_point", start_sum_hook_points)
                #print("hook point", hook_point)
        
            if plane_hooked:
                hook_point = self.plane_pos
                #self.right_arm_position_l.append(position_vector_right_arm)
            
            f_rope_l = model.connect_rope(self.rope_len, self.rope_k, hook_point, left_rope_anchor)
            f_rope_r = model.connect_rope(self.rope_len, self.rope_k, hook_point, right_rope_anchor)
            self.left_arm_theta_list.append(self.theta_left)
            if not plane_hooked:
                self.left_rope_theta_list.append(math.atan2(0, 1))
            else:
                self.left_rope_theta_list.append(math.atan2(self.plane_pos[1]-lra_y, self.plane_pos[0]-lra_x))
            f_spring_system = f_rope_l + f_rope_r
            #f_damping = -f_spring_system*0.1
            #f_spring_system += f_damping
            position_vector_right_arm = self.right_arm_kin(np.negative(f_rope_r), position_vector_right_arm)
            position_vector_left_arm = self.left_arm_kin(np.negative(f_rope_l), position_vector_left_arm)
            #print("SPring force", f_spring_system)
            if self.cr_verbose:
                print("Force from the two ropes", f_spring_system)
                print("Force from left rope", f_rope_l)
                print("Force from right rope", f_rope_r)
            #f_spring_system = f_rope_l + f_rope_r
            #The dynamics of the plane
            if plane_hooked:
                f_spring_system[1] += plane_drag_force
                #y_force = force_on_plane/self.plane_mass #Maybe move this to main
                #braking_de_acc = np.array([0.0, y_acc, 0.0])

                if self.motor_phase == 2:
                    force_on_plane = 2*(self.motor_force*math.sin(self.theta_left)) # We have two motors therefore multiplying by two
                    f_spring_system[1] += force_on_plane
                    
                self.plane_acc = f_spring_system / self.plane_mass
                    
            if self.plane_verbose:
                print("Plane acc", self.plane_acc)
            self.plane_vel = self.plane_vel + self.plane_acc * self.delta_t
            
            self.plane_pos = self.plane_pos + self.plane_vel * self.delta_t
            
            #Check if the plane is hooked to the rope yet
            if not plane_hooked and self.plane_pos[1] > hook_point[1]:
                plane_hooked = True
            
            if self.plane_verbose:
                print("plane pos", self.plane_pos)
                print("Is plane hooked?", plane_hooked)            
            
            #Appending to lists with intresting parameters
            self.plane_pos_l.append(self.plane_pos)
            self.plane_vel_l.append(self.plane_vel)
            self.plane_acc_l.append(self.plane_acc)
            self.total_force_l.append(f_spring_system)
          #  self.all_purpose_l.append(np.linalg.norm(f_rope_l))
            self.end_pos_x_l.append(self.spring_l_end_point[0])
            self.end_pos_y_l.append(self.spring_l_end_point[1])
            self.time_l.append(self.time)
            self.append_motor_stuff()

            
            
            #Moving thrugh time with small steps
            self.time += self.delta_t
            if self.interactive:
                plane_plot = [self.plane_pos[0], self.plane_pos[1], self.plane_vel[0], self.plane_vel[1]]
                left_rope = [[left_rope_anchor[0], left_rope_anchor[1]], [self.plane_pos[0], hook_point[1]]]
                #print("Left rope", left_rope)
                right_rope = [[right_rope_anchor[0], right_rope_anchor[1]], [self.plane_pos[0], hook_point[1]]]
                #print("Righ rope", right_rope)
                left_arm = [[start_left_rope_anchor[0]-vlas_x, start_left_rope_anchor[1]-vlas_y], [left_rope_anchor[0], left_rope_anchor[1]]]
                right_arm = [[start_right_rope_anchor[0]-vras_x, start_right_rope_anchor[1]-vras_y], [right_rope_anchor[0], right_rope_anchor[1]]]
                extra_vec = [3, 0, self.rope_len, 0]#extra_vec = [right_rope_anchor[0], right_rope_anchor[1], f_rope_r[0], f_rope_r[1]]
                plot.update_plot(plane_plot, [0, 0], [0, 0], left_rope, right_rope, left_arm, right_arm, extra_vec) #Plane[ x, y, x vel, y vel], [0, 0], [0, 0], left_rope[armx, army, planex, planey], 
                #right_rope[armx, army, planex, planey], left_arm[centerx, centery, endpointx, endpointy], right_arm[centerx, centery, endpointx, endpointy]
                time.sleep(0.001)

        
            
        self.plot_thetas()
        #self.plot_motor()
        #self.plot_right_arm()
        #self.plot_plane()

        '''
         plane_hooked = False
        f_spring_system = np.array([0, 0])
        while self.time < 3:
            
            Description: Simplified version, where the plane hooks at the middle of the rope,
            Which is fastened at equal fixed distance to the right and left
            
            
            #We have two ropes each going from their left anchor to the hooking point
            #These lines dont need to be in the while loop
            hook_point = np.array([0, 0]) #As the plane are one meter below the hook point (seen in the y direction)
            left_rope_anchor = np.array([-1, 0]) 
            right_rope_anchor = np.array([1, 0]) 
            
            if not plane_hooked:
                f_rope_l = model.connect_rope(self.rope_len, self.rope_k, hook_point, left_rope_anchor)
                f_rope_r = model.connect_rope(self.rope_len, self.rope_k, hook_point, right_rope_anchor)
                f_spring_system = f_rope_l + f_rope_r
                if self.cr_verbose:
                    print("Force from the two ropes", f_spring_system)
            if plane_hooked:
                f_rope_l = model.connect_rope(self.rope_len, self.rope_k, self.plane_pos, left_rope_anchor)
                f_rope_r = model.connect_rope(self.rope_len, self.rope_k, self.plane_pos, right_rope_anchor)
                f_spring_system = f_rope_l + f_rope_r
                if self.cr_verbose:
                    print("Force from the two ropes", f_spring_system)
            #f_spring_system = f_rope_l + f_rope_r
            #The dynamics of the plane
            self.plane_acc = f_spring_system / self.plane_mass
            if self.plane_verbose:
                print("Plane acc", self.plane_acc)
            self.plane_vel = self.plane_vel + self.plane_acc * self.delta_t
            
            self.plane_pos = self.plane_pos + self.plane_vel * self.delta_t
            
            #Check if the plane is hooked to the rope yet
            if not plane_hooked and self.plane_pos[1] > 0:
                plane_hooked = True
            
            if self.plane_verbose:
                print("plane pos", self.plane_pos)
                print("Is plane hooked?", plane_hooked)            
            
            #Appending to lists with intresting parameters
            self.plane_pos_l.append(self.plane_pos)
            self.plane_vel_l.append(self.plane_vel)
            self.plane_acc_l.append(self.plane_acc)
            self.total_force_l.append(f_spring_system)
          #  self.all_purpose_l.append(np.linalg.norm(f_rope_l))
            self.end_pos_x_l.append(self.spring_l_end_point[0])
            self.end_pos_y_l.append(self.spring_l_end_point[1])
            self.time_l.append(self.time)

            
            
            #Moving thrugh time with small steps
            self.time += self.delta_t

            
        self.plot_plane()
        '''

        cnt = 0
        '''
        while self.time < 4:
            # First we have an acceleration from prev time-step then a new vel and finally a new pos,
            # lastly increment the time

            # connect the rope from the hook point to the springs end-points
            hook_point = self.plane_pos
            f_rope_l = model.connect_rope(self.rope_len, self.rope_k, hook_point, self.spring_l_end_point)
            f_rope_r = model.connect_rope(self.rope_len, self.rope_k, hook_point, self.spring_r_end_point)
            # print("rope_force", f_rope_l, f_rope_r)
            # print("end_points", self.spring_l_end_point, self.spring_r_end_point)

            f_damping_l = -self.plane_vel*damping
            f_damping_r = -self.plane_vel*damping

            # tau_l_spring = -self.spring_k * (self.spring_l_ang_rope - self.spring_l_eq)  # The torsion spring contrib
            # tau_r_spring = -self.spring_k * (self.spring_r_ang_rope - self.spring_r_eq)  # The torsion spring contrib
            tau_l_spring = -self.spring_k * (self.spring_l_theta - self.spring_l_eq)  # The torsion spring contrib
            tau_r_spring = -self.spring_k * (self.spring_r_theta - self.spring_r_eq)  # The torsion spring contrib
            print("spring_l_theta, spring_l_ang_rope", self.spring_l_theta, self.spring_l_ang_rope)
            print("spring_r_theta, spring_r_ang_rope", self.spring_r_theta, self.spring_r_ang_rope)
            # print("tau_l_spring vs r", tau_l_spring, tau_r_spring)

            f_l_spring = tau_l_spring/(self.spring_l_arm * self.spring_l_ang_rope)
            f_r_spring = tau_r_spring/(self.spring_r_arm * self.spring_r_ang_rope)

            f_spring_system = f_rope_l + f_rope_r + \
                              f_damping_l + f_damping_r + \
                              f_l_spring + f_r_spring

            # print("f_spring_system", f_l_spring + f_r_spring)

            # print("acc", self.plane_acc)
            # print("vel", self.plane_vel)
            self.plane_acc = f_spring_system / self.plane_mass
            self.plane_vel = self.plane_vel + self.plane_acc * self.delta_t
            self.plane_pos = self.plane_pos + self.plane_vel * self.delta_t
            print("plane pos", self.plane_pos)

            # print("spring_l_end_point", self.spring_l_end_point)
            # print("spring_r_end_point", self.spring_r_end_point)

            # the angle the rope makes from for each side
            theta_rope_l = math.atan2(self.plane_pos[1] - self.spring_l_end_point[1],
                                      self.spring_l_end_point[0] - self.plane_pos[0])
            theta_rope_r = math.atan2(self.plane_pos[1] - self.spring_r_end_point[1],
                                      self.spring_r_end_point[0] - self.plane_pos[0])
            # print("theta rope_l", theta_rope_l * TO_DEG)
            # print("theta rope_r", theta_rope_r * TO_DEG)
            #
            # print("spring_l_theta", self.spring_l_theta * TO_DEG)
            # print("spring_r_theta", self.spring_r_theta * TO_DEG)

            # # calc the angle between the spring and the rope
            self.spring_l_ang_rope = theta_rope_l - self.spring_l_theta #math.pi / 2 - self.spring_l_theta + theta_rope_l + math.pi / 2
            self.spring_r_ang_rope = theta_rope_r - self.spring_r_theta #math.pi / 2 - self.spring_r_theta + theta_rope_r + math.pi / 2

            # print("spring_rope_ang", self.spring_l_ang_rope* TO_DEG, self.spring_r_ang_rope * TO_DEG)

            self.spring_l_end_point, self.spring_l_theta = self.calc_end_point_from_rope_force(self.spring_k,
                                                                                               self.spring_l_ang_rope,
                                                                                               self.spring_rad,
                                                                                               f_rope_l,
                                                                                               self.spring_l_center_pos)
            self.spring_r_end_point, self.spring_r_theta = self.calc_end_point_from_rope_force(self.spring_k,
                                                                                               self.spring_r_ang_rope,
                                                                                               self.spring_rad,
                                                                                               f_rope_r,
                                                                                               self.spring_r_center_pos)
            # print("l_end_point", self.spring_l_end_point)
            # print("r_end_point", self.spring_r_end_point)
            # print("spring_l_ang", self.spring_l_ang_rope)
            # print("spring_r_ang", self.spring_r_ang_rope)
            # print("angle_l_spring", self.spring_l_theta)
            # print("angle_r_spring", self.spring_r_theta)

            self.time += self.delta_t

            # self.plane_k_e_l.append(self.plane_k_e)
            # self.spring_u_e_l.append(self.spring_u_e)
            self.plane_pos_l.append(self.plane_pos)
            self.plane_vel_l.append(self.plane_vel)
            self.plane_acc_l.append(self.plane_acc)
            self.total_force_l.append(f_spring_system)
            self.all_purpose_l.append(np.linalg.norm(f_rope_l))
            self.end_pos_x_l.append(self.spring_l_end_point[0])
            self.end_pos_y_l.append(self.spring_l_end_point[1])

            self.time_l.append(self.time)

            cnt += 1
            print("")

        # y = [item[1] for item in pos_list]

        self.pos_to_vel_acc(self.time_l, [item[1] for item in self.plane_pos_l], "Two springs y dir") #PLotting
        '''
        # self.pos_to_vel_acc(self.time_l, [item[0] for item in self.plane_pos_l] , "Two springs x dir")
        # self.pos_to_vel_acc(self.time_l, self.all_purpose_l, "spring stretch disregard two last plots")
        # self.total_energy(self.plane_k_e_l, self.spring_u_e_l, self.time_l) # k_u is not correct
        # self.spring_forcediagram()
        # self.gen_plot(self.end_pos_x_l, self.time_l, "end pos left")
        # self.gen_plot(self.end_pos_y_l, self.time_l, "end pos left")
        # self.gen_plot(self.all_purpose_l, self.time_l, "rope tension")


if __name__ == "__main__":
    model = ar_model()
    model.main() # spring_forcediagram()

    # center = np.array([4, 3])
    # end_pos = np.array([4, 0])
    #
    # # ab_l = center - left_anchor
    # # bc_l = end_pos - center
    # # theta = math.acos(np.dot(ab_l, bc_l) / (np.linalg.norm(ab_l) * np.linalg.norm(bc_l)))
    #
    # theta = math.atan2(end_pos[1] - center[1], end_pos[0] - center[0])
    #
    # print(theta)
    #
    # rad = 3
    # plane_diff = math.pi/2
    #
    # for i in range(18):
    #     print("theta", theta)
    #
    #     print("old end pos", end_pos)
    #     end_pos[0] = (math.cos(theta)) * rad + center[0]
    #     end_pos[1] = (math.sin(theta)) * rad + center[1]
    #     print("new end pos", end_pos)
    #
    #     if theta >= math.pi:
    #         theta = -math.pi
    #     theta += math.pi/2
    #
    #     print("")

    #
    #     new_x_len = math.sqrt(rope_len**2 - plane_diff**2)
    #     print("new_x_len", new_x_len)
    #     new_x_pos = old_pos[0] - new_x_len - old_x
    #
    #     new_y_pos = center[1] - math.sqrt(rad**2 - (new_x_pos - center[0])**2)
    #
    #     # new_l_x_length = (self.rope_len ** 2 - (self.plane_pos[1] - self.plane_pos_l[cnt][1]) ** 2) ** 0.5
    #     # self.spring_l_end_point[0] = self.spring_l_end_point[0] - new_l_x_length + self.spring_l_center_pos[0]
    #     # new_l_x_diff = (self.spring_l_end_point[0] - self.spring_l_center_pos[0])
    #     # self.spring_l_end_point[1] = self.spring_l_center_pos[1] - (self.spring_l_center_pos[0] ** 2 - new_l_x_diff ** 2) ** 0.5
    #
    #     print("new x", new_x_pos)
    #     print("new y", new_y_pos)
    #
    #     radius = ((old_pos[0] - center[0])**2 + (old_pos[1] - center[1])**2)**0.5
    #     print("orig rad", radius)
    #
    #     radius = ((new_x_pos - center[0])**2 + (new_y_pos - center[1])**2)**0.5
    #     print("new rad", radius)


    # new_l_x_length = (self.rope_len ** 2 - (self.plane_pos[1] - self.plane_pos_l[cnt][1]) ** 2) ** 0.5
    # self.spring_l_end_point[0] = self.rope_len - new_l_x_length + self.spring_l_center_pos[0]
    # new_l_x_diff = (self.spring_l_end_point[0] - self.spring_l_center_pos[0])
    # self.spring_l_end_point[1] = (self.spring_l_center_pos[0] ** 2 - new_l_x_diff ** 2) ** 0.5



