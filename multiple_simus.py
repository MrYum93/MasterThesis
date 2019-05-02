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
        #mult sims control
        self.no_runs_init = 10
        self.run_no_init = 0
        self.change_const_init = (math.pi*0.5)/9 #Amount to change
        self.parameter_change_id = "eqi_change"
        self.time_to_run = 0.4
        self.time_to_sleep = 0.001


        #Multi sim lists of lists
        self.time_lists = []
        self.est_speed_lists = []
        self.est_t_l = []
        self.theta_err_ll = []
        self.fw_a_ll = []
        # lists
        self.est_pos_list = []
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
        self.delta_t = 0.0005


        self.eqi_change = 0 #(math.pi/2) #<-oriented 90 degrees after 90 runs
        self.eqi_angle_left = - self.eqi_change
        self.eqi_angle_right = math.pi + self.eqi_change
        self.plane_hooked = False
        self.half_distane_poles = 2.5
        self.arm_length = 0.4
        self.neutral_rope_length = self.half_distane_poles-abs(self.arm_length*math.cos(self.eqi_angle_right))
        self.distance_arms = self.neutral_rope_length *2
         #Both arms
        
        self.rope_len = self.neutral_rope_length
        self.rope_k = 29.16

        self.plane_mass = 0.7
        self.plane_pos = np.array([0, -0.1, 0]) #The plane starts one meter before the docking station before it is hooked
        self.plane_vel = np.array([0, 17, 0])
        self.plane_acc = np.array([0, 0, 0])
        self.plane_displacement = 0
        self.plane_k_e = 1/2 * self.plane_mass * self.plane_vel**2
        self.spring_k = 42
        self.spring_n = 2#26*0.1 # 26 springs has a max load of 308
        self.spring_rad = 0.05
        
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


        #From init model
        self.plane_hooked = False
        self.placement_v_r = np.array([self.half_distane_poles, -math.sin(self.eqi_angle_left)*self.arm_length , 0])
        self.placement_v_l = np.array([-self.half_distane_poles, -math.sin(self.eqi_angle_left)*self.arm_length , 0])
        
        self.f_spring_system = np.array([0.0, 0.0, 0.0])
        self.start_position_vector_right_arm = np.array([math.cos(self.eqi_angle_right)*self.arm_length, math.sin(self.eqi_angle_right)*self.arm_length, 0.0])
        #self.vras_x = self.start_position_vector_right_arm[0]
        #self.vras_y = self.start_position_vector_right_arm[1]               #ADJUSTING ARM LENGTH
        self.start_position_vector_left_arm = np.array(([math.cos(self.eqi_angle_left)*self.arm_length, math.sin(self.eqi_angle_left)*self.arm_length, 0.0]))
        #self.vlas_x = self.arm_length
        #self.vlas_y = 0.0
        self.position_vector_right_arm = self.start_position_vector_right_arm
        self.position_vector_left_arm = self.start_position_vector_left_arm
        self.hook_point = np.array([0.0, -0.0, 0.0]) #As the plane are one meter below the hook point (seen in the y direction)
        self.start_left_rope_anchor =  np.add(self.placement_v_l, self.start_position_vector_left_arm) 
        self.start_right_rope_anchor = np.add(self.placement_v_r, self.start_position_vector_right_arm) 

        self.f_spring_system = np.array([0.0, 0.0, 0.0])
        self.start_position_vector_right_arm = np.array([math.cos(self.eqi_angle_right)*self.arm_length, math.sin(self.eqi_angle_right)*self.arm_length, 0.0])
        #self.vras_x = -self.arm_length
        #self.vras_y = 0               #ADJUSTING ARM LENGTH
        self.start_position_vector_left_arm = np.array(([math.cos(self.eqi_angle_left)*self.arm_length, math.sin(self.eqi_angle_left)*self.arm_length, 0.0]))
        #self.vlas_x = self.arm_length
        #self.vlas_y = 0.0
        self.position_vector_right_arm = self.start_position_vector_right_arm
        self.position_vector_left_arm = self.start_position_vector_left_arm
        self.hook_point = np.array([0.0, -0.0, 0.0]) #As the plane are one meter below the hook point (seen in the y direction)
        self.start_left_rope_anchor = np.array([-(self.neutral_rope_length+self.arm_length), -0.0, 0.0]) #TODO
        self.start_right_rope_anchor = np.array([(self.neutral_rope_length+self.arm_length), -0.0, 0.0]) 
        #Both arm kinematics
        #self.arm_dampening = 0.9
        self.torsion_K = 1 #3


        #Right arm kinematics
        self.omega_right = 0
        self.theta_right = self.eqi_angle_right
        self.right_arm_position_x_l = []
        self.right_arm_position_y_l = []
        self.right_rope_anchor_list = []#self.right_rope_anchor_l = np.empty([3, 1], dtype=float)
        self.right_rope_anchor_y = []


        #Left arm kinematis
        
        self.omega_left = 0
        self.theta_left = self.eqi_angle_left
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

    def vector_init(self):
        self.eqi_angle_left = 0 - self.eqi_change
        self.eqi_angle_right = math.pi + self.eqi_change
        self.plane_hooked = False
        self.placement_v_r = np.array([self.half_distane_poles, -math.sin(self.eqi_angle_left)*self.arm_length , 0])
        self.placement_v_l = np.array([-self.half_distane_poles, -math.sin(self.eqi_angle_left)*self.arm_length , 0])
        print(self.eqi_angle_left)
        self.f_spring_system = np.array([0.0, 0.0, 0.0])
        self.start_position_vector_right_arm = np.array([math.cos(self.eqi_angle_right)*self.arm_length, math.sin(self.eqi_angle_right)*self.arm_length, 0.0])
        #self.vras_x = self.start_position_vector_right_arm[0]
        #self.vras_y = self.start_position_vector_right_arm[1]               #ADJUSTING ARM LENGTH
        self.start_position_vector_left_arm = np.array(([math.cos(self.eqi_angle_left)*self.arm_length, math.sin(self.eqi_angle_left)*self.arm_length, 0.0]))
        #self.vlas_x = self.arm_length
        #self.vlas_y = 0.0
        self.position_vector_right_arm = self.start_position_vector_right_arm
        self.position_vector_left_arm = self.start_position_vector_left_arm
        self.hook_point = np.array([0.0, -0.0, 0.0]) #As the plane are one meter below the hook point (seen in the y direction)
        self.start_left_rope_anchor =  np.add(self.placement_v_l, self.start_position_vector_left_arm) 
        self.start_right_rope_anchor = np.add(self.placement_v_r, self.start_position_vector_right_arm) 

    def append_motor_stuff(self):
        self.motor_theta_list.append(self.motor_l_theta)
        self.motor_omega_list.append(self.motor_l_omega)
        self.motor_alpha_list.append(self.motor_l_alpha)

    def est_vel(self, theta_l): #this method was written with 1 hand extreme laziness may be apparent
        '''
        print("2")
        
        first_item = True
        time_counter = 0
        
        self.neutral_yaw = 0
        
        for item in theta_l:
            if first_item: 
                self.neutral_yaw = item
                yaw_prev = item #Do nothing
                prev_t = self.time_l[time_counter]
                first_item = False
                time_counter += 1
            else:
                #trigom estimate
                yaw_now = item
                t_now = self.time_l[time_counter]
                delta_yaw = yaw_now - yaw_prev
                delta_t = t_now - prev_t
                yaw_speed = delta_yaw/delta_t
                quick_e = self.neutral_rope_length /math.cos(yaw_speed)
                self.est_speed_list.append(quick_e)
                #self.shorter_time_list.append(self.refined_time[time_counter])
                yaw_prev = item
                prev_t = self.time_l[time_counter]
                time_counter += 1
        '''
        #prolly badidea above, intead find y pos then diff
        first_item = True
        time_counter = 0
        local_t = []
        for item in theta_l:
            if first_item: 
                
                hyp = (self.neutral_rope_length + self.arm_length) /math.cos(item)
                quick_pos = hyp * math.sin(item)
                pos_prev = quick_pos #Do nothing
                prev_t = self.time_l[time_counter]
                first_item = False
                time_counter += 1
                self.est_pos_list.append(0)
            else:
                #trigom estimate
                hyp = (self.neutral_rope_length+self.arm_length) /math.cos(item)
                quick_pos = hyp * math.sin(item)
                print(self.plane_pos_l[time_counter][1], quick_pos)
                yaw_now = item
                t_now = self.time_l[time_counter]
                #print(t_now, prev_t)
                delta_pos = quick_pos - pos_prev
                delta_t = self.delta_t
                vel = delta_pos/delta_t
                #quick_e = self.neutral_rope_length /math.cos(yaw_speed)
                self.est_pos_list.append(quick_pos)
                
                #self.shorter_time_list.append(self.refined_time[time_counter])
                pos_prev = quick_pos
                prev_t = self.time_l[time_counter]
                time_counter += 1




        #plot
        #lets get indixes right
        fig = plt.figure
        print("ffs")
        new = []
        for item in self.plane_pos_l:
            new_i = item[1]
            new.append(new_i)
        plt.plot(self.est_t_l , self.est_pos_list, label="Estimated position")
        plt.plot(self.time_l, new, "r--", label="Actual position")
        print("t",len(self.est_t_l), "p", len(new))
        plt.title("Accuracy of position estimate")
        plt.ylabel('Y-axis position [m]')
        plt.xlabel("Time [seconds]")
        plt.legend()
        plt.show()


    def est_vel_y_alligned(self, theta_l):
        first_item = True
        time_counter = 0
        start_angle = ""
        start_float = 0.0
        #local_t = []
        for item in theta_l:
            if first_item: 
                
                #hyp = (self.neutral_rope_length + self.arm_length) /math.cos(item)
                #quick_pos = hyp * math.sin(item)
                start_float = item
                #Trigonometry see drawing in free body diagram under arrested recovery
                a_y = self.arm_length*math.sin(item)
                a_x = self.arm_length*math.cos(item)
                print("Error in est_vel_y, probably square root errors ", (self.half_distane_poles-a_x))
                
                
                try:
                     r_y = math.sqrt(self.neutral_rope_length**2-(self.half_distane_poles-a_x)**2)
                except:
                    r_y = 0
                else:
                    r_y = 0

                #Result
                y_pos_estimate = a_y-r_y
                
                pos_prev = y_pos_estimate #Do nothing
                prev_t = self.time_l[time_counter]
                first_item = False
                time_counter += 1
                self.est_pos_list.append(0)
                start_angle = str(item)
            else:
                #trigom estimate
                #Trigonometry see drawing in free body diagram under arrested recovery
                #item -= self.eqi_angle_left
                a_y = abs(self.arm_length*math.sin(item))
                a_x = self.arm_length*math.cos(item)
                
                #TODO: change line 367 it only made it worse.
                difference = self.neutral_rope_length**2-(self.half_distane_poles-a_x)**2
                #print("Difference", self.neutral_rope_length**2-(self.half_distane_poles-a_x)**2)
                    #r_y = math.sqrt(abs(self.neutral_rope_length**2-(self.half_distane_poles-a_x)**2))
                #if difference == 0:
                #    difference = 0.01
                r_y = math.sqrt(abs(difference))
                #print("item", item)
                #if self.time_l[time_counter]>0.12 and self.time_l[time_counter] < 0.2:
                #    print(self.time_l[time_counter],item, math.asin(r_y/self.neutral_rope_length))
                #except:
                #print("nope")
                #else:
                #    print(a_x)
                #    r_y = 0
                #Result
                #print("a_y", a_y, "a_x", a_x, "r_y", r_y)
                if item < 0:
                    y_pos_estimate = self.arm_length  -(a_y-r_y)
                else:
                    y_pos_estimate = self.arm_length +(a_y+r_y)
                #print(self.plane_pos_l[time_counter][1], quick_pos)
                yaw_now = item
                t_now = self.time_l[time_counter]
                #print(t_now, prev_t)
                delta_pos = y_pos_estimate - pos_prev
                delta_t = self.delta_t
                vel = delta_pos/delta_t
                #quick_e = self.neutral_rope_length /math.cos(yaw_speed)
                self.est_pos_list.append(y_pos_estimate)
                
                #self.shorter_time_list.append(self.refined_time[time_counter])
                pos_prev = y_pos_estimate
                prev_t = self.time_l[time_counter]
                time_counter += 1




        #plot
        #lets get indixes right
        fig = plt.figure
        print("ffs")
        new = []
        for item in self.plane_pos_l:
            new_i = item[1]
            new.append(new_i)
        plt.plot(self.est_t_l , self.est_pos_list, label="Estimated position")
        plt.plot(self.time_l, new, "r--", label="Actual position")
        print("t",len(self.est_t_l), "p", len(new))
        title_string = "Accuracy of position estimate start arm angle %s [Rad]" %(start_angle)
        plt.title(title_string)
        plt.ylabel('Y-axis position [m]')
        plt.xlabel("Time [seconds]")
        plt.legend()
        plt.show()

    def change_const(self, constant_id, amount, run_cnt):
        #new funct print test 
        print(constant_id, "before getattr", getattr(self, constant_id))
        class_var = getattr(self, constant_id)
        class_var += amount*run_cnt
        setattr(self, constant_id, class_var)
        print(constant_id, "after get/set attr", getattr(self, constant_id))

    def plot_thetas(self):
        plt.plot(self.time_l , self.left_rope_theta_list, label="Theta l r")
        plt.plot(self.time_l, self.left_arm_theta_list, "r--", label="Theta l a")
        #print("t",len(self.est_t_l), "p", len(new))
        title = "The two thetas of the left side"
        plt.ylabel('Thetha [rad]')
        plt.xlabel("Time [seconds]")
        plt.legend()
        plt.show()

    def plot_theta_err(self, plot_l):
        print(len(self.theta_err_ll))
        print(self.theta_err_ll)
        if len(plot_l) == 10:
            #r g b c mclass_var = getattr(self, constant_id)
            #class_var += amount*run_cnt
            plt.plot(self.time_l, plot_l[0], "m", label= self.parameter_change_id +" = " + str(getattr(self, self.parameter_change_id)-9*self.change_const_init))
            plt.plot(self.time_l, plot_l[1], "r", label=self.parameter_change_id +" = " + str(getattr(self, self.parameter_change_id)-8*self.change_const_init))
            plt.plot(self.time_l, plot_l[2], "g", label=self.parameter_change_id +" = " + str(getattr(self, self.parameter_change_id)-7*self.change_const_init))
            plt.plot(self.time_l, plot_l[3], "b", label=self.parameter_change_id +" = " + str(getattr(self, self.parameter_change_id)-6*self.change_const_init))
            plt.plot(self.time_l, plot_l[4], "c", label=self.parameter_change_id +" = " + str(getattr(self, self.parameter_change_id)-5*self.change_const_init))
            plt.plot(self.time_l, plot_l[5], "m--", label=self.parameter_change_id +" = " + str(getattr(self, self.parameter_change_id)-4*self.change_const_init))
            plt.plot(self.time_l, plot_l[6], "r--", label=self.parameter_change_id +" = " + str(getattr(self, self.parameter_change_id)-3*self.change_const_init))
            plt.plot(self.time_l, plot_l[7], "g--", label=self.parameter_change_id +" = " + str(getattr(self, self.parameter_change_id)-2*self.change_const_init))
            plt.plot(self.time_l, plot_l[8], "b--", label=self.parameter_change_id +" = " + str(getattr(self, self.parameter_change_id)-1*self.change_const_init))
            plt.plot(self.time_l, plot_l[9], "c--", label=self.parameter_change_id +" = " + str(getattr(self, self.parameter_change_id)+0*self.change_const_init))

           
            title = "FW acceleration at multiple arm orientations"
            plt.ylabel('FW acceleration [m/s^2]')
            plt.xlabel("Time [seconds]")
            plt.legend(loc='upper right')
            plt.title(title)
            plt.show()

        else:
            print("l error plot theta")

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
    def plot_plane_acc(self):
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
        
        new = []
        for item in self.plane_acc_l:
            new_i = item[1]
            new.append(new_i)
        print("t",len(self.time_l), "a", len(new))
        plt.plot(self.time_l , new, label="Fw acc")
        #plt.plot(self.time_l, new, "r--", label="True pos")
        print("t",len(self.est_t_l), "p", len(new))
        title = "The two thetas of the left side"
        plt.ylabel('Thetha [rad]')
        plt.xlabel("Time [seconds]")
        plt.legend()
        plt.show()
    
    def calc_theta_err(self):
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
       
        try:
            theta_err_l = []
            index = 0
            for item in self.left_arm_theta_list:
                err = item - self.left_rope_theta_list[index]
                theta_err_l.append(err)
                index += 1
            self.theta_err_ll.append(theta_err_l)
        except:
            print("houston...")
        else:
            print("Something went wrong in calc_theta_err")
         '''
        
        theta_err_l = []
        index = 0
        for item in self.left_arm_theta_list:
            err = item - self.left_rope_theta_list[index]
            theta_err_l.append(err)
            index += 1
        return theta_err_l
        #self.theta_err_ll.append(theta_err_l)
        
       

        '''
        print("plot theta", len(self.time_l), len(self.left_rope_theta_list))
        fig = plt.figure
        plt.plot(self.time_l, self.left_rope_theta_list, label="Rope theta")
        plt.plot(self.time_l, self.left_arm_theta_list, "r--", label="Arm theta")
        title = "The two thetas of the left side"
        plt.ylabel('Thetha [rad]')
        plt.xlabel("Time [seconds]")
        plt.legend()
        plt.show()
        #print(self.refined_time)
        '''

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
        stretch_v = hook_point - anchor_point
        #print("stretch vector", stretch_v)
        if self.cr_verbose:
            print("Rope stretch", stretch)
        lhat = stretch_v / ((stretch_v**2).sum())**0.5 #Normalized direction vector
        stretch = (stretch_v[0]**2+stretch_v[1]**2)**0.5 - naturel_length
        #print("Rope stretch", stretch)
        force = -spring_k*stretch*lhat
       
        #print("Rope force", force, stretch)
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
        #print("rope f", rope_force)
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
        #The length of the arm
        
         #Equilibrim angle of the torsion spring this angle is 135 degrees from the x axis cw
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
        
        position_vector[0] = self.arm_length*math.cos(self.theta_right)
        position_vector[1] = self.arm_length*math.sin(self.theta_right)
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
        #L = 0.4 #The length of the arm
        #self.torsion_K = 3 #The spring constant of the torsion spring
        #self.eqi_angle_left = 0 #Equilibrim angle of the torsion spring this angle is 45 degrees from the x axis cw
        spring_torque = (self.eqi_angle_left-self.theta_left)*self.torsion_K
        lever_torque = np.cross(position_vector, force_vector)
        total_torque = spring_torque + lever_torque[2] #fortegnsfejl.... og inverse vector passeret
        acceleration = total_torque/inertia_arm
        #print("total_t", force_vector)
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
        position_vector[0] = self.arm_length*math.cos(self.theta_left)
        position_vector[1] = self.arm_length*math.sin(self.theta_left)
        if self.right_arm_verbose:
            print("Omega left", self.omega_left)
            print("Theta left", self.theta_left)
            print("Position arm left", position_vector)
            #print("Acceleration of arm", alpha_vec)
        self.left_arm_position_x_l.append(position_vector[0])
        self.left_arm_position_y_l.append(position_vector[1])
        #print("Left arm position", position_vector)
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
    def init_model(self):

        # Append twice to get descrete acceleration
        self.plane_k_e_l.append(self.plane_k_e)
        # self.spring_l_u_e_l.append(self.spring_l_u_e)
        # self.spring_r_u_e_l.append(self.spring_r_u_e)
        #self.plane_pos_l.append(self.plane_pos)
        self.plane_vel_l.append(self.plane_vel)
        #self.plane_acc_l.append(self.plane_acc)
        self.total_force_l.append(0)
        self.all_purpose_l.append(0)
        self.end_pos_x_l.append(self.spring_l_end_point[0])
        self.end_pos_y_l.append(self.spring_l_end_point[1])
        #self.time_l.append(self.time)
        self.append_motor_stuff()
        self.neutral_rope_length = self.half_distane_poles-abs(self.arm_length*math.cos(self.eqi_angle_right))
        self.theta_right = self.eqi_angle_right#math.atan2(self.start_position_vector_right_arm[1], self.start_position_vector_right_arm[0])
        self.theta_left = self.eqi_angle_left#math.atan2(self.start_position_vector_left_arm[1], self.start_position_vector_left_arm[0])
    
    

    def main(self):
        no_runs = self.no_runs_init #keep local copy so we can re init every run
        run_no = self.run_no_init
        change_const = self.change_const_init

        ##
        plotting_list = []
        while run_no < no_runs:

            self.__init__()
            #lets make the rope stiffer
            if run_no == 0:
                
                self.no_runs_init = no_runs 
                self.run_no_init = run_no
                self.change_const_init = change_const
                self.vector_init()
                self.init_model()
                #print("left wqi angle, left theta", self.eqi_angle_left, self.theta_left)
            else:
                
                self.no_runs_init = no_runs
                self.run_no_init = run_no
                self.change_const_init = change_const
                self.change_const(self.parameter_change_id, change_const, run_no)
                self.vector_init()
                self.init_model()
                #print("left wqi angle, left theta", self.eqi_angle_left, self.theta_left)
                
            #Interactive plot
            if self.interactive:
                plot = PlotSystem(-3.5, 3.5, -1.5, 5.5)
            while self.time < self.time_to_run:
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
                rra_x = self.start_right_rope_anchor[0]+ self.position_vector_right_arm[0]
                rra_y = self.start_right_rope_anchor[1]+ self.position_vector_right_arm[1]
                right_rope_anchor = np.add(self.position_vector_right_arm, self.placement_v_r)#np.array([rra_x, rra_y, 0.0])
                self.right_rope_anchor_list.append(right_rope_anchor)#np.append(self.right_rope_anchor_l, right_rope_anchor)
                
                lra_x = self.start_left_rope_anchor[0] + self.position_vector_left_arm[0]
                lra_y = self.start_left_rope_anchor[1] + self.position_vector_left_arm[1]
                left_rope_anchor = np.add(self.position_vector_left_arm, self.placement_v_l)#np.array([lra_x, lra_y, 0.0])
                #print("Left_rope_anchor", left_rope_anchor)
                #print("Rope anchors r / l", right_rope_anchor, left_rope_anchor)
                #self.right_rope_anchor_x.append(right_rope_anchor[0])
                #self.right_rope_anchor_y.append(right_rope_anchor[1])
                #We have two ropes each going from their left anchor to the hooking point
                #These lines dont need to be in the while loop
                if not self.plane_hooked:
                    start_sum_hook_points = np.add(self.start_right_rope_anchor, self.start_left_rope_anchor)
                    position_arm_diff = np.subtract(self.position_vector_left_arm, self.position_vector_right_arm)
                    hook_y = self.position_vector_left_arm[1]
                    #print("hok y", hook_y)
                    hook_point = np.array([0.0, 0, 0.0])
                    #print("position_arm_diff", position_arm_diff)
                    #print("Start_sum_hook_point", start_sum_hook_points)
                    #print("hook point", hook_point)
            
                if self.plane_hooked:
                    hook_point = self.plane_pos
                    #self.right_arm_position_l.append(position_vector_right_arm)
                
                f_rope_l = model.connect_rope(self.neutral_rope_length, self.rope_k, hook_point, left_rope_anchor)
                #print("self neutral rope lenghr", self.neutral_rope_length)
                #print("Rope", self.hook_point, left_rope_anchor)
                f_rope_r = model.connect_rope(self.neutral_rope_length, self.rope_k, hook_point, right_rope_anchor)
                self.left_arm_theta_list.append(self.theta_left)
                self.est_t_l.append(self.time)

                if not self.plane_hooked:
                    self.left_rope_theta_list.append(math.atan2(0, 1))
                else:
                    self.left_rope_theta_list.append(math.atan2(self.plane_pos[1]-lra_y, self.plane_pos[0]-lra_x))
                self.f_spring_system = f_rope_l + f_rope_r
                #f_damping = -f_spring_system*0.1
                #f_spring_system += f_damping
                #print("f_rope_r", f_rope_r)
                #print("rope f", f_rope_r)
                self.position_vector_right_arm = self.right_arm_kin(np.negative(f_rope_r), self.position_vector_right_arm)
                self.position_vector_left_arm = self.left_arm_kin(np.negative(f_rope_l), self.position_vector_left_arm)
                #print("SPring force", f_spring_system)
                if self.cr_verbose:
                    print("Force from the two ropes", self.f_spring_system)
                    print("Force from left rope", f_rope_l)
                    print("Force from right rope", f_rope_r)
                #f_spring_system = f_rope_l + f_rope_r
                #The dynamics of the plane
                if self.plane_hooked:
                    #self.f_spring_system[1] += plane_drag_force
                    #y_force = force_on_plane/self.plane_mass #Maybe move this to main
                    #braking_de_acc = np.array([0.0, y_acc, 0.0])

                    if self.motor_phase == 2:
                        force_on_plane = 2*(self.motor_force*math.sin(self.theta_left)) # We have two motors therefore multiplying by two
                        self.f_spring_system[1] += force_on_plane
                        
                    self.plane_acc = self.f_spring_system / self.plane_mass
                        
                if self.plane_verbose:
                    print("Plane acc", self.plane_acc)
                self.plane_vel = self.plane_vel + self.plane_acc * self.delta_t
                
                self.plane_pos = self.plane_pos + self.plane_vel * self.delta_t
                
                #Check if the plane is hooked to the rope yet
                if not self.plane_hooked and self.plane_pos[1] > hook_point[1]:
                    self.plane_hooked = True
                
                if self.plane_verbose:
                    print("plane pos", self.plane_pos)
                    print("Is plane hooked?", plane_hooked)            
                
                #Appending to lists with intresting parameters
                self.plane_pos_l.append(self.plane_pos)
                self.plane_vel_l.append(self.plane_vel)
                self.plane_acc_l.append(self.plane_acc[1])
                self.total_force_l.append(self.f_spring_system)
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
                    left_arm = [[self.placement_v_l[0], 
                    self.placement_v_l[1]], 
                    [self.placement_v_l[0]+self.position_vector_left_arm[0], 
                    self.placement_v_l[1]+self.position_vector_left_arm[1]]]
                    right_arm = [[self.placement_v_r[0], 
                    self.placement_v_r[1]], 
                    [self.placement_v_r[0]+self.position_vector_right_arm[0], 
                    self.placement_v_r[1]+self.position_vector_right_arm[1]]]
                    extra_vec = [self.time, 0, 0, 0]#[self.start_position_vector_right_arm[0]-self.arm_length*math.cos(self.theta_right), self.start_position_vector_right_arm[1]-self.arm_length*math.sin(self.theta_right), self.start_position_vector_right_arm[0], self.start_position_vector_right_arm[1]]#extra_vec = [right_rope_anchor[0], right_rope_anchor[1], f_rope_r[0], f_rope_r[1]]
                    plot.update_plot(plane_plot, [0, 0], [0, 0], left_rope, right_rope, left_arm, right_arm, extra_vec) #Plane[ x, y, x vel, y vel], [0, 0], [0, 0], left_rope[armx, army, planex, planey], 
                                                                    #[[self.start_right_rope_anchor[0]-self.vras_x, self.start_right_rope_anchor[1]-self.vras_y], [right_rope_anchor[0], right_rope_anchor[1]]]
                    #right_rope[armx, army, planex, planey], left_arm[centerx, centery, endpointx, endpointy], right_arm[centerx, centery, endpointx, endpointy]
                    time.sleep(self.time_to_sleep)
            #after 1 run saving
            # theta err save and plotplotting_list.append(self.calc_theta_err()) #also saves the list
            #self.plot_plane_acc()
            self.est_vel_y_alligned(self.left_arm_theta_list)
            plotting_list.append(self.plane_acc_l)
            run_no += 1
            plt.close('all')    
            #self.plot_thetas()
        print("here", len(plotting_list))
        self.plot_theta_err(plotting_list)
        #self.plot_motor()
        #self.plot_right_arm()
        


        cnt = 0


if __name__ == "__main__":
    model = ar_model()
    model.main() # spring_forcediagram()
    
   