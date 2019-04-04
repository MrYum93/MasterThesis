# !/usr/bin/env python
# -*- coding: utf-8 -*-
# # /***************************************************************************
# Copyright (c) 2018, Mathias W. Madsen <matam14@student.sdu.dk> <mwittenm@gmail.com>
#                     Mark Buch         <mabuc13@student.sdu.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ****************************************************************************
'''
The purpose of this script is to verify that the speed of the fixed wing can be estimated to a satisfying degree using only yaw data from the docking station. Where the math from simple_dock_model.py is used.
The experiment analysed is the one conducted 26-03-19
Revision control
YYYY-MM-DD
2019-04-03 Created the program.
'''


import math
import numpy as np
import matplotlib.pyplot as plt
import sys
import argparse
#from plot_system import PlotSystem
#import time


class speed_estimator(object):
    def __init__(self):
        self.neutral_rope_length = 2.5 #By memory this should be in the log too, 
        self.current_rope_length = self.neutral_rope_length
        self.FW_speed_list = []
        self.shorter_time_list = []
        self.neutral_yaw = 0
    #From the analyzer class
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument('--n', help='the name/path to the file to analyse')
        #self.parser.add_argument('--p', help ='Use --p if you want the histogram of the period instead of the low and high flanks')
        #self.parser.add_argument('--s', help ='Use --s if you want the histogram saved instead of shown')
        self.args = self.parser.parse_args()

        self.f = open(self.args.n, "r")
        self.time_list = []
        self.refined_time = [] #List which fixes the bug mentioned in scale seconds
        self.yaw_list = []
        
        
    def read_file(self): #The wierd IMU code i write first writes the yaw, time
        for line in self.f:
            line_list = line.split(',')
            self.time_list.append(float(line_list[1]))
            self.yaw_list.append(float(line_list[0]))

    def scale_seconds(self): #I found another bug in the IMU program, the seconds just rollback after 60, to fix this we need to count
        minute_counter = -1
        at_zero = False
        first = True
        for time in self.time_list:
            if first:
                if int(time) > 0:
                    minute_counter = 0
                    first = False
            if int(time) == 0 and not at_zero:
                minute_counter += 1
                at_zero = True            
            if int(time) == 1:
                at_zero = False
            scaled_time = time+minute_counter*60
            
            #print(int(time), minute_counter, scaled_time)
            self.refined_time.append(scaled_time)
    
    

    def raw_yaw_plot(self): #Lets plot time versus yaw from the file provided at the command line
        fig = plt.figure
        plt.plot(self.refined_time, self.yaw_list)
        title = "Yaw in function of time of the file %s" %(self.args.n)
        plt.ylabel('Yaw [degrees]')
        plt.xlabel("Time [seconds]")
        plt.show()

    def quick_estimate(self, t_prev, t_now, yaw_now, yaw_prev):
        yaw_now -= self.neutral_yaw
        yaw_now = math.radians(yaw_now)
        yaw_prev -= self.neutral_yaw 
        yaw_prev = math.radians(yaw_prev)

        delta_yaw = yaw_now - yaw_prev
        delta_t = t_now - t_prev
        #self.neutral_rope_length/math.cos()
        yaw_speed = delta_yaw/delta_t
        print("delta_t", delta_t, "Delta, yaw", delta_yaw, "yaw_speed", yaw_speed)
        #Try next line
        
        maybe_fw_speed = self.neutral_rope_length /math.cos(yaw_speed)#before diffmath.cos(yaw_speed)
                                                    #Differentiate right side
        #new_rope_length = math.sin(yaw_now)/self.neutral_rope_length
        #fw_y = math.sqrt(new_rope_length**2-self.neutral_rope_length**2)

        #Using start rope length and the yaw increment a new rope length can be calculated
        #Where we assume that th earm orientation is perfectly aligned with the ropes direction
        return maybe_fw_speed

    def estimate_controller(self):
        first_item = True
        time_counter = 0
        self.neutral_yaw = 0
        for item in self.yaw_list:
            if first_item: 
                self.neutral_yaw = item
                prev_yaw = item #Do nothing
                prev_t = self.refined_time[time_counter]
                first_item = False
                time_counter += 1
            else:
                self.FW_speed_list.append(self.quick_estimate(prev_t, self.refined_time[time_counter], item, prev_yaw))
                self.shorter_time_list.append(self.refined_time[time_counter])
                prev_yaw = item
                prev_t = self.refined_time[time_counter]
                time_counter += 1



    def plot_speed_n_yaw(self):
        fig = plt.figure
        plt.plot(self.shorter_time_list, self.FW_speed_list, "r--", self.refined_time[1:], self.yaw_list[1:])
        title = "FW speed in function of time %s" %(self.args.n)
        plt.ylabel('FW y direction speed [m/s^2]')
        plt.xlabel("Time [seconds]")
        plt.show()
        print(self.refined_time)


if __name__=="__main__":
    estimator = speed_estimator()
    estimator.read_file()
    estimator.scale_seconds()
    estimator.estimate_controller()
    estimator.plot_speed_n_yaw()