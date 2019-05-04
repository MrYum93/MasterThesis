#!/usr/bin/env python
# /***************************************************************************
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
This is a script containing all functions needed to pass RoVi1

NOT YET: Rest of lectures...

Revision
YYYY-MM-DD
2018-11-07 MW First version
'''
#import cv2
import numpy as np
import csv
from os import listdir
from os.path import isfile, join
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import itertools
from matplotlib.ticker import FormatStrFormatter


class CSV_to_speed(object):
    def __init__(self):
        self.csv_folder_path = 'optitrack/' 
        # Following line lists all files in the folder you look for
        self.csv_files = [f for f in listdir(self.csv_folder_path) if isfile(join(self.csv_folder_path, f))]
        self.frame_rate = 0
        self.an_x = [] #Hopefully one pole
        self.an_y = []
        self.an_z = []

        self.ano_x = []
        self.ano_y = []
        self.ano_z = [] #Hopefully another pole

        self.debug = False
        self.time_list =[]
        self.specific_debug = True
  #cv = 1 2 .. [6 7 8]
    def remove_extensive_points(self, x_l, y_l, z_l, over_x, over_y, over_z, filter_x, filter_y, filter_z, t_l):
        if self.debug:
            print("Passed length of lists in remove extensive_points", len(x_l), len(y_l), len(z_l), len(t_l))
        #They get removed if they're true in the over_x else when they're below
        #Over_x means that a point will be removed if it has a value which is "over_x"
        
        ##DISCLAIMER
            # if you dont want to filt er a value, just set it as you dont expect it to appear....
        
        ##END 
        #bool_vector = [over_x, over_y, over_z]
        #raw_mat = [x_l, y_l, z_l]
        #filter_vector = [filter_x, filter_y, filter_z]
        #Would be best practice to remove the specific indexed but time constraints
        #refined_x = [] #x_l
        #refined_y = [] #y_l
        ##refined_z = []#z_l
        #refined_mat = [refined_x, refined_y, refined_z]
        #list_counter = 0 #x= 0, y=1, z=2
        #item_counter = 0


        #MORE local variables
        x_li = [] #xlist_item
        y_li = []
        z_li = []
        t_li = []
        item_counter = 0
        for item in x_l:
            x_good = True
            y_good = True
            z_good = True
            if over_x:
                if item > filter_x:
                    x_good = False
            else:
                if item < filter_x:
                    x_good = False
            
            if over_y:
                if y_l[item_counter] > filter_y:
                    y_good = False
            else:
                if y_l[item_counter] < filter_y:
                    x_good = False
            
            if over_z:
                if z_l[item_counter] > filter_z:
                    z_good = False
            else:
                if z_l[item_counter] < filter_z:
                    z_good = False
            if self.debug:
                print("Bools for keeping variables", x_good, y_good, z_good)
            if z_good and x_good and y_good:
                x_li.append(item)
                y_li.append(y_l[item_counter])
                z_li.append(z_l[item_counter])
                t_li.append(t_l[item_counter])
            item_counter += 1
            


        #item_counter = 0
        #for item in x_l:
            
        '''for list in raw_mat:
            item_counter = 0
            for item in list:
                if bool_vector[list_counter] is True: #The x value needs to be above something
                    if item > filter_vector[list_counter]:
                        x_li = raw_mat[0] #xlist_item
                        y_li = raw_mat[1]
                        z_li = raw_mat[2]
                        #print("Removed", item, "in list", list_counter, "Due to", bool_vector[list_counter], "Bool at the filter of", filter_vector[list_counter])
                        #print("Removed", x_li[item_counter], y_li[item_counter], z_li[item_counter])
                        #x_li.pop(item_counter)
                        #y_li.pop(item_counter)
                        #z_li.pop(item_counter)
                        x_li.append(item)
                        y_li.append(item)
                        z_li.append(item)
                             
                        #item_counter -= 1
                                                
                else:
                    if item < filter_vector[list_counter]:
                        #x_li = raw_mat[0]
                        #y_li = raw_mat[1]
                        #z_li = raw_mat[2]
                        #print("Removed", item, "in list", list_counter, "Due to", bool_vector[list_counter], "Bool at the filter of", filter_vector[list_counter])
                        
                        x_li.append(item)
                        y_li.append(item)
                        z_li.append(item)
                        
                        #item_counter -= 1
                        
                item_counter += 1
            print("List couter incremented, item counter to", item_counter)    
            list_counter += 1
        '''
        if self.debug:
            print("Passed length of lists in remove extensive_points after algorithm", len(x_li), len(y_li), len(z_li), len(t_li))

        return x_li, y_li, z_li, t_li
                        #Remove point ix_lll three lists

    def calc_dis(self, x, y, z, start_x, start_y, start_z):
        #if self.debug:
        #    print("x, and start_x in calc_dis", x, start_x)
        x_dif = x-start_x
        y_dif = y-start_y
        z_dif = z-start_z

        dis = (x_dif**2 + y_dif**2 + z_dif**2)**0.5
        print("Distance", dis)
        return dis
    
    def find_dist_to_hooking_point(self, x, y, z, c, hx, hy, hz, time):
        #This method will find the distance to the hooking point using the sorted data.
        #It will automatically execute when the distance do not increase more than the previous + a small constant to regulate for small deviations.

        #First the distance will be negative. Using pythagoras this will off course result in a positive value. so finding the point where the distance becomes smaller then bigger.
        #Will determine when the hooking point has been reached and the distance should not be set negative.
        dis = 0.1
        last_dis = 1
        dis_l = []
        time_list = []
        list_counter = 0
        
        for item in x:
            dis = self.calc_dis(x[list_counter], y[list_counter], z[list_counter], hx, hy, hz)
            #delta_dis = dis-last_dis
            dis_l.append(dis)
            time_list.append(time[list_counter])
            list_counter += 1
            
        '''
        while True:
            last_dis = dis
            dis = self.calc_dis(x[list_counter], y[list_counter], z[list_counter], hx, hy, hz)
            delta_dis = dis-last_dis
            dis_l.append(dis)
            time_list.append(time[list_counter])
            list_counter += 1
            if list_counter > 25:
                break
        '''
            #
            #if delta_dis > last_dis:
            #    break
            
        
        

        #Next step is calculating the distance and also checking when it becomes smaller
        #while dis > last_dis + c:
        #    dis = self.calc_dis(x[list_counter], y[list_counter], z[list_counter], hx, hy, hz)
        #    last_dis = dis
        #    dis_l.append(dis)
        #    time_list.append(time[list_counter])
        #    list_counter += 1
        float_time = []
        for item in time_list:
            float_t = float(item)
            float_time.append(float_t)

        print(float_time)
        print("Length of distance to dock and time list", len(dis_l),len(float_time))
        #Plot to verify the algorithm
        fig, ax = plt.subplots()

        #ax.xaxis.set_major_formatter(FormatStrFormatter('%.5f'))
        #ax.xaxis.set_ticks(np.arange(float_time[0], float_time[-1], 0.1))

        #x = np.arange(-1, 1, 0.1)
        #plt.plot(x, x**2)
        #plt.show()
        
        
        plt.title("Distance between FW and estimated hooking point")
        plt.plot(float_time, dis_l, 'r')
        plt.xlabel('Time[s]', fontsize=12)
        plt.ylabel('Distance[m]', fontsize=12)
        #.yaxis.set_major_formatter(FormatStrFormatter('%.1'))
        plt.show()

        self.pos_to_vel_acc(float_time, dis_l, "Speed of distance")




    def plot_xyz(self, x, y, z, time, title="Optitrack data analysis of"): 
        ##THis makes more sense 
        
        if self.specific_debug:
            print("Time list in plot xyz", time)
        temp_y = y
        temp_z = z
        
        y = temp_z
        z = temp_y

        ###
        
        title += " " + str(self.csv_files[1])
        print(title)
        
        #0 = x*a+y*b+z*c 

        x, y, z, new_time = self.remove_extensive_points(x, y, z, True, False, False, 2, -0.2, 1.9350396428571442, time)
        
        #SMall print loop
        counter = 0
        for item in y:
            print("y, z, t", item, z[counter], new_time[counter])
            counter += 1
        
        
        
        hook_x = -0.18
        hook_y = 0.21240174301 
        hook_z = 2.120824179487176
        self.find_dist_to_hooking_point(x, y, z, 0.0, hook_x, hook_y, hook_z, new_time)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(x, y, z, label="Optitrack FW estimate")
        ax.scatter(self.an_x, self.an_y, self.an_z, label ="Left pole marker") #an = right ano = left
        right_pole_com = [0.0, 0.0, 0.0]
        left_pole_com = [0.0, 0.0, 0.0]
        


        1.9452805347985354, 0.3916005000000005, 2.0350396428571442
        for x in self.an_x:
            right_pole_com[0] += x/len(self.an_x)

        for y in self.an_y:
            right_pole_com[1] += y/len(self.an_x)

        for z in self.an_z:
            right_pole_com[2] += z/len(self.an_z)

        for x in self.ano_x:
            left_pole_com[0] += x/len(self.ano_x)

        for y in self.ano_y:
            left_pole_com[1] += y/len(self.ano_x)

        for z in self.ano_z:
            left_pole_com[2] += z/len(self.ano_z)

        print("Right pole COM", right_pole_com)
        print("left pole COM", left_pole_com)
        rope_l_x = []
        rope_l_y = []

        x_it = (left_pole_com[0]+right_pole_com[0])/100 + left_pole_com[0] + 2*abs((left_pole_com[0]+right_pole_com[0])/100)
        print("INFO", x_it, left_pole_com[0], right_pole_com[0])
        while x_it > left_pole_com[0] and x_it < right_pole_com[0]:
            #print(x_it, right_pole_com)
            y = (-2.120824179487176 -0.785763*x_it)/(-11.8599*0.785763)
            x = ((+11.8599*0.785763)*y-2.120824179487176) /(0.785763)
            rope_l_x.append(x)
            rope_l_y.append(y)
            #.append([x, y, 2.120824179487176])
            x_it += abs(((left_pole_com[0]+right_pole_com[0])/50))
            #print("Delta", ((left_pole_com[0]+right_pole_com[0])/50))

        ax.scatter(rope_l_x, rope_l_y, right_pole_com[2], label="Estimated rope")
        ax.scatter(self.ano_x, self.ano_y, self.ano_z, label="Left pole marker")
        ax.set_xlabel('x') #aa = 27 so we start at 26 for second pole
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.legend()
        plt.title(title)
        plt.show()

        
        '''plt.subplot(3, 1, 1)
        plt.title(title)
        plt.plot(time, x, 'r')
        plt.xlabel('time', fontsize=16)
        plt.ylabel('x pos', fontsize=18)

        plt.subplot(3, 1, 2)
        plt.plot(time, y, 'r')
        plt.xlabel('time', fontsize=16)
        plt.ylabel('y pos', fontsize=18)

        plt.subplot(3, 1, 3)
        plt.plot(time, z, 'r')
        plt.xlabel('time', fontsize=16)
        plt.ylabel('z pos', fontsize=18)

        plt.show()
        '''
    def pos_to_vel_acc(self, time_list, pos_list, title=""):
        x = time_list
        y = pos_list
        '''
        last_pos = pos_list[0] + 0.001
        last_t = time_list[0] + 0.001
        speed_list = []
        
        for item in pos_list:
            delta_pos = item - last_pos
            delta_t = t -last_t
            speed = delta_pos/delta_t
            speed_list.append(speed)
        
        
        '''
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
        plt.plot(time_list, pos_list, 'r')
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

    

    def run(self):
        '''
        in CSV files the useful cells are
        2:5 Which is the rotation in x,y,z,w Quaternians
        6:9 which are the position in x,y,z with ?? being forward, ?? being upwards and ?? being left

        Row 8+ is data rest is header
        :return:
        '''
        path = self.csv_folder_path + self.csv_files[0]
        print('path:', path)
        curr_time = 0.0
        x_elem = 0
        y_elem = 0
        z_elem = 0
        x_l = []
        y_l = []
        z_l = []
        final_t = []
        time_l = []
        pos_l = []
        time_list = []
        #an_x = []
        #an_y = []
        #an_z = []
        with open(path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                #row 6 7 and 8 is the cv body x y and z
                line_count += 1
                
                if line_count == 1:
                    self.frame_rate = row[7]
                if line_count > 6:
                    time_list.append(row[1])
                curr_time += 1 / float(self.frame_rate)

                if line_count > 7:
                    if row[6] is not '' and row[7] is not '' and row[8] is not '':
                        self.an_x.append(float(row[6]))
                        self.an_y.append(float(row[8]))
                        self.an_z.append(float(row[7]))
                
                if line_count > 7:
                    if row[26] is not '' and row[27] is not '' and row[28] is not '':
                        self.ano_x.append(float(row[26]))
                        self.ano_y.append(float(row[28]))
                        self.ano_z.append(float(row[27])) #y is z and vice versa
                
                #if line_count == 1:
                #    self.frame_rate = row[7]
                
                if line_count == 4:
                    # print('{:>3}'.format(line_count), row)
                    for i in range(len(row)):
                        if row[i] == "drone":
                            x_elem = i + 4
                            y_elem = i + 5
                            z_elem = i + 6
                            t_elem = i - 42
                            break
                if line_count == 7:
                    # print("xyz:", row[6:9])
                    pass
                if line_count >= 8:
                    # print('{:>3}'.format(line_count), row[6:9])
                    if row[x_elem] is not "":
                        x = float(row[x_elem])
                    else:
                        x = x_l[-1]
                        continue
                    if row[y_elem] is not "":
                        y = float(row[y_elem])
                    else:
                        y = y_l[-1]
                        continue
                    if row[z_elem] is not "":
                        z = float(row[z_elem])
                    else:
                        z = z_l[-1]
                        continue
                    if row[t_elem] is not "":
                        t = float(row[t_elem])

                    x_l.append(x)
                    y_l.append(y)
                    z_l.append(z)
                    final_t.append(t)
                    time_l.append(curr_time)
                    if line_count < 600:
                        print("line_count", line_count)
                    #t_list.append()
         
                    pos_l.append((x**2 + y**2 + z**2)**0.5)

                    # print("vel", vel)
                    # old_vel = vel

                # if line_count >= 18:
                #     break

        #self.time_l 

        # self.plot_xyz(x_l, y_l, z_l, time_l)
        self.plot_xyz(x_l, y_l, z_l, time_l)
        #self.pos_to_vel_acc(time_l, z_l)

        # print(z_l[1])
        # print(time_l[1])

if __name__ == "__main__":
    reader = CSV_to_speed()
    reader.run()
