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
import cv2
import numpy as np
import csv
from os import listdir
from os.path import isfile, join
import matplotlib.pyplot as plt


class CSV_to_speed(object):
    def __init__(self):
        self.csv_folder_path = '../data/test3/'
        # Following line lists all files in the folder you look for
        self.csv_files = [f for f in listdir(self.csv_folder_path) if isfile(join(self.csv_folder_path, f))]
        self.frame_rate = 0

    def plot_xyz(self, x, y, z, time, title="xyz"):
        plt.subplot(3, 1, 1)
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

    def pos_to_vel_acc(self, time_list, pos_list, title=""):
        x = time_list
        y = pos_list

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
        time_l = []
        pos_l = []

        with open(path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                line_count += 1
                if line_count == 1:
                    self.frame_rate = row[7]
                curr_time += 1 / float(self.frame_rate)
                if line_count == 4:
                    # print('{:>3}'.format(line_count), row)
                    for i in range(len(row)):
                        if row[i] == "drone":
                            x_elem = i + 4
                            y_elem = i + 5
                            z_elem = i + 6
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
                        x_l.pop()
                        continue
                    if row[z_elem] is not "":
                        z = float(row[z_elem])
                    else:
                        z = z_l[-1]
                        x_l.pop()
                        y_l.pop()
                        continue

                    x_l.append(x)
                    y_l.append(y)
                    z_l.append(z)
                    time_l.append(curr_time)
                    pos_l.append((x**2 + y**2 + z**2)**0.5)

                    # print("vel", vel)
                    # old_vel = vel

                # if line_count >= 18:
                #     break

        # self.plot_xyz(x_l, y_l, z_l, time_l)
        self.pos_to_vel_acc(time_l, z_l)

        # print(z_l[1])
        # print(time_l[1])

if __name__ == "__main__":
    reader = CSV_to_speed()
    reader.run()
