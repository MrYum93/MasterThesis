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
This script takes a CSV file with pos and makes velocity

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

class speed_from_pos_IMU(object):
    def __init__(self):
        self.csv_folder_path = '../data/test6_vectorNav_opti/'
        self.csv_files = [f for f in listdir(self.csv_folder_path) if isfile(join(self.csv_folder_path, f))]

        self.pos_l = []  # pos
        self.time_l = []  # time

    def plot_diff_np(self, time_l, pos_l):

        x = time_l
        y = pos_l
        # y = [item[1] for item in pos_list]
        # print("y", y)

        dy = np.zeros(y.__len__(), np.float)
        ddy = np.zeros(y.__len__(), np.float)
        dy[0:-1] = np.diff(y) / np.diff(x)
        ddy[0:-1] = np.diff(dy) / np.diff(x)
        dy[-1] = (y[-1] - y[-2]) / (x[-1] - x[-2])
        ddy[-1] = (dy[-1] - dy[-2]) / (x[-1] - x[-2])

        # Remove the last two points bc of spike in acceleration
        x = x[:-2]
        y = y[:-2]
        dy = dy[:-2]
        ddy = ddy[:-2]

        plt.subplot(3, 1, 1)
        plt.title("Plane movement along the y-axis")
        plt.plot(x, y, 'r-')
        plt.xlabel('Time', fontsize=18)
        plt.ylabel('Plane y pos', fontsize=16)

        plt.subplot(3, 1, 2)
        plt.plot(x, dy, 'r')
        plt.xlabel('Time', fontsize=18)
        plt.ylabel('Plane y velocity', fontsize=16)

        plt.subplot(3, 1, 3)
        plt.plot(x, ddy, 'r')
        plt.xlabel('Time', fontsize=18)
        plt.ylabel('Plane y acceleration', fontsize=16)

        plt.show()

    def diff(self, time_l, x_l):
        dx = []
        for i in range(len(time_l)):
            if (i == 0):
                h = time_l[i+1] - time_l[i]
                dx.append((x_l[i+2] - x_l[i]) / (2*h))
            elif (i == len(time_l) - 1):
                h = time_l[i] - time_l[i-1]
                dx.append((x_l[i] - x_l[i-2]) / (2 * h))
            else:
                h = time_l[i + 1] - time_l[i]
                dx.append((x_l[i+1] - x_l[i-1]) / (2*h))

        return dx


    def plot_diff(self, time_l, pos_l):
        x = time_l
        y = pos_l
        # y = [item[1] for item in pos_list]
        # print("y", y)

        dy = self.diff(x, y)
        ddy = self.diff(x, dy)

        # Remove the last two points bc of spike in acceleration
        # x = x[:-2]
        # y = y[:-2]
        # dy = dy[:-2]
        # ddy = ddy[:-2]

        plt.subplot(3, 1, 1)
        plt.title("Plane movement along the y-axis")
        plt.plot(x, y, 'r-')
        plt.xlabel('Time', fontsize=18)
        plt.ylabel('Plane y pos', fontsize=16)

        plt.subplot(3, 1, 2)
        plt.plot(x, dy, 'r')
        plt.xlabel('Time', fontsize=18)
        plt.ylabel('Plane y velocity', fontsize=16)

        plt.subplot(3, 1, 3)
        plt.plot(x, ddy, 'r')
        plt.xlabel('Time', fontsize=18)
        plt.ylabel('Plane y acceleration', fontsize=16)

        plt.show()

    def load_csv(self, folder):
        # if no folder is provided use the first
        if folder == '':
            folder = self.csv_files[0]

        path = self.csv_folder_path + folder
        # print('path:', path)
        pos_l = []
        time_list = []

        with open(path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if row[0] == 'time':
                    continue

                self.time_l.append(float(row[0])/1000)  # convert time-unit to sec.
                                                        # Run at 200Hz so should be equal to 0.005
                self.pos_l.append(float(row[2]))        # FW

            print(self.time_l[0] - self.time_l[1])
            time_zero = self.time_l[0]
            for i in range(len(self.time_l)):
                self.time_l[i] -= time_zero


    def run(self):
        self.load_csv('vel_est_test_3_33.txt')
        self.plot_diff(self.time_l, self.pos_l)
        # self.plot_diff_np(self.time_l, self.pos_l)


if __name__ == "__main__":
    reader = speed_from_pos_IMU()
    reader.run()










