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
        self.csv_folder_path = '../data_for_git/test6_vectorNav_opti/'
        self.csv_files = [f for f in listdir(self.csv_folder_path) if isfile(join(self.csv_folder_path, f))]

        self.opti_data_path = '../data_for_git/test6_vectorNav_opti/Optitrack_from_mark_3_20.csv'

        # global lists
        self.imu_pos_l = []  # pos
        self.imu_vel_l = []  # vel
        self.imu_yaw_l = []  # yaw
        self.imu_time_l = []  # time

        self.opti_pos_l = []  # pos
        self.opti_vel_l = []  # vel
        self.opti_time_l = []  # time

    def plot_diff_simple_np(self, time_l, pos_l):

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

    def plot_pos_only(self, time_l, pos_l_imu, pos_l_opti, title='Position of FW from OptiTrack and IMU estimate'):
        x = time_l
        y = pos_l_imu
        y2 = pos_l_opti
        # y = [item[1] for item in pos_list]
        # print("y", y)

        plt.title(title, fontsize=20)
        plt.grid(color='k')
        plt.plot(x, y, 'r-', label='IMU estimate')
        plt.plot(x, y2, 'g-', label='OptiTrack Data')
        plt.xlabel('Time [s]', fontsize=12)
        plt.ylabel('Plane y pos [m]', fontsize=12)

        plt.show()


    def plot_diff(self, time_l, pos_l, opt_data=[], title='plots'):
        x = time_l
        y = pos_l
        # y = [item[1] for item in pos_list]
        # print("y", y)

        dy = self.diff(x, y)
        ddy = self.diff(x, dy)

        plt.subplot(3, 1, 1)
        plt.title(title)
        plt.plot(x, y, 'r-')
        plt.xlabel('Time', fontsize=18)
        plt.ylabel('Plane y pos', fontsize=16)

        plt.subplot(3, 1, 2)
        plt.plot(x, dy, 'r')
        plt.xlabel('Time', fontsize=18)
        plt.ylabel('Plane y velocity', fontsize=16)

        if len(opt_data) is 0:
            plt.subplot(3, 1, 3)
            plt.plot(x, ddy, 'r')
            plt.xlabel('Time', fontsize=18)
            plt.ylabel('Plane y acceleration', fontsize=16)
        else:
            plt.subplot(3, 1, 3)
            plt.plot(x, opt_data, 'r')
            plt.xlabel('Time', fontsize=18)
            plt.ylabel('opt data', fontsize=16)

        plt.show()

        return dy, ddy

    def load_imu_csv(self, file=''):
        # if no folder is provided use the first
        if file == '':
            file = self.csv_files[0]

        path = self.csv_folder_path + file
        # print('path:', path)

        self.imu_time_l.append(0.0)  # convert time-unit to sec.
        # Run at 20Hz so should be equal to 0.05
        self.imu_yaw_l.append(0.0)  # yaw
        self.imu_pos_l.append(0.0)  # placeholder for the first 0 pos

        with open(path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if row[0] == 'time':
                    continue

                if float(row[2]) == 0.0:  # FW pos
                    self.imu_time_l[0] = float(row[0])/1000
                    self.imu_yaw_l[0] = float(row[1])
                    self.imu_pos_l[0] = float(row[2])
                    continue

                if float(float(row[0])/1000) > 40459.05058:
                    continue

                # print("time, yaw", float(row[0])/1000, float(row[2]))
                self.imu_time_l.append(float(row[0])/1000)  # convert time-unit to sec.
                                                        # Run at 20Hz so should be equal to 0.05
                self.imu_yaw_l.append(float(row[1]))        # yaw
                self.imu_pos_l.append(float(row[2]))        # FW pos

            # print(self.imu_time_l[0] - self.imu_time_l[1])
            time_zero = self.imu_time_l[0]
            for i in range(len(self.imu_time_l)):
                self.imu_time_l[i] -= time_zero

    def load_opti_csv(self, time_index=0, pos_index=1):
        path = self.opti_data_path

        pos_l = []
        vel_l = []
        time_l = []
        with open(path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                # Out-comment the lines below to get before hooking point, looks nice
                if float(row[pos_index]) < 0:
                    continue
                time_l.append(float(row[time_index]))  # time
                pos_l.append(float(row[pos_index]))  # pos

            time_zero = time_l[0]
            for i in range(len(time_l)):
                time_l[i] -= time_zero

        return time_l, pos_l

    def compare_lists_w_time(self, list_one, time_l_one, list_two, time_l_two):
        '''
        :param list_one: Short list
        :param list_two: Long list
        :return:
        '''
        i = 0
        j = 0
        pos_one_list = []
        pos_two_list = []
        for i in range(len(list_one)):
            time_in_tile_l_one = time_l_one[i]
            pos_one_list.append(list_one[i])
            for j in range(len(time_l_two)):
                if time_in_tile_l_one + 0.01 > time_l_two[j] and time_in_tile_l_one - 0.01 < time_l_two[j]:
                    pos_two_list.append(list_two[j])
                    print("time vs time", time_in_tile_l_one, time_l_two[j])
                    break
        print(pos_one_list)
        print(pos_two_list)


        # error = [abs(i - j) / i * 100 for i, j in zip(pos_one_list, pos_two_list)]
        error = [abs(j - i) / j * 100 for i, j in zip(pos_one_list, pos_two_list)]
        # print("Len of lists", len(list_one), len(list_two))
        print("Error on points", error)

        return pos_one_list[0:7], pos_two_list[0:7]

    def run(self):
        # Load the imu data from test6
        self.load_imu_csv('vel_est_test_3_33.txt')
        self.imu_vel_l, _ = self.plot_diff(self.imu_time_l, self.imu_pos_l, self.imu_yaw_l, title='data from imu')

        # load the optitrack data from test6 that Mark has refined
        self.opti_time_l, self.opti_pos_l = self.load_opti_csv(time_index=0, pos_index=1)
        self.opti_vel_l, _ = self.plot_diff(self.opti_time_l, self.opti_pos_l, title='optitrack data')

        imu_list, opti_list = self.compare_lists_w_time(self.imu_pos_l, self.imu_time_l, self.opti_pos_l, self.opti_time_l)

        print(len(imu_list), len(opti_list))
        self.plot_pos_only(self.imu_time_l[0:7], imu_list, opti_list)


if __name__ == "__main__":
    reader = speed_from_pos_IMU()
    reader.run()

