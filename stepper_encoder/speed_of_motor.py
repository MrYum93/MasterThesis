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
This is used to calc the rotational speed of the motor from csv files

NOT YET: Rest of lectures...

Revision
YYYY-MM-DD
2018-04-02 MW First version
'''

#import
import csv
import pandas as pd
import matplotlib.pyplot as plt


class SpeedOfMotor():
    def __init__(self):
        self.encoder_stepper = pd.read_csv("../data_for_git/encoder_data.txt")
        self.total_tics = 1024
        self.tick_deg = 360/1024

    def run(self):
        # print(self.encoder_stepper)
        # index = self.encoder_stepper.columns
        # print(index)
        time = self.encoder_stepper['time']
        tics = self.encoder_stepper['tics']
        rev = 0
        tic_i = 0
        tmp_tic = 0
        time_now = 0
        time_old = 0
        deg_now = 0
        deg_old = 0
        speed_l = []
        time_l = []
        rev_l = []
        for i in range(len(time[94500:-1])):# range(int((len(time)-1))):
            index = i# + 94500

            tic_i = tics[index] - tmp_tic
            if tic_i >= 1024:
                tmp_tic = tics[index]
                time_now = time[index]
                deg_now = tics[index]

                delta_time = time_old - time_now
                delta_deg = deg_old - deg_now
                tics_speed = delta_deg/delta_time
                # print("time", time[index])
                # print("tics speed", tics_speed)# tics[index])
                print("speed", tics_speed)

                time_old = time_now
                deg_old = deg_now

                speed_l.append(tics_speed)#tics[index])
                time_l.append(time[index])

        fig = plt.figure()
        plt.xlabel('time')
        plt.ylabel('speed')
        plt.plot(time_l, speed_l, '-')
        plt.show()

if __name__ == "__main__":
    speed = SpeedOfMotor()
    speed.run()
