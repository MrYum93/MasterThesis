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
        self.encoder_stepper = pd.read_csv("../data/encoder_reads.txt")
        self.total_tics = 1024
        self.tick_deg = 360/1024

    def run(self):
        # print(self.encoder_stepper)
        # index = self.encoder_stepper.columns
        # print(index)
        time = self.encoder_stepper['time']
        tics = self.encoder_stepper['tics']
        speed_l = []
        time_l = []
        for i in range(int((len(time)-1))):
            delta_time = time[i+1] - time[i]
            delta_deg = tics[i+1] - tics[i]
            tics_speed = delta_deg/delta_time
            # if tics_speed > 0.0:
            speed_l.append(tics[i]) 
            time_l.append(time[i])
        fig = plt.figure()
        plt.plot(time_l, speed_l, '-')
        plt.show()

if __name__ == "__main__":
    speed = SpeedOfMotor()
    speed.run()
