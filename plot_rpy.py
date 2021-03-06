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


# imports
import numpy as np
from pylab import ion
# import matplotlib
# matplotlib.use('Qt4Agg')
from matplotlib.pyplot import figure, clf, axis, plot, draw, show, pause, subplots
import matplotlib.pyplot as plt
import sys
import time

# defines
EPS = sys.float_info.epsilon

# msg ids

# topics

# messages

class PlotRPY(object):
    def __init__(self, time, yaw):
        # self.fig = figure(num=None, figsize=(6,6), dpi=80, facecolor='w', edgecolor='k')
        self.fig, self.ax = plt.subplots()
        # ion()
        # plt.show(block=False)

    def update_plot(self, time_, yaw, pitch, roll):
        # clf()
        # yaw = int(yaw)
        axis([time_[0], time_[-1], -180, 180])

        # self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        plot(time_, yaw, '-')
        plot(time_, pitch, '-')
        plot(time_, roll, '-')

        pause(0.00001)
