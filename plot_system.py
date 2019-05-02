#!/usr/bin/env python
# /***************************************************************************
# RoVi1 scrip for all exersice Robotcs part
# Copyright (c) 2018, Mathias W. Madsen <matam14@student.sdu.dk> <mwittenm@gmail.com>
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

# defines

# msg ids

# topics

# messages

#!/usr/bin/env python
# /***************************************************************************
# RoVi1 scrip for all exersice Robotcs part
# Copyright (c) 2018, Mathias W. Madsen <matam14@student.sdu.dk> <mwittenm@gmail.com>
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
from matplotlib.pyplot import figure, clf, axis, plot, draw, show, pause, subplots
import matplotlib.pyplot as plt
import sys
import time
import math

# defines
EPS = sys.float_info.epsilon

# msg ids

# topics

# messages


class PlotSystem(object):
    def __init__(self, min_x, max_x, min_y, max_y):
        self.fig = figure()
        ion()

        self.ax = axis([min_x, max_x, min_y, max_y])
        plt.show(block=False)
        self.plane = [0, 0, 0, 17]
        self.left_theta = 0
        self.right_theta = math.pi
        self.hooked_flag = False

    def update_plot(self, plane, left_pole, right_pole, left_rope, right_rope, left_spring, right_spring, extra_vec):
        clf()
        plt.grid(True)
        # yaw = int(yaw)
        axis([self.ax[0], self.ax[1], self.ax[2], self.ax[3]])

        # Here we plot what is needed
        self.plot_plane(plane[0], plane[1], plane[2], plane[3])

        if self.plane[1] >= 0 and self.hooked_flag is False:
            self.hooked_flag = True

        if self.hooked_flag is False:
            left_rope = [left_spring[1], [0, 0]]
            right_rope = [right_spring[1], [0, 0]]
        else:
            left_rope = left_rope
            right_rope = right_rope

        self.plot_rope(left_rope[0], left_rope[1])
        self.plot_rope(right_rope[0], right_rope[1])
        # print("left spring", left_spring[0])
        self.plot_torsion_spring(left_spring[0], left_spring[1])
        self.plot_torsion_spring(right_spring[0], right_spring[1])
        self.plot_extra_vector(extra_vec[0], extra_vec[1], extra_vec[2], extra_vec[3])

        self.fig.canvas.flush_events()      

    def plot_plane(self, pos_x, pos_y, vel_x, vel_y):
        plt.arrow(pos_x, pos_y, vel_x, vel_y, head_width=0.05, head_length=0.1)

    def plot_extra_vector(self, pos_x, pos_y, mag_x, mag_y):
        plt.arrow(pos_x, pos_y, mag_x, mag_y, head_width=0.1, head_length=0.1)

    def plot_rope(self, anchor1, anchor2):
        plt.plot([anchor1[0], anchor2[0]], [anchor1[1], anchor2[1]], 'r-')

    def plot_torsion_spring(self, center, end_pos):
        plt.plot(center[0], center[1], 'b,')
        plt.plot([center[0], end_pos[0]], [center[1], end_pos[1]], 'k-')

    def end_point_of_spring(self, center, rad, theta):
        end_point = [0, 0]
        end_point[0] = (math.cos(theta)) * rad + center[0]
        end_point[1] = (math.sin(theta)) * rad + center[1]
        return end_point

    def close_plot(self):
        plt.close()

    def run(self):
        '''
        This method is to simulate the real method. So only give what should be plotted nothing else
        :return:
        '''
        time.sleep(0.05)

        self.plane = [0, -5, 0, 17]  # pos then vel
        delta_theta = math.pi/16

        # print("left pole", left_rope)
        # right_rope =
        while True:
            self.plane[1] += 0.4
            if self.plane[1] >= 0:
                self.left_theta += delta_theta
                self.right_theta -= delta_theta

            left_pole = [-1, 0]
            right_pole = [1, 0]
            left_spring = [left_pole, self.end_point_of_spring([-1, 0], 0.5, self.left_theta)]  # center, end_pos
            right_spring = [right_pole, self.end_point_of_spring([1, 0], 0.5, self.right_theta)]
            left_rope = [left_spring[1], [self.plane[0], self.plane[1]]]
            right_rope = [right_spring[1], [self.plane[0], self.plane[1]]]
            self.update_plot(self.plane,
                             left_pole, right_pole,
                             left_rope, right_rope,
                             left_spring, right_spring)

            pause(0.00001)


if __name__ == '__main__':
    plot = PlotSystem(-3, 3, -20, 20)
    plot.run()
