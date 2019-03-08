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


class PlotSystem:
    def __init__(self, min_x, max_x, min_y, max_y):
        self.fig = figure()
        ion()

        self.ax = axis([min_x - 0.3, max_x + 0.3, min_y - 1, max_y + 1])
        plt.show(block=False)
        self.plane = [0, 0, 0, 17]
        self.hooked_flag = False

    def update_plot(self, plane, left_dock, right_dock, left_rope, right_rope):
        clf()
        # yaw = int(yaw)
        axis([self.ax[0], self.ax[1], self.ax[2], self.ax[3]])

        # Here we plot what is needed
        self.plot_plane(plane[0], plane[1], plane[2], plane[3])
        self.plot_side(left_dock)
        self.plot_side(right_dock)

        if self.plane[1] >= 0:
            self.hooked_flag = True

        self.plot_rope(left_rope[0], left_rope[1])
        self.plot_rope(right_rope[0], right_rope[1])


        self.fig.canvas.flush_events()

    def plot_side(self, pos):
        plt.plot(pos[0], pos[1], 'ro')

    def plot_plane(self, pos_x, pos_y, vel_x, vel_y):
        plt.arrow(pos_x, pos_y, vel_x, vel_y, head_width=0.05, head_length=0.1)

    def plot_rope(self, anchor1, anchor2):
        plt.plot([anchor1[0], anchor2[0]], [anchor1[1], anchor2[1]], '-')

    def run(self):
        time.sleep(0.2)

        self.plane = [0, 3, 0, 17]  # pos then vel

        # ALL THE IF NEEDS TO BE PUT IN UPDATE METHOD!!!!!!!!
        left_pole = [-1, 0]
        right_pole = [1, 0]
        if self.hooked_flag is False:
            left_rope = [[0, 0], left_pole]
            right_rope = [[0, 0], right_pole]

        if self.hooked_flag is True:
            left_rope = [[self.plane[0], self.plane[1]], left_pole]
            right_rope = [[self.plane[0], self.plane[1]], right_pole]
        # print("left pole", left_rope)
        # right_rope =

        while True:
            self.update_plot(self.plane, left_pole, right_pole, left_rope, right_rope)

            pause(0.00001)


if __name__ == '__main__':
    plot = PlotSystem(-3, 3, -20, 20)
    plot.run()
