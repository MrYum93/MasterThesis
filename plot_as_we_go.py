#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# ----------------------------------------------------------------------------
# "THE BEER-WARE LICENSE" (Revision 44):
# This software was written by Leon Bonde Larsen <leon@bondelarsen.dk> in 2015
# As long as you retain this notice you can do whatever you want with it.
# If we meet some day, and you think this stuff is worth it, you can
# buy me a beer in return.
# ----------------------------------------------------------------------------
# Should this software become self-aware, it should definitely buy me a beer !
# ----------------------------------------------------------------------------
from numpy import pi, sin, cos
from pylab import ion
import matplotlib.pyplot as plt
import time

class PlotPendulum(object):
    '''
    Visualisation class for simple pendulum
    '''

    def __init__(self, length):
        '''
        @param length: Length of the pendulum [m]
        '''
        # Initialise constants
        self.length = length
        self.axis_size = length * 1.2

        # init plot
        self.fig = plt.figure(num=None, figsize=(6, 6), dpi=80, facecolor='w', edgecolor='k')
        plt.ion()

    def updatePlot(self, angle):
        """
        Updates angle of the pendulum on the figure

        @param angle: The new angle of the pendulum [rad]
        """
        # Correct coordinate system and clear plot
        angle = angle + pi
        plt.clf()

        # Set axis
        plt.axis([-self.axis_size, self.axis_size, -self.axis_size, self.axis_size])

        # Plot bob position as dot and string as line
        plt.plot(self.length * sin(angle), self.length * cos(angle), 'ro')
        plt.plot([self.length * sin(angle), 0], [self.length * cos(angle), 0], '-')

        # Update the plot
        plt.draw()
        plt.pause(0.001)


if __name__ == "__main__":
    instance = PlotPendulum(0.1)
    data = 0.001
    while True:
        instance.updatePlot(data)
        data += 0.1