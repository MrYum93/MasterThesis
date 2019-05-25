import numpy as np
import csv
from os import listdir
from os.path import isfile, join
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import itertools
from matplotlib.ticker import FormatStrFormatter
import math


start_rad = 0
end_rad = 100*math.pi/180

k = 1
theta = start_rad
distance_to_cyl = 0.3
cyl_r = 0.1


def calc_end_point(theta, cyl_r):
    x = math.cos(theta)
    y = math.sin(theta)
    
    dis = (((x-3)**2+y**2)**0.5)*3
    print("x, y", x, y, dis)
    return x, y, dis

dis_l = []
thetha_l = []
tor_f = []
start_d = 3
while theta < end_rad:
    x, y, dis = calc_end_point(theta, cyl_r)
    x_dis = 3-x
    y_dis = y
    spring_dis = (x_dis**2-y_dis**2)**0.5
    dis_l.append(spring_dis*3)
    tor_f.append(theta*3)
    thetha_l.append(theta)
    theta += end_rad/100

fig = plt.figure
plt.plot(thetha_l, dis_l)
plt.plot(thetha_l, tor_f)
title = "Force of tension and torsion springs" 
plt.ylabel('Force [Nm]')
plt.xlabel("Arm angle [radians]")
plt.show()