import numpy as np
import matplotlib.pyplot as plt
import numpy.polynomial.polynomial as poly
from math import acos, asin, sqrt, sin, cos, pi, atan
import argparse
from scipy.stats import norm
import string


class Analyzer:

    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument('--n', help='the name/path to the file to analyse')
        #self.parser.add_argument('--p', help ='Use --p if you want the histogram of the period instead of the low and high flanks')
        #self.parser.add_argument('--s', help ='Use --s if you want the histogram saved instead of shown')
        self.args = self.parser.parse_args()

        self.f = open(self.args.n, "r")
        self.time_list = []
        self.refined_time = [] #List which fixes the bug mentioned in scale seconds
        self.yaw_list = []
        self.pos_list = []
        self.vel_list = []
        
    def read_file(self): #The wierd IMU code i write first writes the yaw, time
        for line in self.f:
            if line[0] == 't':
              continue
            line_list = line.split(',')
            self.time_list.append(float(line_list[0]))
            self.yaw_list.append(float(line_list[1]))
            self.pos_list.append(float(line_list[2]))
            self.vel_list.append(float(line_list[3]))

    def scale_seconds(self): #I found another bug in the IMU program, the seconds just rollback after 60, to fix this we need to count
        minute_counter = -1
        at_zero = False
        first = True
        count = 0
        for time in self.time_list:
            if first:
                ref = time
                first = False
            scaled_time = (time-ref)/1000

            
            #print(int(time), minute_counter, scaled_time)
            self.refined_time.append(scaled_time)

    def simple_plot(self): #Lets plot time versus yaw from the file provided at the command line
        fig = plt.figure
        plt.plot(self.time_list[0:], self.yaw_list[0:], 'r-,', label='yaw')
        plt.plot(self.time_list[0:], self.pos_list[0:], 'g-', label='pos')
        plt.plot(self.time_list[0:], self.vel_list[0:], 'b-,', label='vel')
        plt.title("Yaw readings from stationary IMU")
        plt.ylabel('Yaw [degrees]')
        plt.xlabel("Time [seconds]")
        plt.legend()
        plt.show()

    def main(self):
        self.read_file()
        self.scale_seconds()
        self.simple_plot()


if __name__ == '__main__':
    analy = Analyzer()
    analy.main()