#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
# from serial import *
import serial
import time
from struct import *
import numpy as np
import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from matplotlib import style
from pylab import ion
from plot_rpy import PlotRPY
from save_file import *
ser = serial.Serial('/dev/ttyUSB0', 57600)
# try:
#     ser = serial.Serial('/dev/ttyUSB0', 57600)  # open serial port
# except serial.SerialException:
#     ser = serial.Serial('/dev/ttyUSB1', 57600)
time.sleep(3)   # This is to make sure that the razor is up and running, there is a faster way using the guide
        # https://github.com/Razor-AHRS/razor-9dof-ahrs/wiki/Tutorial#writing-your-own-code-to-read-from-the-tracker
print(ser.name)         # check which port was really used
ser.timeout = 1

ser.write("#ob") # Turn on binary output
ser.write("#o1") # Turn on continuous streaming output
ser.write("#oe0") # Disable error message output
ser.flush()
ser.write("#s00") #Request synch token

#print("Is the serial port open", ser.is_open)
# it is buffering. required to get the data out *now*
yaw = 0
pitch = 0
roll = 0
#Der er 3 læsninger per YPR, måske bare joine dem i en string og så unpack?

# while True: #The processor little indean to big indean java reading syntax
#     float = ser.read() + (ser.read() <<8 ) + (ser.read() <<16) +( ser.read() <<24)
#     print(float)

reading_counter = 0
array_cnt = 0
timer = 0
yaw_l = []  # np.array(np.zeros(20))
pitch_l = []  # np.array(np.zeros(20))
roll_l = []  # np.array(np.zeros(20))
timer_l = []  # np.array(np.zeros(20))

style.use('fivethirtyeight')
# fig, ax = plt.subplots()
# fig = plt.figure()
# ax = fig.add_subplot(1, 1, 1)


# def animate(i):
#     xs = timer_l
#     ys = yaw_l
#     # xs.append(timer)
#     # ys.append(yaw)
#     print("xs", xs)
#     print("ys", ys)
#     print("timer", timer_l)
#     print("yaw", yaw_l)
#     print('')
#     ax.clear()
#     ax.plot(xs, ys)
#     return ax

#plot = PlotRPY(timer, yaw)

ser.readline()  # This solves the sync problems...
filesaver = log_yaw()
plot_cnt = 0
while True:
    yaw0 = ser.read()
    yaw1 = ser.read()
    yaw2 = ser.read()
    yaw3 = ser.read()
    yaw_float = unpack('f', yaw0 + yaw1 + yaw2 + yaw3)
    pitch0 = ser.read()
    pitch1 = ser.read()
    pitch2 = ser.read()
    pitch3 = ser.read()
    pitch_float = unpack('f', pitch0 + pitch1 + pitch2 + pitch3)
    roll0 = ser.read()
    roll1 = ser.read()
    roll2 = ser.read()
    roll3 = ser.read()
    roll_float = unpack('f', roll0 + roll1 + roll2 + roll3)

    timer_l.append(timer)
    yaw_l.append(yaw_float[0])
    pitch_l.append(pitch_float[0])
    roll_l.append(roll_float[0])
    filesaver.append_float(yaw_float[0])
    print("timer vs yaw", timer, yaw_float[0])
    print("timer vs pitch", timer, pitch_float[0])
    print("timer vs roll", timer, roll_float[0])

    if plot_cnt >= 5:
        # plot.update_plot(timer, yaw_float[0])
        plot.update_plot(timer_l[-100:], yaw_l[-100:], pitch_l[-100:], roll_l[-100:])
        # print("should be 20 long", timer_l[-20:])
        plot_cnt = 0


    # if reading_counter == 0:
    #     byte0 = ser.read()
    #     byte1 = ser.read()
    #     byte2 = ser.read()
    #     byte3 = ser.read()
    #
    #     dataFloat = unpack('f', byte0 + byte1 + byte2 + byte3)
    #     # print("The yaw value is", dataFloat[0])
    #
    #     timer_l.append(timer)
    #     yaw_l.append(dataFloat[0])
    #     # print("timer", timer)
    #     # print("datafloat", dataFloat)
    #     # print("timer_l", timer_l)
    #     # print("yaw_l", yaw_l)
    #     # updatePlot(timer_l, yaw_l)
    #
    #     # ani = animation.FuncAnimation(fig, animate, interval=10, blit=True repeat=False)
    #     print("timer vs yaw", timer, dataFloat[0])
    #     # plt.plot(timer, dataFloat[0], 'ro')
    #     # plt.show()
    #
    #     # plt.clf()
    #     plot.update_plot(timer, dataFloat[0])
    #
    #     pass
    # if reading_counter == 1:
    #     byte0 = ser.read()
    #     byte1 = ser.read()
    #     byte2 = ser.read()
    #     byte3 = ser.read()
    #
    #     dataFloat = unpack('f', byte0 + byte1 + byte2 + byte3)
    #
    #     print("The pitch value is", dataFloat)
    #     dummy = 0
    # if reading_counter == 2:
    #     byte0 = ser.read()
    #     byte1 = ser.read()
    #     byte2 = ser.read()
    #     byte3 = ser.read()
    #
    #     dataFloat = unpack('f', byte0 + byte1 + byte2 + byte3)
    #
    #     print("The roll value is", dataFloat)
    #     dummy = 1
    # reading_counter += 1
    # if reading_counter > 2:
    #     reading_counter = 0

    plot_cnt += 1
    timer += 1
    print('')
    # time.sleep(0.1)


#Lets try to read the data until the same token is found again
data_array = []
while(True):
    inc = ser.read()
    data_array.append(inc)
    # if data_array[0] == inc

# ser.close()             # close port






#Bus 001 Device 006: ID 0403:6001 Future Technology Devices International, Ltd FT232 USB-Serial (UART) IC


#def ev3_write(command):
    # To send commands, we need an Endpoint.

    # To get to the endpoint we need to descend down the hierarchy of
    # 1. Device
 #   VENDOR_LEGO = 0x0694
 #   PRODUCT_EV3 = 5
    # 2. Configuration
  #  CONFIGURATION_EV3 = 1       # 1-based
    # 3. Interface
   # INTERFACE_EV3 = 0           # 0-based
    # 4. Alternate setting
   # SETTING_EV3 = 0             # 0-based
    # 5. Endpoint
   # ENDPOINT_EV3 = 1            # 0-based

    # 1. Device
   # device = usb.core.find(idVendor=VENDOR_LEGO, idProduct=PRODUCT_EV3)
    #if device is None:
    #    print("Is the brick connected and turned on?")
    #    sys.exit(1)

    # By default, the kernel will claim the device and make it available via
    # /dev/usb/hiddevN and /dev/hidrawN which also prevents us
    # from communicating otherwise. This removes these kernel devices.
    # Yes, it is weird to specify an interface before we get to a configuration.
    #if device.is_kernel_driver_active(INTERFACE_EV3):
    #    print("Detaching kernel driver")
    #    device.detach_kernel_driver(INTERFACE_EV3)

    # 2. Configuration
    # A Device may have multiple Configurations, and only one can be active at
    # a time. Most devices have only one. Supporting multiple Configurations
    # is reportedly useful for offering more/less features when more/less
    # power is available.
    ## Because multiple configs are rare, the library allows to omit this:
    ## device.set_configuration(CONFIGURATION_EV3)
    #configuration = device.get_active_configuration()

    # 3. Interface
    # A physical Device may have multiple Interfaces active at a time.
    # A typical example is a scanner-printer combo.
    #
    # 4. Alternate setting
    # I don't quite understand this, but they say that if you need Isochronous
    # Endpoints (read: audio or video), you must go to a non-primary
    # Alternate Setting.
    #interface = configuration[(INTERFACE_EV3, SETTING_EV3)]

    # 5. Endpoint
    # The Endpoint 0 is reserved for control functions
    # so we use Endpoint 1 here.
    # If an Interface uses multiple Endpoints, they will differ
    # in transfer modes:
    # - Interrupt transfers (keyboard): data arrives soon, with error detection
    # - Isochronous transfers (camera): data arrives on time, or gets lost
    # - Bulk transfers (printer): all data arrives, sooner or later
    #endpoint = interface[ENDPOINT_EV3]

    # Finally!
    #endpoint.write(command)

#beep_command = \
#    '\x0F\x00\x01\x00\x80\x00\x00\x94\x01\x81\x02\x82\xE8\x03\x82\xE8\x03'
#ev3_write(beep_command)