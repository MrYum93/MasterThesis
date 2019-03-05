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
time = 0
yaw_l = []  # np.array(np.zeros(20))
pitch_l = np.array(np.zeros(20))
roll_l = np.array(np.zeros(20))
time_l = []  # np.array(np.zeros(20))

style.use('fivethirtyeight')
# fig, ax = plt.subplots()
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)


def animate(i):
    xs = time_l
    ys = yaw_l
    # xs.append(time)
    # ys.append(yaw)
    print("xs", xs)
    print("ys", ys)
    print("time", time_l)
    print("yaw", yaw_l)
    print('')
    ax.clear()
    ax.plot(xs, ys)
    return ax

def updatePlot(time_l_, yaw_l_):
    # clear the current plot
    plt.clf()
    # print("time", time_l_)
    # print("yaw", yaw_l_)

    # Plot bob position as dot and string as line
    plt.plot(time_l_, yaw_l_, 'ro')

    # Update the plot
    plt.draw()
    # plt.show()


plot = PlotRPY(time, yaw)

ser.readline()  # This solves the sync problems...

while True:

    if reading_counter == 0:
        byte0 = ser.read()
        byte1 = ser.read()
        byte2 = ser.read()
        byte3 = ser.read()

        dataFloat = unpack('f', byte0 + byte1 + byte2 + byte3)
        # print("The yaw value is", dataFloat[0])

        time_l.append(time)
        yaw_l.append(dataFloat[0])
        # print("time", time)
        # print("datafloat", dataFloat)
        # print("time_l", time_l)
        # print("yaw_l", yaw_l)
        # updatePlot(time_l, yaw_l)

        # ani = animation.FuncAnimation(fig, animate, interval=10, blit=True repeat=False)
        print("time vs yaw", time, dataFloat[0])
        # plt.plot(time, dataFloat[0], 'ro')
        # plt.show()

        # plt.clf()
        plot.update_plot(time, dataFloat[0])

        pass
    if reading_counter == 1:
        byte0 = ser.read()
        byte1 = ser.read()
        byte2 = ser.read()
        byte3 = ser.read()

        dataFloat = unpack('f', byte0 + byte1 + byte2 + byte3)

        print("The pitch value is", dataFloat)
        dummy = 0
    if reading_counter == 2:
        byte0 = ser.read()
        byte1 = ser.read()
        byte2 = ser.read()
        byte3 = ser.read()

        dataFloat = unpack('f', byte0 + byte1 + byte2 + byte3)

        print("The roll value is", dataFloat)
        dummy = 1
    reading_counter += 1
    if reading_counter > 2:
        reading_counter = 0

    time += 1
    print('')


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