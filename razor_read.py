#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
from serial import *
import time
from struct import *

import io

#Setting up serial communication
try:
    ser = Serial('/dev/ttyUSB0', 57600)  # open serial port
except SerialException:
    ser = Serial('/dev/ttyUSB1', 57600)
time.sleep(3) #This is to make sure that the razor is up and running, there is a faster way using the guide https://github.com/Razor-AHRS/razor-9dof-ahrs/wiki/Tutorial#writing-your-own-code-to-read-from-the-tracker
print(ser.name)         # check which port was really used
ser.timeout = 1


ser.write("#ob") # Turn on binary output
ser.write("#o1") # Turn on continuous streaming output
ser.write("#oe0") # Disable error message output
ser.flush()
ser.write("#s00") #Request synch token

#print("Is the serial port open", ser.is_open)
# it is buffering. required to get the data out *now*
#yaw = 0
#pitch = 0
#roll = 0
#Der er 3 læsninger per YPR, måske bare joine dem i en string og så unpack?

#def bits_to_float(b):
#    #s = pack('<1', b)
#    return unpack('>f', b)[0]
data_array = []
counter = 0
line = ser.readline()
while(True):
    inc = ser.read()
    data_array.append(inc)
    if counter == 40:
        print(data_array)
        data_array = []
        counter = 0
        #ser.write("#s00")
    counter += 1

#Lets try to read the data until the same token is found again
reading_counter = 0


ser.readline() #This should empty the synch and abit after in the buffer


while True:


    byte0 = ser.read()
    byte1 = ser.read()
    byte2 = ser.read()
    byte3 = ser.read()

    data0 = unpack("<b", byte0)
    data1 = unpack("<b", byte1)
    data2 = unpack("<b", byte2)
    data3 = unpack("<b", byte3)

    #unpacked = unpack("<B", data)
    reading = data0 + data1 + data2 + data3
    if reading_counter == 0:
        print("The yaw value is", reading)
    if reading_counter == 1:
        print("The pitch value is", reading)
        dummy = 0
    if reading_counter == 2:
        print("The roll value is", reading, '\n')
        dummy = 1
    reading_counter += 1
    if reading_counter > 2:
        reading_counter = 0





ser.close()             # close port




def find_synch():

    #To find the sync
    ser.readline()
    #sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
    #test = sio.readline()
    #print(test)
    synch_token = "#SYNCH00\r\n"
    while(True):
        chunk = ser.read(1)
        print("chunk", chunk)
        hr_chunk = unpack("<b",chunk)
        print(hr_chunk)


data_array = []
counter = 0





#while True:
#    ser.read()
#
#    print(bits_to_float(ser.read()))




#Method to read from the



#while True: #The processor little indean to big indean java reading syntax
#    float = ser.read() + (ser.read() <<8 ) + (ser.read() <<16) +( ser.read() <<24)
#    print(float)
#reading_counter = 0


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