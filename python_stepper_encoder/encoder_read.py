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
This script function to read the encoder data from a E6B2-CWZ3E

NOT YET:

Revision
YYYY-MM-DD
2018-01-04 MW First version
'''

#IMPORTS
import time
import RPi.GPIO as GPIO
import datetime

#DEFINE
PHASE_A = 16
PHASE_B = 19
PHASE_Z = 13

class ReadEncoder:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PHASE_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # setting a pull down resistor
        GPIO.setup(PHASE_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(PHASE_Z, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(20, GPIO.OUT)
        GPIO.add_event_detect(PHASE_A, GPIO.BOTH, callback=self.a_call)  # add rising edge detection on a channel
        GPIO.add_event_detect(PHASE_B, GPIO.BOTH, callback=self.b_call)
        GPIO.add_event_detect(PHASE_Z, GPIO.BOTH, callback=self.z_call)  # add rising edge d$
        #GPIO.add_event_detect(PHASE_A, GPIO.FALLING, callback=self.a_falling_call)  # add rising edge d$
        #GPIO.add_event_detect(PHASE_B, GPIO.FALLING, callback=self.b_falling_call)
        #GPIO.add_event_detect(PHASE_Z, GPIO.FALLING, callback=self.z_falling_call)
        self.tics = 0
        self.diff_tics = 0
        self.seq = 0
        self.old_seq = 0
        self.rotations = 0
        self.a_h = False
        self.b_h = False
        self.z_h = False
        self.t_old = 0
        self.file = open("/home/pi/stepper_encoder_test/encoder_data.txt", "w")
	self.file.write("time, tics")

    def a_call(self, channel):
        if GPIO.input(PHASE_A):   
            print "Rising edge detected on A"
            self.a_h = True  
        else:                    
            print "Falling edge detected on A"          
            self.a_h = False

    def b_call(self, channel):
        if GPIO.input(PHASE_B):  
            print "Rising edge detected on B" 
            self.b_h = True
        else:                   
            print "Falling edge detected on B"
            self.b_h = False

    def z_call(self, channel):
        if GPIO.input(PHASE_Z):  
            print "Rising edge detected on Z" 
            self.z_h = True
        else:                   
            print "Falling edge detected on Z"
            self.z_h = False
	    self.rotations += self.diff_tics

    def a_rising_call(self, channel):
        print('phase a rising')
        self.a_h = True

    def b_rising_call(self, channel):
        print('phase b rising')
        self.b_h = True

    def z_rising_call(self, channel):
        print('phase z rising')
        self.z_h = True


    def read(self):
        while(True):
	    t = str(datetime.datetime.now().time())
	    hours, min, sec = t.split(':')
            #print('time', t_now - self.t_old)
            self.seq = (self.a_h ^ self.b_h) | self.b_h << 1  # xor & or
            delta = (self.seq - self.old_seq) % 4
            #print("delta", delta)
	    if delta is 0:
                self.tics = self.tics
            elif delta is 1:
                self.tics += 1
                self.diff_tics = 1
            elif delta is 2:
                self.tics += self.diff_tics*2
            elif delta is 3:
                self.tics -= 1
                self.diff_tics = -1
            self.old_seq = self.seq

	    #print('a_h', self.a_h)
            #print('rotations:', self.rotations)
            print('tics:', self.tics / 4)
	    to_write = sec + ', ' +  str(self.tics/4) + '\n'
	    self.file.write(to_write)
            #if self.a_h is True and 
            #self.t_old = t_now
        self.file.close()


if __name__ == "__main__":
    read = ReadEncoder()
    read.read()
