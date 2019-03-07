#!/usr/bin/env python2
# -*- coding: utf-8 -*-


import datetime
import io
import string

class log_yaw(object):
    def __init__(self):
        #This model assumes that the plane has an initial speed and is all ready hooked to the springs at t=0
        #print(datetime.datetime.now())
        self.date = datetime.datetime.now().date()
        self.yaw_counter = 0
        date = datetime.datetime.now()
        name = str(date)+".txt"
        self.file = open(name, "w+")
        #self.open_file(self)

    #En metode der bliver kaldt hvor der bliver passet en float

    #Hvis der er gået mere end 1 minut laver den en ny fil med nuværende datetime og appender i istedet

    def verify(self):
        print("date", self.date)
        print("time", self.init_time)

    def append_float(self, yaw_value):
	time = str(datetime.datetime.now().time())
	hours, minutes, seconds = time.split(':')
        if self.yaw_counter <= 10000:
            self.file.write(str(yaw_value) +", " + seconds+ "\n")
            self.yaw_counter += 1
        else:
            date = datetime.datetime.now()
            name = str(date) + ".txt"
            self.file = open(name, "w+")
            self.file.write(str(yaw_value) + ", " + seconds + "\n")
            self.yaw_counter = 0

    def open_file(self):
        name = str(datetime.datetime.now())
        self.file = open(name, "w+")


if __name__ == "__main__":
    instance = log_yaw()
    data = 0.001
    while True:
	time = str(datetime.datetime.now().time())
	hours, minutes, seconds = time.split(':')
	print(seconds)
        instance.append_float(data)
        data += 0.001

