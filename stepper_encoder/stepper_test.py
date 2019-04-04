import RPi.GPIO as GPIO 
import time
import datetime

freq = 100.0  # in Hz
print("freq: ", freq)
GPIO.setmode(GPIO.BCM) #This does so that we can acces the GPIO by their GPIO number
GPIO.setwarnings(False)

GPIO.setup(15, GPIO.OUT) #Enable
GPIO.setup(18, GPIO.OUT) #Direction
GPIO.setup(17, GPIO.OUT) #Step

GPIO.output(15, True)
GPIO.output(18, True)

pulse_cnt = 0
faster_cnt = 0

t = time.time()

file = open("/home/pi/stepper_encoder_test/stepper_data.txt", "w")
file.write("time, freq")
while True:
    t = str(datetime.datetime.now().time())
    hours, min, sec = t.split(':')
    to_write = sec + ", " + str(freq) + '\n'
    file.write(to_write)
    GPIO.output(17, True)
    time.sleep(1/(freq*2))
    GPIO.output(17, False)
    time.sleep(1/(freq*2))

    if faster_cnt >= 500:
        freq += 100.0
        print('FASTER with freq: ', freq)
        faster_cnt = 0
    
    faster_cnt += 1


