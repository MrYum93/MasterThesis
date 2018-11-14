import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import math

time = 0
plane_init_speed = 17.0
plane_speed = 17.0
plane_weight = 2.3
spring_constant = 42
plane_speed_list = []
time_list = []
displacement = 0
init_plane_e = 0
inner_spring_arm = 0.105
outer_spring_arm = 0.2165
half_distance_arms = 1
plane_momentum_list = []
plane_momentum = plane_weight * plane_init_speed
no_springs = 2

def spring_energy(displacement):
    return 0.5 * spring_constant * (displacement) ** 2


def spring_force(displacement):
    return -0.5 * spring_constant * (displacement) ** 2


def plane_energy(plane_speed):
    return 0.5 * plane_weight * (plane_speed) ** 2


def angle_to_spring_stretch(angle):
    return math.sqrt((inner_spring_arm * math.sin(angle)) ** 2 + (
                -inner_spring_arm * math.cos(angle) + outer_spring_arm) ** 2)


def displacement_to_angle(displacement):
    return math.atan(displacement / half_distance_arms)

def run_simu():
    while time < 1: #m5del.plane_momentum > 1:
        delta_time = 0.01
        delta_displacement = plane_speed * delta_time

        displacement += delta_displacement
        time += delta_time
        # .displacement += 0.01*.plane_speed

        current_angle = displacement_to_angle(displacement)

        spring_displacement = angle_to_spring_stretch(current_angle)

        F_spring = spring_force(spring_displacement)

        delta_momentum = F_spring*delta_time

        plane_momentum += delta_momentum


        plane_momentum_list.append(plane_momentum)
        time_list.append(time)



fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.25)
t = np.arange(0.0, 1.0, 0.001)
a0 = 5
f0 = 3
delta_f = 5.0

plt.axis([0, 1, -10, 10])

axcolor = 'lightgoldenrodyellow'
#axfreq = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
#axamp = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)

s_springs = Slider(axfreq, 'Springs', 2, 30.0, valinit=2, valstep=2)
#samp = Slider(axamp, 'Amp', 0.1, 10.0, valinit=1)


def update(val):
    amp = samp.val
    freq = sfreq.val
    l.set_ydata(amp*np.sin(2*np.pi*freq*t))
    fig.canvas.draw_idle()
sfreq.on_changed(update)
samp.on_changed(update)

resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')


def reset(event):
    sfreq.reset()
    samp.reset()
button.on_clicked(reset)

rax = plt.axes([0.025, 0.5, 0.15, 0.15], facecolor=axcolor)
radio = RadioButtons(rax, ('red', 'blue', 'green'), active=0)


def colorfunc(label):
    l.set_color(label)
    fig.canvas.draw_idle()
radio.on_clicked(colorfunc)



plane_momentum_list = []



plt.show()