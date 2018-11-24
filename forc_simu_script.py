import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import math

class ar_model(object):
    def __init__(self):
        self.time = 0
        self.plane_init_speed = 17.0
        self.plane_speed = 17.0
        self.plane_weight = 2.3
        self.spring_constant = 42
        self.plane_speed_list = []
        self.time_list = []
        self.displacement = 0
        self.init_plane_e = 0
        self.inner_spring_arm = 0.105
        self.outer_spring_arm = 0.2165
        self.pulley_arm = 0.3 #FIND THE RIGHT ONE
        self.half_distance_arms = 1
        self.plane_momentum = self.plane_weight * self.plane_init_speed
        self.plane_momentum_list = []
        self.no_springs = 2
        self.rope_pos = 0
        self.plane_momentum_list = []
        self.plane_speed_list = []
        self.time_list = []
        self.angle_l = []
        self.rope_speed_list = []
        self.arm_rot_vel_l = []
        self.arm_pos_l = []
        self.instant_a_l = []
        """time = 0
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
no_springs = 2"""

    def reset_model(self):
        self.time = 0
        self.plane_init_speed = 17.0
        self.plane_speed = 17.0
        self.plane_weight = 2.3
        self.spring_constant = 42
        self.displacement = 0
        self.init_plane_e = 0
        self.inner_spring_arm = 0.105
        self.outer_spring_arm = 0.2165
        self.pulley_arm = 0.3  # FIND THE RIGHT ONE
        self.half_distance_arms = 1
        self.plane_momentum = self.plane_weight * self.plane_init_speed
        self.no_springs = 2
        self.rope_pos = 0
        self.plane_momentum_list = []
        self.plane_speed_list = []
        self.time_list = []

        self.rope_speed_list = []
        self.arm_rot_vel_l = []
        self.arm_pos_l = []
        self.instant_a_l = []
        self.angle_l = []

    def update_lists(self):
        self.plane_momentum_list.append(self.plane_momentum)
        self.plane_speed_list.append(self.plane_speed)
        self.time_list.append(self.time)
        self.rope_pos_list.append(self.rope_pos)


    def spring_energy(self, displacement):
        return 0.5 * self.spring_constant * (displacement) ** 2


    def spring_force(self, displacement):
        return -0.5 * self.spring_constant * (displacement) ** 2


    def plane_energy(self, plane_speed):
        return 0.5 * self.plane_weight * (plane_speed) ** 2


    def angle_to_spring_stretch(self, angle):
        return math.sqrt((self.inner_spring_arm * math.sin(angle)) ** 2 + (
                    -self.inner_spring_arm * math.cos(angle) + self.outer_spring_arm) ** 2)


    def displacement_to_angle(self, displacement):
        if displacement < 0.21213203435:
            return math.asin(displacement / self.pulley_arm)
        else:
            print "At else"
            return math.asin((displacement - 0.21213203435) / self.pulley_arm)+math.pi/4

    def rope_pos_to_vel_acc(self, time_list, rope_pos_list):
        x = time_list
        y = rope_pos_list
        dy = np.zeros(y.__len__(), np.float)
        dyy = np.zeros(y.__len__(), np.float)
        dy[0:-1] = np.diff(y) / np.diff(x)
        dyy[0:-1] = np.diff(dy) / np.diff(x)
        dy[-1] = (y[-1] - y[-2]) / (x[-1] - x[-2])
        dyy[-1] = (dy[-1] - dy[-2]) / (x[-1] - x[-2])

        plt.subplot(3, 1, 1)
        plt.plot(x, y, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Rope len', fontsize=16)

        plt.subplot(3, 1, 2)
        plt.plot(x, dy, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Rope vel', fontsize=16)

        plt.subplot(3, 1, 3)
        plt.plot(x, dyy, 'r')
        plt.xlabel('Time since hooking', fontsize=18)
        plt.ylabel('Rope acc', fontsize=16)

        plt.show()

    def run_simu(self):
        while self.time < 5:
            delta_time = 0.001
            delta_displacement = self.plane_speed * delta_time


            self.displacement += delta_displacement
            self.time += delta_time
            # .displacement += 0.01*.plane_speed
            current_angle = self.displacement_to_angle(self.displacement)
            print current_angle
            self.angle_l.append(current_angle)
            spring_displacement = self.angle_to_spring_stretch(current_angle)

            F_spring = self.no_springs*0.5*self.spring_force(spring_displacement)
            instant_a = F_spring/self.plane_weight*0.5
            delta_momentum = F_spring*delta_time

            self.plane_momentum += delta_momentum

            self.plane_momentum_list.append(self.plane_momentum)
            self.time_list.append(self.time)
        #self.rope_pos_to_vel_acc(self.time_list, self.angle_l)

def main():

    model = ar_model()

    fig = plt.subplots()
    plt.subplots_adjust(left=0.25, bottom=0.25)
    t = np.arange(0.0, 1.0, 0.001)

    model.run_simu()
    l, = plt.plot(model.time_list, model.plane_momentum_list, lw=2, color='red')
    plt.axis([0, 5, -1, 50])

    axcolor = 'lightgoldenrodyellow'
    ax_spring = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
    #ax_x = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)

    s_springs = Slider(ax_spring, 'Springs', 2, 30.0, valinit=2, valstep=2)
    #samp = Slider(axamp, 'Amp', 0.1, 10.0, valinit=1)


    def update(val):
        model.reset_model()
        model.no_springs = s_springs.val
        #freq = sfreq.val
        model.run_simu()
        l.set_ydata(model.plane_momentum_list)
        #fig.plot(model.time_list, model.plane_momentum_list, lw=2, color='red')
        #print("Actually went here", model.no_springs)
        #fig.canvas.draw_idle()
        #fig.draw()

    s_springs.on_changed(update)
    #samp.on_changed(update)

    #resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
    #button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')


    def reset(event):
        s_springs.reset()
        #samp.reset()
    #button.on_clicked(reset)

    #rax = plt.axes([0.025, 0.5, 0.15, 0.15], facecolor=axcolor)









    plt.show()


if __name__ == "__main__":
    main()

