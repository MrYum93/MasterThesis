#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
from bokeh.io import curdoc, show, output_file
from bokeh.layouts import row, widgetbox, layout, gridplot
from bokeh.models import ColumnDataSource, CustomJS
from bokeh.models.widgets import Slider, TextInput
from bokeh.plotting import figure

class simple_model(object):
    def __init__(self):
        #This model assumes that the plane has an initial speed and is all ready hooked to the springs at t=0
        self.time = 0
        self.delta_t = 0.001
        self.l_s = spring()
        self.r_s = spring()
        self.plane = fixed_wing()
        self.time_max = 10
        self.no_of_springs = 2

        self.time_list = []

    def plane_x_to_spring_stretch(self, plane_x, distance_to_mount_point):

        if plane_x > 0:
            return math.sqrt(plane_x**2+distance_to_mount_point**2)-self.l_s.spring_equilibrum
        else:
            return -math.sqrt(plane_x**2+distance_to_mount_point**2)-self.l_s.spring_equilibrum

    def log(self):
        self.time_list.append(self.time)
        self.l_s.log()
        self.r_s.log()
        self.plane.log()



class fixed_wing(object):
    def __init__(self):
        self.weight = 2.6
        self.init_speed = 17
        self.init_ke = 0.5*self.weight*self.init_speed**2
        self.speed = self.init_speed
        self.x = 0

        self.speed_list = []
        self.x_list = []

    def log(self):
        self.speed_list.append(self.speed)
        self.x_list.append(self.x)


class spring(object): #https://www.thespringstore.com/pe028-375-94768-mw-3160-mh-n-in.html?unit_measure=me
    def __init__(self):
        self.neutral_length = 80/1000
        self.spring_constant = 42
        self.spring_equilibrum = self.neutral_length

        self.pre_stretched = 0.03
        self.current_stretch = self.pre_stretched
        self.spring_force = -self.spring_constant*self.current_stretch
        self.energy_stored = 0.5*self.spring_constant*self.current_stretch**2
        self.spring_angle = 0

        self.current_stretch_list = []
        self.spring_force_list = []

    def log(self):
        self.current_stretch_list.append(self.current_stretch)
        self.spring_force_list.append(self.spring_force)




#if __name__ == "__main__":

    #output_file = "simu.html"
model = simple_model()

def run_simu(model):
    model.log()
    while model.time < model.time_max:
        #Looking at the planes movement in delta t
        delta_x = model.plane.speed*model.delta_t
        model.plane.x += delta_x
        current_spring_stretch = model.plane_x_to_spring_stretch(model.plane.x, model.l_s.neutral_length+model.l_s.pre_stretched)
        model.l_s.current_stretch = current_spring_stretch
        model.r_s.current_stretch = current_spring_stretch
        total_spring_force = (model.l_s.spring_force + model.r_s.spring_force)*model.no_of_springs/2

        #This force will accelerate the plane according to newtons second law
         #The springs will accelerate in the opposite direction of the planes movement
        a = total_spring_force/model.plane.weight

        delta_v = a*model.delta_t
        model.plane.speed += delta_v
        model.time += model.delta_t
        model.log()



# Set up plot
plot_pos = figure(plot_height=400, plot_width=400, title="Position plot",
              tools="crosshair,pan,reset,save,wheel_zoom",  x_range=[0, 4*np.pi], y_range=[-2.5, 2.5])
source_pos = ColumnDataSource(data=dict(x=model.time_list, pos=model.plane.x_list))
plot_pos.line('t', 'pos', source=source_pos, line_width=3, line_alpha=0.6)

plot_spring_force = figure(plot_height=400, plot_width=400, title="Spring force",
              tools="crosshair,pan,reset,save,wheel_zoom",  x_range=[0, 4*np.pi], y_range=[-2.5, 2.5])
source_spring_force = ColumnDataSource(data=dict(t=model.time_list, y=model.plane.speed_list))
plot_spring_force.line('t', 'f_spring', source=source_spring_force, line_width=3, line_alpha=0.6)

plot_vel = figure(plot_height=400, plot_width=400, title="Velocity plot",
              tools="crosshair,pan,reset,save,wheel_zoom",  x_range=[0, 4*np.pi], y_range=[-2.5, 2.5])
source_vel = ColumnDataSource(data=dict(t=model.time_list, y=model.plane.speed_list))
plot_vel.line('t', 'velocity', source=source_vel, line_width=3, line_alpha=0.6)


# Set up widgets
time_max_s = Slider(title="Simulation time", value=1.0, start=1, end=20, step=0.1)
no_springs_s = Slider(title="Number of springs", value=2.0, start=2, end=40, step=2)
plane_weight_s = Slider(title="Fixed-wing weight", value=2.6, start=0.2, end=5)
init_plane_speed_s = Slider(title="Initial speed of fixed-wing", value=17, start=5, end=30, step=0.1)



def update_data(attrname, old, new):

    model = simple_model()
    # Get the current slider values
    model.time_max = time_max_s.value
    model.no_of_springs = no_springs_s.value
    model.plane.weight = plane_weight_s.value
    model.plane.speed = init_plane_speed_s.value

    # Generate the new curve
    run_simu(model)

    source_pos.data =          dict(t=model.time_list, pos=model.plane.x_list)
    source_spring_force.data = dict(t=model.time_list, f_spring=model.l_s.spring_force_list)
    source_vel.data =          dict(t=model.time_list, velocity=model.plane.speed_list)

time_max_s = Slider(title="Simulation time", value=1.0, start=1, end=20, step=0.1)
no_springs_s = Slider(title="Number of springs", value=2.0, start=2, end=40, step=2)
plane_weight_s = Slider(title="Fixed-wing weight", value=2.6, start=0.2, end=5, step=0.1)
init_plane_speed_s = Slider(title="Initial speed of fixed-wing", value=17, start=5, end=30, step=0.1)

for w in [time_max_s, no_springs_s, plane_weight_s, init_plane_speed_s]:
    w.on_change('value', update_data)

# Set up layouts and add to document
inputs = widgetbox(time_max_s, no_springs_s, plane_weight_s, init_plane_speed_s)

curdoc().add_root(row(inputs, plot_pos, plot_vel, plot_spring_force, width=800))
curdoc().title = "Sliders"


print "Ran this script"
#show([inputs, plot])
#l = layout([inputs, plot], sizing_mode='stretch_both')
#grid = gridplot([inputs, plot], ncols=2, plot_width=250, plot_height=250)
#show(grid)
