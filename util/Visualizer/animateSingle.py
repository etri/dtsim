#!/usr/bin/python3.6

import shapefile
import matplotlib.pyplot as plt
import matplotlib.animation
import numpy as np
import configparser
import random
import time 

######################################
# STEP1 : load config parameter
######################################
config = configparser.ConfigParser()
config.read('animate.conf')

opt_xinch = float(config.get('PARAMETER', 'screen.xwidth.inch'))
opt_shape_path = config.get('PARAMETER', 'shape.file.path')
opt_road_enable = int(config.get('PARAMETER', 'shape.road.enable'))
opt_area_enable = int(config.get('PARAMETER', 'shape.area.enable'))
opt_building_enable = int(config.get('PARAMETER', 'shape.building.enable'))
opt_human_path = config.get('PARAMETER', 'file.path.human')
opt_human_size = int(config.get('PARAMETER', 'human.icon.size'))
opt_human_color = config.get('PARAMETER', 'human.icon.color')
opt_human_rate = float(config.get('PARAMETER', 'human.sampling.rate'))
opt_bus_path = config.get('PARAMETER', 'file.path.bus')
opt_bus_size = int(config.get('PARAMETER', 'bus.icon.size'))
opt_bus_color = config.get('PARAMETER', 'bus.icon.color')

######################################
# STEP2 : set global variables
######################################
# declare geometry for Sejong city
xmin = 127.127729912708
xmax = 127.410803835874
ymin = 36.3972620634463
ymax = 36.7338218767278

# list for containing all agent's location within a step
STEPS = ""
X_HUMAN = []
Y_HUMAN = []

END = False

# canvas size in proportion to the real geometry
x_inch = opt_xinch
y_inch = x_inch * ((ymax-ymin) / (xmax-xmin))

######################################
# STEP3 : create matplot object 
######################################
fig = plt.figure()
fig.set_size_inches(x_inch, y_inch)

ax = fig.add_subplot(111)

plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)
plt.xlim([xmin, xmax])
plt.ylim([ymin, ymax])

annotation = plt.annotate('', xy=(0, 0), xytext=(0, 0))

######################################
# STEP4 : read shape object 
######################################
if opt_road_enable == 1:
    sf = shapefile.Reader(opt_shape_path + "/ROAD")
    for shape in sf.shapes():
        ap = plt.Polygon(shape.points, closed=False, fill=False, alpha=0.3, linewidth=0.5, edgecolor="k")
        ax.add_patch(ap)

if opt_area_enable == 1 :
    sf_area = shapefile.Reader(opt_shape_path + "/areas")
    for shape in sf_area.shapes():
        ap = plt.Polygon(shape.points, fill=False, alpha=0.5, linewidth=0.5, edgecolor="k")
        ax.add_patch(ap)

if opt_building_enable == 1 :
    sf_building = shapefile.Reader(opt_shape_path + "/buildings")
    for shape in sf_building.shapes():
        ap = plt.Polygon(shape.points, fill=True, alpha=0.5, linewidth=0.7, edgecolor="k")
        ax.add_patch(ap)

######################################
# STEP5 : load routing information  
######################################
file_human = open(opt_human_path, 'rt')

def loadHumanMovement():
    global STEPS
    global END

    while True:
        line = file_human.readline()
        if line == "":
            END = True;
            break

        if line.startswith('STEP'):
            STEPS = line
            break

        # filtering agent by probability
        if opt_human_rate != 1.0 :
            if random.random() > opt_human_rate :
                continue;

        ar = line.strip('\n').split(',')
        xval = float(ar[0])
        yval = float(ar[1])
        X_HUMAN.append(xval)
        Y_HUMAN.append(yval)

######################################
# STEP6 : animate agent movement  
######################################
def animate(i):
    global annotation 

    # clear list 
    X_HUMAN.clear()
    Y_HUMAN.clear()

    # load agents location in a step
    loadHumanMovement()

    if END == True:
        time.sleep(1)
        return

    # clear canvas
    ax = plt.gca()
    del ax.collections[:]
    annotation.remove()

    # print agents location 
    plt.scatter(X_HUMAN, Y_HUMAN, s=opt_human_size, c=opt_human_color)

    annotation = plt.annotate(STEPS, xy=(127.35, 36.70), xytext=(127.35, 36.70))

############### START ###############

# set animation loop
ani = matplotlib.animation.FuncAnimation(fig, animate, frames=1440, interval=100, repeat=True)

# draw pyplot
plt.show()

# close data file
file_human.close();
