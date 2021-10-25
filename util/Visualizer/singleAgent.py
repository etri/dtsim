#!/usr/bin/python3.6

import shapefile
import matplotlib.pyplot as plt
import matplotlib.animation
import numpy as np
import configparser
import time 

######################################
# STEP1 : load config parameters
######################################
config = configparser.ConfigParser()
config.read('animate.conf')

opt_xinch = float(config.get('PARAMETER', 'screen.xwidth.inch'))
opt_shape_path = config.get('PARAMETER', 'shape.file.path')
opt_road_enable = int(config.get('PARAMETER', 'shape.road.enable'))
opt_area_enable = int(config.get('PARAMETER', 'shape.area.enable'))
opt_building_enable = int(config.get('PARAMETER', 'shape.building.enable'))
opt_single_agent_path = config.get('PARAMETER', 'file.path.single.agent')
opt_human_path = config.get('PARAMETER', 'file.path.human')
opt_human_size = int(config.get('PARAMETER', 'human.icon.size'))
opt_human_color = config.get('PARAMETER', 'human.icon.color')
opt_bus_path = config.get('PARAMETER', 'file.path.bus')
opt_bus_size = int(config.get('PARAMETER', 'bus.icon.size'))
opt_bus_color = config.get('PARAMETER', 'bus.icon.color')


######################################
# STEP1 : set global variables
######################################
# declare geometry for Sejong city
xmin = 127.127729912708
xmax = 127.410803835874
ymin = 36.3972620634463
ymax = 36.7338218767278

# list for containing all agent's location within a step
STEPS = ""
X = []
Y = []

# canvas size in proportion to the real geometry
x_inch = opt_xinch
y_inch = x_inch * ((ymax-ymin) / (xmax-xmin))

######################################
# STEP2 : create matplot object 
######################################
fig = plt.figure()
fig.set_size_inches(x_inch, y_inch)

ax = fig.add_subplot(111)

plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)
plt.xlim([xmin, xmax])
plt.ylim([ymin, ymax])

######################################
# STEP3 : read shape object 
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
# STEP4 : load routing information  
######################################
f = open(opt_single_agent_path, 'rt')

def loadAgentMovement():
    while True:
        line = f.readline()
        if line == "":
            break

        ar = line.strip('\n').split(',')
        xval = float(ar[0])
        yval = float(ar[1])
        X.append(xval)
        Y.append(yval)

######################################
# STEP5 : print movement  
######################################

# load agents location in a step
loadAgentMovement()

# print agents location 
plt.scatter(X, Y, s=opt_human_size, c=opt_human_color, marker='h')

############### START ###############

# draw pyplot
plt.show()

# close data file
f.close();
