#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sys
sys.path.append("../Stability")

from robot_description import Robot
import static_stability
import utils as ut




robot = Robot.load_from_file("robots/robot_8.xml")

poly_static = static_stability.static_stability_polyhedron(robot, 0.001, 100, measure=static_stability.Measure.AREA, linearization=False, friction_sides = 16, mode=static_stability.Mode.best)
poly_static.project_static_stability()

file = open("build/vertices.txt", 'r')

x = []
y = []
z = []

u = []
v = []
w = []

edges = []

# outer points
oX = []
oY = []
oZ = []

# outer edges
outerEdges = []

for line in file:
    line = line.split(';')
    if line[0]=='iv':
        x.append(float(line[1]))
        y.append(float(line[2]))
        z.append(float(line[3]))
        u.append(float(line[4]))
        v.append(float(line[5]))
        w.append(float(line[6]))

    elif line[0]=='ie':
        edges.append([[float(line[1]), float(line[4])],
                      [float(line[2]), float(line[5])],
                      [float(line[3]), float(line[6])]])

    elif line[0]=='ov':
        oX.append(float(line[1]));
        oY.append(float(line[2]));
        oZ.append(float(line[3]));

    elif line[0]=='oe':
        outerEdges.append([[float(line[1]), float(line[4])],
                           [float(line[2]), float(line[5])],
                           [float(line[3]), float(line[6])]])

    else:
        print("Unrecognise type :", line[0])

ax = robot.display_robot_configuration()

# ----------- display of static stability -----------
x1 = [v[0] for v in poly_static.inner_vertices]
x1.append(x1[0])
y1 = [v[1] for v in poly_static.inner_vertices]
y1.append(y1[0])

# ax.plot(x1, y1, color="xkcd:blue grey")
ax.plot(x1, y1, color="r")

# ----------- displahy of inner polyhedron -----------
ax.plot(x, y, z, 'go')
scale = 0.2
# ax.quiver(x, y, z, u, v, w, color="xkcd:kelly green")

for e in edges:
    ax.plot(e[0], e[1], e[2], color="xkcd:kelly green")

# ----------- displahy of outer polyhedron -----------
ax.plot(oX, oY, oZ, 'yo')

for e in outerEdges:
    ax.plot(e[0], e[1], e[2], color="xkcd:lemon yellow")


ax.set_xlabel("x")
ax.set_ylabel("y")

plt.show()

file.close()
