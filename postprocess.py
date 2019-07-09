#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sys
sys.path.append("/home/julien/Desktop/code-pyhton-hri/Stability")

import robot_description
import static_stability
import utils as ut


def generate_robot(pos, fric=0.5):
    masse = 1

    # Feet
    n_feet = 3
    feet = [np.eye(4) for i in range(n_feet)]
    mu = [fric for i in range(n_feet)]
    if pos == 1:
        feet[0][0:3, 3] = np.array([0, 1, 1])
        feet[0][:3, :3] = ut.euler2RotMat(-np.pi/4, 0, 0)

        feet[1][0:3, 3] = np.array([np.sqrt(3)/2, -1/2, 0])
        feet[1][:3, :3] = ut.euler2RotMat(0, np.pi/3, np.pi/4)

        feet[2][0:3, 3] = np.array([-np.sqrt(3)/2, -1/2, 0])
        feet[2][:3, :3] = ut.euler2RotMat(0, -np.pi/3, np.pi/4)

        robot = robot_description.Robot(masse, feet, mu)
    if pos == 2:
        feet[0][0:3, 3] = np.array([0, 1, 0])
        feet[0][:3, :3] = ut.euler2RotMat(-np.pi/4, 0, 0)

        feet[1][0:3, 3] = np.array([np.sqrt(3)/2, -1/2, 0])
        feet[1][:3, :3] = ut.euler2RotMat(0, np.pi/3, np.pi/4)

        feet[2][0:3, 3] = np.array([-np.sqrt(3)/2, -1/2, 0])
        feet[2][:3, :3] = ut.euler2RotMat(0, -np.pi/3, np.pi/4)

        robot = robot_description.Robot(masse, feet, mu)

    if pos == 3:
        feet[0][0:3, 3] = np.array([0, 1, 0])
        feet[0][:3, :3] = ut.euler2RotMat(-3*np.pi/4, 0, 0)

        feet[1][0:3, 3] = np.array([np.sqrt(3)/2, -1/2, 0])
        feet[1][:3, :3] = ut.euler2RotMat(0, np.pi/3, np.pi/4)

        feet[2][0:3, 3] = np.array([-np.sqrt(3)/2, -1/2, 0])
        feet[2][:3, :3] = ut.euler2RotMat(0, -np.pi/3, np.pi/4)
        robot = robot_description.Robot(masse, feet, mu)

    if pos == 4:
        feet[0][0:3, 3] = np.array([0, 1, 0])
        feet[0][:3, :3] = ut.euler2RotMat(0, 0, 0)

        feet[1][0:3, 3] = np.array([np.sqrt(3)/2, -1/2, 0])
        feet[1][:3, :3] = ut.euler2RotMat(0, 0, 0)

        feet[2][0:3, 3] = np.array([-np.sqrt(3)/2, -1/2, 0])
        feet[2][:3, :3] = ut.euler2RotMat(0, 0, 0)

        robot = robot_description.Robot(masse, feet, mu)

    if pos == 5:
        feet[0][0:3, 3] = np.array([1, 0, 1])
        feet[0][:3, :3] = ut.euler2RotMat(0, 0, 0)

        feet[1][0:3, 3] = np.array([-1/2, np.sqrt(3)/2, 0])
        feet[1][:3, :3] = ut.euler2RotMat(0, 0, 0)

        feet[2][0:3, 3] = np.array([-1/2, -np.sqrt(3)/2, 0])
        feet[2][:3, :3] = ut.euler2RotMat(0, 0, 0)

        robot = robot_description.Robot(masse, feet, mu)

    if pos == 6:
        n_feet = 8
        feet = [np.eye(4) for i in range(n_feet)]
        mu = [0.5 for i in range(n_feet)]
        masse = 80

        feet[0][0:3, 3] = np.array([-.6, .3, 0])
        feet[1][0:3, 3] = np.array([-.4, .3, 0])
        feet[2][0:3, 3] = np.array([-.6, -.3, 0])
        feet[3][0:3, 3] = np.array([-.4, -.3, 0])
        feet[4][0:3, 3] = np.array([.6, .3, 0])
        feet[5][0:3, 3] = np.array([.4, .3, 0])
        feet[6][0:3, 3] = np.array([.4, -.3, 0])
        feet[7][0:3, 3] = np.array([.6, -.3, 0])

        robot = robot_description.Robot(masse, feet, mu)

    if pos == 7:
        n_feet = 12
        feet = [np.eye(4) for i in range(n_feet)]
        mu = [1 for i in range(n_feet)]
        masse = 80

        feet[0][0:3, 3] = np.array([-0.25, 0.075, 0])
        feet[1][0:3, 3] = np.array([-0.25, -0.075, 0])
        feet[2][0:3, 3] = np.array([-0.35, 0.075, 0])
        feet[3][0:3, 3] = np.array([-0.35, -0.075, 0])

        feet[4][0:3, 3] = np.array([0.25, 0.075, 0])
        feet[5][0:3, 3] = np.array([0.25, -0.075, 0])
        feet[6][0:3, 3] = np.array([0.35, 0.075, 0])
        feet[7][0:3, 3] = np.array([0.35, -0.075, 0])

        feet[8][0:3, 3] = np.array([-0.075, .5, .9])
        feet[8][:3, :3] = ut.euler2RotMat(np.pi/2-0.001, 0, 0)
        feet[9][0:3, 3] = np.array([0.075, .5, .9])
        feet[9][:3, :3] = ut.euler2RotMat(np.pi/2-0.001, 0, 0)
        feet[10][0:3, 3] = np.array([-0.075, .5, 1.0])
        feet[10][:3, :3] = ut.euler2RotMat(np.pi/2-0.001, 0, 0)
        feet[11][0:3, 3] = np.array([0.075, .5, 1.0])
        feet[11][:3, :3] = ut.euler2RotMat(np.pi/2-0.001, 0, 0)

        robot = robot_description.Robot(masse, feet, mu)


    if pos == 0:
        n_feet = randint(3, 5)
        robot = robot_description.Robot.generate_random_robot(1, n_feet)

    return robot

robot = generate_robot(2)

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

ax.plot(x1, y1, color="xkcd:blue grey")

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
