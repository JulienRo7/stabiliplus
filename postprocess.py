#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import xml.etree.ElementTree as ET # phone home!

import sys


sys.path.append("../Stability")

from robot_description import Robot
import static_stability
import utils as ut

class innerOuterPoly:
    def __init__(self, file_name):
        # coordinates of the inner points
        self.innerX = []
        self.innerY = []
        self.innerZ = []

        # Normals associated to the inner points
        self.innerU = []
        self.innerV = []
        self.innerW = []

        # list of inner edges, each edge is represented using a 2x3 matrix and each column gives the coordinates of one point
        self.innerEdges = []

        # coordinates of the outer points
        self.outerX = []
        self.outerY = []
        self.outerZ = []

        # list of outer edges, each edge is represented using a 2x3 matrix and each column gives the coordinates of one point
        self.outerEdges = []

        self.read_polytopeFile(file_name)

    def read_polytopeFile(self, file_name):
        file = open(file_name, 'r')

        for line in file:
            line = line.split(';')
            if line[0]=='iv':
                self.innerX.append(float(line[1]))
                self.innerY.append(float(line[2]))
                self.innerZ.append(float(line[3]))
                self.innerU.append(float(line[4]))
                self.innerV.append(float(line[5]))
                self.innerW.append(float(line[6]))

            elif line[0]=='ie':
                self.innerEdges.append([[float(line[1]), float(line[4])],
                                        [float(line[2]), float(line[5])],
                                        [float(line[3]), float(line[6])]])

            elif line[0]=='ov':
                self.outerX.append(float(line[1]));
                self.outerY.append(float(line[2]));
                self.outerZ.append(float(line[3]));

            elif line[0]=='oe':
                self.outerEdges.append([[float(line[1]), float(line[4])],
                                        [float(line[2]), float(line[5])],
                                        [float(line[3]), float(line[6])]])

            else:
                print("Unrecognise type :", line[0])

    def display_inner(self, ax, dispEdges=True, dispInnerNormals=False, color="xkcd:kelly green"):
        # ----------- display of inner polyhedron -----------
        innerline = ax.plot(self.innerX, self.innerY, self.innerZ, 'go')

        if dispInnerNormals:
            scale = 0.2
            U = [scale*u for u in self.innerU]
            V = [scale*v for v in self.innerV]
            W = [scale*w for w in self.innerW]

            ax.quiver(self.innerX, self.innerY, self.innerZ, U, V, W, color=color)

        if dispEdges:
            for e in self.innerEdges:
                ax.plot(e[0], e[1], e[2], color=color)

        return innerline

    def display_outer(self, ax, dispEdges=True, color="xkcd:kelly green"):
        # ----------- display of outer polyhedron -----------
        ax.plot(self.outerX, self.outerY, self.outerZ, color=color, marker='o')

        if dispEdges:
            for e in self.outerEdges:
                ax.plot(e[0], e[1], e[2], color=color)


    def display(self, ax, dispInner = True, dispOuter = False):
        lines = []
        if dispInner:
            lines.extend(self.display_inner(ax))

        if dispOuter:
            lines.extend(self.display_outer(ax))

        return lines



class PostProcessor:
    def __init__(self, file_name="/res/results.xml"):
        self.mode = 0
        self.numComputedPoints = 0
        self.robots = []
        self.polytopes = []

        self.loadExperiment(file_name)

    def loadCompPoint(self, compPoint):

        for child in compPoint:
            if child.tag == "poly":
                self.polytopes.append(innerOuterPoly(child.attrib['file_name']))

            elif child.tag == "robot":
                self.robots.append(Robot.load_from_file(child.attrib['file_name']))

            else:
                print("Unrecognise compPoint tag:", child.tag)

    def loadExperiment(self, file_name):
        print("Loading experiment...")
        tree = ET.parse(file_name)
        root = tree.getroot()

        for child in root:
            if child.tag == 'mode':
                self.mode = int(child.attrib['mode'])
            elif child.tag == 'numComp':
                self.numComputedPoints = int(child.attrib['numComputedPoints'])
            elif child.tag == 'compPoint':
                self.loadCompPoint(child)
            else:
                print("Unrecognise tag: ", child.tag)

        print("Experiment loaded, mode {} with {} computed points".format(self.mode, self.numComputedPoints))

    def display_mode_1(self):

        poly_static = static_stability.static_stability_polyhedron(self.robots[0], 0.001, 100, measure=static_stability.Measure.AREA, linearization=False, friction_sides = 16, mode=static_stability.Mode.best)
        poly_static.project_static_stability()

        ax, lines = self.robots[0].display_robot_configuration()

        # ----------- display of static stability -----------
        x1 = [v[0] for v in poly_static.inner_vertices]
        x1.append(x1[0])
        y1 = [v[1] for v in poly_static.inner_vertices]
        y1.append(y1[0])
        ax.plot(x1, y1, color="xkcd:blue grey")
        # ax.plot(x1, y1, color="r")

        self.polytopes[0].display(ax)

        ax.set_xlabel("x")
        ax.set_ylabel("y")

        plt.show()



    def display_mode_3(self):
        fig = plt.figure()
        ax = Axes3D(fig)
        lines = []

        # precomputing the static stability polyhedrons
        print("Precomputing the static polyhedrons")
        staticPolys = []
        for rob in self.robots:
            poly_static = static_stability.static_stability_polyhedron(rob, 0.01, 50, measure=static_stability.Measure.AREA, linearization=False, friction_sides = 16, mode=static_stability.Mode.best)
            poly_static.project_static_stability()
            staticPolys.append(poly_static)
        print("Precomputation done!")

        def update(frame, lines, ax):
            ax.cla()
            ax, lines = self.robots[frame].display_robot_configuration(ax)

            x1 = [v[0] for v in staticPolys[frame].inner_vertices]
            x1.append(x1[0])
            y1 = [v[1] for v in staticPolys[frame].inner_vertices]
            y1.append(y1[0])
            lines.extend(ax.plot(x1, y1, color="xkcd:blue grey"))

            lines.extend(self.polytopes[frame].display(ax))


        ani = FuncAnimation(fig, update, self.numComputedPoints, fargs=(lines, ax), interval=10, blit=False, repeat_delay=200, save_count=1)
        plt.show()

    def display_results(self):

        if self.mode == 1:
            self.display_mode_1()
        elif self.mode == 3:
            self.display_mode_3()
        else:
            print("Unknown Mode {}".format(self.mode))
            assert False



if __name__ == '__main__':
    postProcess = PostProcessor("./res/results.xml")

    postProcess.display_results()
