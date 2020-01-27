#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation, FFMpegWriter
import xml.etree.ElementTree as ET # phone home!

import sys


sys.path.append("./python")

from robot_description import Robot
import static_stability
import utilities as ut

# Print iterations progress
def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█'):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print('\r%s |%s| %s%% %s' % (prefix, bar, percent, suffix), end = '\r')
    # Print New Line on Complete
    if iteration == total:
        print()

class polytope:
    def __init__(self, file_name):
        self.readFile(file_name)

    def readFile(self):
        pass

    def dispInner(self):
        pass

    def dispOuter(self):
        pass
    
    def display(self):
        pass

class staticPoly(polytope):
    def __init__(self, file_name="/tmp/static_res.txt"):
        self.innerVertices = [] # inner vertices
        self.searchDirs = [] # search directions
        self.outerVertices = [] # outer vertices
        self.normals = [] # inner sides normals

        self.readFile(file_name)
        
    def readFile(self, file_name):
        file = open(file_name, 'r')

        for line in file:
            line = line.split(';')
            if line[0]=='iv':
                self.innerVertices.append([float(line[1]),float(line[2])])
            elif line[0]=='sd':
                self.searchDirs.append([float(line[1]),float(line[2])])
            elif line[0]=='ov':
                self.outerVertices.append([float(line[1]),float(line[2])])
            elif line[0]=='no':
                self.normals.append([float(line[1]),float(line[2])])
            else:
                print("Unknown value")

                
    def dispInner(self, ax, display_innerVertices = True, display_searchDirs = True, display_normals = True):
        scale = 0.5
        lines = []
        if display_innerVertices:
            iv_x = [v[0] for v in self.innerVertices]
            iv_y = [v[1] for v in self.innerVertices]
            iv_x.append(iv_x[0])
            iv_y.append(iv_y[0])

            lines.append(ax.plot(iv_x, iv_y, 'r-'))

        if display_searchDirs:
            for v,d in zip(self.innerVertices, self.searchDirs):
                lines.append(ax.arrow(v[0], v[1], scale*d[0], scale*d[1], color='r'))
                
        if display_normals:
            m_x = [(v1[0]+v2[0])/2 for v1,v2 in zip(self.innerVertices[:-1], self.innerVertices[1:])]
            m_y = [(v1[1]+v2[1])/2 for v1,v2 in zip(self.innerVertices[:-1], self.innerVertices[1:])]
            m_x.append((self.innerVertices[-1][0]+self.innerVertices[0][0])/2)
            m_y.append((self.innerVertices[-1][1]+self.innerVertices[0][1])/2)
            
            for x, y, d in zip(m_x, m_y, self.normals):
                lines.append(ax.arrow(x, y, scale*d[0], scale*d[1], color='b'))

        return lines
                
    def dispOuter(self, ax, display_outerVertices = True):
        scale = 0.5
        lines = []
        
        if display_outerVertices:
            ov_x = [v[0] for v in outerVertices]
            ov_y = [v[1] for v in outerVertices]
            ov_x.append(ov_x[0])
            ov_y.append(ov_y[0])

            lines.append(ax.plot(ov_x, ov_y, 'g-'))

        return lines

    def display(self, ax, dispInner = True, dispOuter = False):

        
        lines = []
        if dispInner:
            lines.extend(self.dispInner(ax))

        if dispOuter:
            lines.extend(self.dispOuter(ax))

        # ax.set_aspect('equal')
        ax.set_xbound(-2, 2)
        ax.set_ybound(-2, 2)
        return lines
        
class robustPoly(polytope):
    def __init__(self, file_name):
        # coordinates of the inner points
        self.innerX = []
        self.innerY = []
        self.innerZ = []

        # Normals associated to the inner points/ search directions
        self.innerU = []
        self.innerV = []
        self.innerW = []

        # list of inner edges, each edge is represented using a 2x3 matrix and each column gives the coordinates of one point
        self.innerEdges = []
        self.innerEdgesNormals = []
        
        # coordinates of the outer points
        self.outerX = []
        self.outerY = []
        self.outerZ = []

        # list of outer edges, each edge is represented using a 2x3 matrix and each column gives the coordinates of one point
        self.outerEdges = []

        self.readFile(file_name)

    def readFile(self, file_name):
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

    def dispInner(self, ax, dispEdges=True, dispInnerNormals=False, color="xkcd:kelly green"):
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

    def dispOuter(self, ax, dispEdges=True, color="xkcd:kelly green"):
        # ----------- display of outer polyhedron -----------
        ax.plot(self.outerX, self.outerY, self.outerZ, color=color, marker='o')

        if dispEdges:
            for e in self.outerEdges:
                ax.plot(e[0], e[1], e[2], color=color)


    def display(self, ax, dispInner = True, dispOuter = False):
        lines = []
        if dispInner:
            lines.extend(self.dispInner(ax))

        if dispOuter:
            lines.extend(self.dispOuter(ax))

        return lines



class PostProcessor:
    def __init__(self, file_name="/res/results.xml"):
        self.mode = 0
        self.robust = False
        self.numComputedPoints = 0
        self.robots = []
        self.robot_names = []
        self.polytopes = []

        self.solvers = []
        self.total_times = []
        self.LPTimes = []
        self.initTimes = []
        self.structTimes = []
        
        self.loadExperiment(file_name)

    def loadCompPoint(self, compPoint):

        for child in compPoint:
            if child.tag == "poly":
                if self.robust:
                    self.polytopes.append(robustPoly(child.attrib['file_name']))
                else:
                    self.polytopes.append(staticPoly(child.attrib['file_name']))
                    
            elif child.tag == "robot":
                self.robots.append(Robot.load_from_file(child.attrib['file_name']))
                self.robot_names.append(child.attrib['name'])

            elif child.tag == "times":
                self.total_times.append(int(child.attrib['total']))
                self.LPTimes.append(int(child.attrib['LP']))
                self.initTimes.append(int(child.attrib['init']))
                self.structTimes.append(int(child.attrib['struct']))

            elif child.tag == "solver":
                self.solvers.append(child.attrib['name'])
            
            else:
                print("Unrecognise compPoint tag:", child.tag)

    def loadExperiment(self, file_name):
        print("Loading experiment...")
        tree = ET.parse(file_name)
        root = tree.getroot()

        for child in root:
            if child.tag == 'mode':
                self.mode = int(child.attrib['mode'])
            elif child.tag == 'robust':
                self.robust = child.attrib['robust'] == "true"
            elif child.tag == 'numComp':
                self.numComputedPoints = int(child.attrib['numComputedPoints'])
            elif child.tag == 'compPoint':
                self.loadCompPoint(child)
            else:
                print("Unrecognise tag: ", child.tag)

        print("Experiment loaded, mode {} with {} computed points".format(self.mode, self.numComputedPoints))

    def display_mode_1(self):
        # print("There are {} inner vertices and {} outer vertices".format(len(self.polytopes[0].innerVertices), len(self.polytopes[0].outerVertices)))
        ax, lines = self.robots[0].display_robot_configuration()

        if self.robust:
            poly_static = static_stability.static_stability_polyhedron(self.robots[0], 0.001, 100, measure=static_stability.Measure.AREA, linearization=False, friction_sides = 16, mode=static_stability.Mode.best)
            poly_static.project_static_stability()
            
            # ----------- display of static stability -----------
            x1 = [v[0] for v in poly_static.inner_vertices]
            x1.append(x1[0])
            y1 = [v[1] for v in poly_static.inner_vertices]
            y1.append(y1[0])
            ax.plot(x1, y1, color="xkcd:red")
            # ax.plot(x1, y1, color="r")

        self.polytopes[0].display(ax)

        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.grid(True)

        plt.show()

    def extract_timings(self, solver_name, robot_name):
        numPts = 0
        total = 0
        LP = 0
        init = 0
        struct = 0

        for i in range(self.numComputedPoints):
            if (self.solvers[i]==solver_name and self.robot_names[i]==robot_name):
                numPts+=1
                total+=self.total_times[i]
                LP+=self.LPTimes[i]
                init+=self.initTimes[i]
                struct+=self.structTimes[i]

        assert numPts != 0, "No point found for solver {} and robot {}".format(solver_name, robot_name)
        return (numPts, int(total/numPts), int(LP/numPts), int(init/numPts), int(struct/numPts))

    def display_mode_2(self):
        solvers = set(self.solvers)
        # robots = ["robot_1", "robot_2", "robot_3", "robot_4"]
        robots = set(self.robot_names)

        total_avg_per_sol = dict()
        total_LP_per_sol = dict()
        total_init_per_sol = dict()
        total_struct_per_sol = dict()
        
        for sol in solvers:
            sol_times = []
            LP_times = []
            init_times = []
            struct_times = []
            
            for rob in robots:
                timings = self.extract_timings(sol, rob)
                sol_times.append(timings[1])
                LP_times.append(timings[2])
                init_times.append(timings[3])
                struct_times.append(timings[4])
                # print(sol, rob, timings)
            total_avg_per_sol[sol]=sol_times
            total_LP_per_sol[sol]=LP_times
            total_init_per_sol[sol]=init_times
            total_struct_per_sol[sol]=struct_times

        print(total_avg_per_sol)
        print(total_LP_per_sol)

        x = np.arange(len(robots))  # the label locations
        width = 0.20  # the width of the bars

        # fig, axs = plt.subplots(2, 2)
        
        # ax_tot = axs[0,0]
        # ax_LP = axs[0,1]
        # ax_init = axs[1,0]
        # ax_struct = axs[1,1]
        fig_tot, ax_tot = plt.subplots()
        fig_LP, ax_LP = plt.subplots()
        fig_init, ax_init = plt.subplots()
        fig_struct, ax_struct = plt.subplots()
        
        rects_tot = []
        rects_LP = []
        rects_init = []
        rects_struct = []

        total_num_solvers = len(solvers)
        num_sol = 0
        
        for sol in solvers:
            rects_tot.append(ax_tot.bar(x - width*(total_num_solvers-1)/2 + num_sol*width, total_avg_per_sol[sol] , width, label=sol))
            rects_LP.append(ax_LP.bar(x - width*(total_num_solvers-1)/2 + num_sol*width, total_LP_per_sol[sol] , width, label=sol))
            rects_init.append(ax_init.bar(x - width*(total_num_solvers-1)/2 + num_sol*width, total_init_per_sol[sol] , width, label=sol))
            rects_struct.append(ax_struct.bar(x - width*(total_num_solvers-1)/2 + num_sol*width, total_struct_per_sol[sol] , width, label=sol))
            num_sol += 1

        axes = (ax_tot, ax_LP, ax_init, ax_struct)

        for ax in axes:
            # Add some text for labels, title and custom x-axis tick labels, etc.
            ax.set_xticks(x)
            ax.set_xticklabels(robots)
            ax.legend()

        ax_tot.set_ylabel('time (µs)')
        ax_tot.set_title('Comparison of different solvers: average stability region computation time')
        
        ax_LP.set_ylabel('time (µs)')
        ax_LP.set_title('Comparison of different solvers: average time solving LP')
        
        ax_init.set_ylabel('time (µs)')
        ax_init.set_title('Comparison of different solvers: average LP initialisation time')

        ax_struct.set_ylabel('time (µs)')
        ax_struct.set_title('Comparison of different solvers: average structural time')

        def autolabel(rects, rect_ax):
            """Attach a text label above each bar in *rects*, displaying its height."""
            for rect in rects:
                height = rect.get_height()
                rect_ax.annotate('{}'.format(height),
                            xy=(rect.get_x() + rect.get_width() / 2, height),
                            xytext=(0, 3),  # 3 points vertical offset
                            textcoords="offset points",
                            ha='center', va='bottom')


        for rect in rects_tot:
            autolabel(rect, ax_tot)

        for rect in rects_LP:
            autolabel(rect, ax_LP)

        for rect in rects_init:
            autolabel(rect, ax_init)

        for rect in rects_struct:
            autolabel(rect, ax_struct)
            
        # fig.tight_layout()
        plt.show()
    
    def display_mode_3(self):
        fig = plt.figure()
        ax = Axes3D(fig)

        lines = []

        # precomputing the static stability polyhedrons
        print("Precomputing the static polyhedrons")
        staticPolys = []
        it = 0
        for rob in self.robots:
            poly_static = static_stability.static_stability_polyhedron(rob, 0.01, 50, measure=static_stability.Measure.AREA, linearization=False, friction_sides = 16, mode=static_stability.Mode.best)
            poly_static.project_static_stability()
            staticPolys.append(poly_static)
            it+=1
            printProgressBar( it, len(self.robots))
        print("Precomputation done!")

        def update(frame, lines, ax):
            ax.cla()
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax, lines = self.robots[frame].display_robot_configuration(ax)

            x1 = [v[0] for v in staticPolys[frame].inner_vertices]
            x1.append(x1[0])
            y1 = [v[1] for v in staticPolys[frame].inner_vertices]
            y1.append(y1[0])
            lines.extend(ax.plot(x1, y1, color="xkcd:red"))

            lines.extend(self.polytopes[frame].display(ax))


        ani = FuncAnimation(fig, update, self.numComputedPoints, fargs=(lines, ax), interval=10, blit=False, repeat_delay=200, save_count=1)

        # print("Saving the animation...")
        # # moviewriter = FFMpegWriter(fps=20)
        # # moviewriter.setup(fig=fig, outfile="res/video.mp4")
        # ani.save("/home/julien/Desktop/video.mp4", fps=10, dpi=360)
        # # moviewriter.finnish()
        # print("Animation saved!")
        plt.show()        
    
    def display_results(self):

        if self.mode == 1:
            self.display_mode_1()
        elif self.mode == 2:
            self.display_mode_2()
        elif self.mode == 3:
            self.display_mode_3()
        else:
            print("Unknown Mode {}".format(self.mode))
            assert False


if __name__ == '__main__':
    postProcess = PostProcessor("/tmp/results.xml")

    postProcess.display_results()
