#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation, FFMpegWriter

from matplotlib.tri import Triangulation
from matplotlib.colors import LightSource

import xml.etree.ElementTree as ET # phone home!
import sys

from scipy.spatial import ConvexHull

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
    def __init__(self):
        self.innerVertices = [] # inner vertices
        self.searchDirs = [] # search directions
        self.outerVertices = [] # outer vertices
        self.normals = [] # inner sides normals
        
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

    def loadStaticPointXML(self, xmlPoint):
        for child in xmlPoint:
            if (child.tag == "InnerVertex"):
                self.innerVertices.append([float(child.attrib['x']), float(child.attrib['y'])])
            elif (child.tag == "SearchDirection"):
                self.searchDirs.append([float(child.attrib['x']), float(child.attrib['y'])])
            elif (child.tag == "OuterVertex"):
                self.outerVertices.append([float(child.attrib['x']), float(child.attrib['y'])])
            elif (child.tag == "Normal"):
                self.normals.append([float(child.attrib['x']), float(child.attrib['y'])])
            else:
                raise NameError("Unrecognized static point tag: "+child.tag)

    def loadXML(self, xmlPoly):
        for child in xmlPoly:
            if (child.tag == "staticPoint"):
                self.loadStaticPointXML(child)
            else:
                print("Unrecognized static polytope tag: ", child.tag)

                
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
                lines.append(ax.quiver(v[0], v[1], 0, scale*d[0], scale*d[1], 0, color='r'))
                
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
            lines.extend(self.dispInner(ax, display_innerVertices = True, display_searchDirs = False, display_normals = False))

        if dispOuter:
            lines.extend(self.dispOuter(ax))

        # ax.set_aspect('equal')
        # ax.set_xbound(-2, 2)
        # ax.set_ybound(-2, 2)
        return lines
        
class robustPoly(polytope):
    def __init__(self):

        # index of the inner vertices
        self.innerIndex = []

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

        # list of the inner faces vertices (each inner face is a triangle and is reprensented by the index of its 3 vertices)
        self.innerFaces = []
        self.innerFacesNormals = []
        self.innerFacesOffsets = []

        #self.readFile(file_name)

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

    def loadVertexXML(self, xmlVertex):
        self.innerIndex.append(int(xmlVertex.attrib['i']))
        self.innerX.append(float(xmlVertex.attrib['x']))
        self.innerY.append(float(xmlVertex.attrib['y']))
        self.innerZ.append(float(xmlVertex.attrib['z']))
        self.innerU.append(float(xmlVertex.attrib['dx']))
        self.innerV.append(float(xmlVertex.attrib['dy']))
        self.innerW.append(float(xmlVertex.attrib['dz']))

    def loadEdgeXML(self, xmlEdge):
        self.innerEdges.append([[float(xmlEdge.attrib['x1']), float(xmlEdge.attrib['x2'])],
                                [float(xmlEdge.attrib['y1']), float(xmlEdge.attrib['y2'])],
                                [float(xmlEdge.attrib['z1']), float(xmlEdge.attrib['z2'])]])
    
    def loadFaceXML(self, xmlFace):
        self.innerFaces.append([int(xmlFace.attrib[a]) for a in ['v1', 'v2', 'v3']])
        self.innerFacesNormals.append(np.array([float(xmlFace.attrib[a]) for a in ['x', 'y', 'z']]))
        self.innerFacesOffsets.append(float(xmlFace.attrib['offset']))

    def getVertexPos(self, index): 
        return self.innerIndex.index(index)

    def reOrderFaceIndexes(self):
        # swap faces vertices' index to make sure they are in counter clockwise order 
        innerVertices = [np.array([x, y, z]) for x, y, z in zip(self.innerX, self.innerY, self.innerZ)]
        
        for face, n in zip(self.innerFaces, self.innerFacesNormals):
            v1 = innerVertices[self.getVertexPos(face[0])]
            v2 = innerVertices[self.getVertexPos(face[1])]
            v3 = innerVertices[self.getVertexPos(face[2])]

            norm = np.cross(v2-v1, v3-v1)
            if (np.dot(n, norm)<0):
                i = face[2]
                face[2] = face[1]
                face[1] = i


    def loadXML(self, xmlPoly):
        for child in xmlPoly:
            if (child.tag == "innerVertices"):
                for xmlVertex in child:
                    self.loadVertexXML(xmlVertex)
                    
            elif (child.tag == "innerEdges"):
                for xmlEdge in child:
                    self.loadEdgeXML(xmlEdge)

            elif (child.tag == "innerFaces"):
                for xmlFace in child:
                    self.loadFaceXML(xmlFace)
            else:
                print("Unrecognized robust polytope tag: ", child.tag)
                
    def dispInner(self, ax, dispEdges=True, dispInnerNormals=False, dispVertexes=True, color="xkcd:kelly green"):
        # ----------- display of inner polyhedron -----------
        innerline = []
        if dispVertexes:
            innerline.append(ax.scatter(self.innerX, self.innerY, self.innerZ, color=color, marker='o'))

        if dispInnerNormals:
            scale = 0.2
            U = [scale*u for u in self.innerU]
            V = [scale*v for v in self.innerV]
            W = [scale*w for w in self.innerW]

            ax.quiver(self.innerX, self.innerY, self.innerZ, U, V, W, color=color)

        if dispEdges:
            for e in self.innerEdges:
                ax.plot(e[0], e[1], e[2], color=color)

        dispSides = True
        if dispSides:
            self.sideDisplay(ax)

        return innerline

    def dispOuter(self, ax, dispEdges=True, color="xkcd:purple"):
        # ----------- display of outer polyhedron -----------
        ax.plot(self.outerX, self.outerY, self.outerZ, color=color, marker='o')

        if dispEdges:
            for e in self.outerEdges:
                ax.plot(e[0], e[1], e[2], color=color)

        return []


    def display(self, ax, dispInner = True, dispOuter = False, innerColor="xkcd:green", outerColor="xkcd:purple"):
        lines = []
        if dispInner:
            lines.extend(self.dispInner(ax, True, False, color=innerColor))

        if dispOuter:
            lines.extend(self.dispOuter(ax, color=outerColor))

        return lines
    
    def flatDisplay(self, ax, color="xkcd:maroon", name=""):
        self.innerX
        self.innerY
        inner = []
        for i in range(len(self.innerX)):
            inner.append([self.innerX[i], self.innerY[i]])
            
        conv = ConvexHull(np.reshape(np.array(inner), (len(self.innerX), 2)))
        x = [self.innerX[i] for i in conv.vertices]
        y = [self.innerY[i] for i in conv.vertices]

        x.append(x[0])
        y.append(y[0])

        if (name == ""):
            ax.plot(x, y, color=color)
        else:
            ax.plot(x, y, color=color, label=name)

    def sideDisplay(self, ax, color="xkcd:green", name=""):
        # build the triangulation object
        self.reOrderFaceIndexes()

        def getVerticePos(listI): return [self.getVertexPos(i) for i in listI]
        faces = [getVerticePos(f) for f in self.innerFaces]

        triangles = Triangulation(self.innerX, self.innerY, faces)

        ax.plot_trisurf(triangles, self.innerZ,
        # ax.plot_trisurf(self.innerX, self.innerY, self.innerZ, 
                         color=color, alpha=0.4, shade=True)

    
class ComputationPoint:
    def __init__(self):
        self.contactSet = None
        self.contactSetName = ""
        self.polyType = ""
        self.polytopes = []

        # storing the times
        self.totalTime = 0
        self.lpTime = 0
        self.initTime = 0
        self.structTime = 0

        # storing the solver type
        self.solver = None

        # storing points
        self.points = {}
        self.pointsColor = {}
        self.pairs = {}

    def loadXML(self, compPoint):
        for child in compPoint:
            if child.tag == "poly":
                # open the file
                # Find which kind of polytope it is then load it
                fileName = child.attrib['file_name']
                compPtTree = ET.parse(fileName)
                compPtRoot = compPtTree.getroot()

                self.polyType = compPtRoot.attrib['type']
                if (self.polyType == "robust"):
                    poly = robustPoly()
                    poly.loadXML(compPtRoot)
                    self.polytopes.append(poly)
                    
                elif (self.polyType == "constrained"):
                    poly1 = robustPoly()
                    poly1.loadXML(compPtRoot[0])
                    self.polytopes.append(poly1)
                    
                    poly2 = robustPoly()
                    poly2.loadXML(compPtRoot[1])
                    self.polytopes.append(poly2)
                
                elif (self.polyType == "static"):
                    poly = staticPoly()
                    poly.loadXML(compPtRoot)
                    self.polytopes.append(poly)
                    
                else:
                    raise NameError("Unknown polytope type")
                
            elif child.tag == "robot":
                self.contactSet = Robot.load_from_file(child.attrib['file_name'])
                self.constacSetName = child.attrib['name']

            elif child.tag == "times":
                self.totalTime = int(child.attrib['total'])
                self.lpTime = int(child.attrib['LP'])
                self.initTime = int(child.attrib['init'])
                self.structTime = int(child.attrib['struct'])

            elif child.tag == "solver":
                self.solver = child.attrib['name']

            elif child.tag == "point":
                name = child.attrib['name']
                x = float(child.attrib['x'])
                y = float(child.attrib['y'])
                z = float(child.attrib['z'])
                coord = np.reshape(np.array([x, y, z]), (3, 1))
                
                self.points[name] = coord

                if ('color' in child.attrib):
                    self.pointsColor[name] = child.attrib['color']
            
            else:
                print("Unrecognise compPoint tag:", child.tag)

        # looking for the pairs
        for key in self.points.keys():
            if key[-4:]=="_Max":
                minKey = key[:-4]+"_Min"
                if minKey in self.points.keys():
                    self.pairs[key] = minKey
        
    def displayPolytopes(self, ax=None):
        lines = []
        colors = ["xkcd:green", "xkcd:olive", "xkcd:teal"]

        colorIndex = 0
        
        for pol in self.polytopes:
            lines.extend(pol.display(ax, innerColor = colors[colorIndex]))
            
            colorIndex +=1
            if colorIndex >= len(colors):
                colorIndex = 0
        
        return lines


class PostProcessor:
    def __init__(self, file_name="/res/results.xml"):
        self.mode = 0
        self.robust = False
        self.numComputedPoints = 0

        self.computationPoints = []
        
        # self.robots = []
        # self.robot_names = []
        # self.polytopes = []

        # self.solvers = []
        # self.total_times = []
        # self.LPTimes = []
        # self.initTimes = []
        # self.structTimes = []

        self.points = {}
        
        self.loadExperiment(file_name)

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
                compPt = ComputationPoint()
                compPt.loadXML(child)
                self.computationPoints.append(compPt)
            else:
                print("Unrecognise tag: ", child.tag)

        print("Experiment loaded, mode {} with {} computed points".format(self.mode, self.numComputedPoints))

    def display_mode_1(self):
        # print("There are {} inner vertices and {} outer vertices".format(len(self.polytopes[0].innerVertices), len(self.polytopes[0].outerVertices)))
        ax, lines = self.computationPoints[0].contactSet.display_robot_configuration()

        if self.robust:
            poly_static = static_stability.static_stability_polyhedron(self.computationPoints[0].contactSet, 0.001, 100, measure=static_stability.Measure.AREA, linearization=False, friction_sides = 16, mode=static_stability.Mode.best)
            poly_static.project_static_stability()
            
            # ----------- display of static stability -----------
            x1 = [v[0][0] for v in poly_static.inner_vertices]
            x1.append(x1[0])
            y1 = [v[1][0] for v in poly_static.inner_vertices]
            y1.append(y1[0])
            ax.plot(x1, y1, color="xkcd:red")
            # ax.plot(x1, y1, color="r")

        print(len(self.computationPoints[0].polytopes))
        self.computationPoints[0].polytopes[0].display(ax, dispInner=True, dispOuter=False)
        
        # Displaying the points
        for ptName in self.computationPoints[0].points.keys():
            coord = self.computationPoints[0].points[ptName]
            x = coord[0][0]
            y = coord[1][0]
            z = coord[2][0]
            if (ptName in self.computationPoints[0].pointsColor.keys()):
                color = self.computationPoints[0].pointsColor[ptName]
            else:
                color = "xkcd:red"
            
            ax.plot([x],[y],[z], 'o-', color = color)

        # ax.set_xlim(-0.0, 1.5)
        # ax.set_ylim(-0.0, 1.5)
        # # ax.set_zlim(-0.1, 2)
        # # ax.set_xlim(-0.5, 0.5)
        # ax.set_ylim(-0.5, 0.5)
        # ax.set_zlim(-0.1, 2)
        
        ax.set_xlabel("x", size="xx-large")
        ax.set_ylabel("y", size="xx-large")
        ax.set_zlabel("z", size="xx-large")

        ax.xaxis.set_label_coords(10, 10)

        
        ax.xaxis.set_tick_params(labelsize="xx-large")
        ax.yaxis.set_tick_params(labelsize="xx-large")
        ax.zaxis.set_tick_params(labelsize="xx-large")
        
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

    def display_mode_4(self):

        fig = plt.figure()
        ax = Axes3D(fig)

        lines = []

        pointsData = {}
        
        def update(frame, lines, ax, pointsData):
            ax.cla()
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            # ax, lines = self.computationPoints[frame].contactSet.display_robot_configuration(ax)
            lines = []
            # x1 = [v[0] for v in staticPolys[frame].inner_vertices]
            # x1.append(x1[0])
            # y1 = [v[1] for v in staticPolys[frame].inner_vertices]
            # y1.append(y1[0])
            # lines.extend(ax.plot(x1, y1, color="xkcd:red"))
            
                
            # for ptName in self.computationPoints[frame].points.keys():
            for ptName in ["comQP"]:
                coord = self.computationPoints[frame].points[ptName]
                x = coord[0][0]
                y = coord[1][0]
                z = coord[2][0]

                if (ptName in self.computationPoints[frame].pointsColor.keys()):
                    color = self.computationPoints[frame].pointsColor[ptName]
                else:
                    color = "xkcd:red"
                
                lines.extend(ax.plot([x],[y],[z], 'o-', color=color, label=ptName))

                if (ptName in pointsData.keys()):
                    data = pointsData[ptName]
                    data[0].append(x)
                    data[1].append(y)
                    data[2].append(z)
                else:
                    data = ([x], [y], [z])
                    pointsData[ptName] = data

            # for ptName in pointsData.keys():
            for ptName in ["comQP"]:
                data = pointsData[ptName]

                if (ptName in self.computationPoints[frame].pointsColor.keys()):
                    color = self.computationPoints[frame].pointsColor[ptName]
                else:
                    color = "xkcd:red"

                lines.extend(ax.plot(data[0], data[1], data[2], '-', color=color, alpha=0.2))

                for (key1, key2) in self.computationPoints[frame].pairs.items():
                    coord1 = self.computationPoints[frame].points[key1]
                    coord2 = self.computationPoints[frame].points[key2]
                    x = [coord1[0][0], coord2[0][0]]
                    y = [coord1[1][0], coord2[1][0]]
                    z = [coord1[2][0], coord2[2][0]]
                    
                    if (key1 in self.computationPoints[frame].pointsColor.keys()):
                        color = self.computationPoints[frame].pointsColor[key1]
                    else:
                        color = "xkcd:red"
                    
                    lines.extend(ax.plot(x,y,z, '-', color=color))

            lines.extend(self.computationPoints[frame].displayPolytopes(ax))
            ax.set_xlim(-0.5, 1.0)
            ax.set_ylim(-1.0, 0.5)
            # ax.set_zlim(-0.1, 2)

        ani = FuncAnimation(fig, update, self.numComputedPoints, fargs=(lines, ax, pointsData), interval=10, blit=False, repeat_delay=200, save_count=1)

        # print("Saving the animation...")
        # # moviewriter = FFMpegWriter(fps=20)
        # # moviewriter.setup(fig=fig, outfile="res/video.mp4")
        # ani.save("/home/julien/Desktop/video.mp4", fps=10, dpi=360)
        # # moviewriter.finnish()
        # print("Animation saved!")
        plt.show()

    def display_mode_5(self):
        # print("Starting display mode 5")
        # total_times = [compPt.totalTime for compPt in self.computationPoints]
        # plt.plot(total_times)
        # plt.xlabel("Contact set number")
        # plt.ylabel("Computing time (ms)")
        # plt.title("Equilibrium region computation time")
        # plt.show()

        pointsData = {}
        pointsColor = {}
        for compPt in self.computationPoints:
            for name in compPt.points.keys():
                coord = compPt.points[name]
                x = coord[0][0]
                y = coord[1][0]
                z = coord[2][0]
            
                if (name in pointsData.keys()):
                    data = pointsData[name]
                    data[0].append(x)
                    data[1].append(y)
                    data[2].append(z)
                else:
                    data = ([x], [y], [z])
                    pointsData[name] = data
                    pointsColor[name] = compPt.pointsColor[name]

        for name in pointsData.keys():
            # if (name == "chebichev"):
            #     data = pointsData[name]
            #     color = pointsColor[name]

            #     plt.plot(data[0], data[1], '-', color=color, label=name)
                
            # elif (name == "chebichev_Max"):
            #     data = pointsData[name]
            #     color = pointsColor[name]

            #     plt.plot(data[0], data[1], '-', color=color)
            # elif (name == "baryPoint"):
            #     data = pointsData[name]
            #     color = pointsColor[name]

            #     plt.plot(data[0], data[1], '-', color=color, label=name)
                
            # elif (name == "baryPoint_Max"):
            #     data = pointsData[name]
            #     color = pointsColor[name]

            #     plt.plot(data[0], data[1], '-', color=color)

            if (name == "comQP"):
                data = pointsData[name]
                color = pointsColor[name]

                plt.plot(data[0], data[1], '-', color=color, label=name)
            elif (name == "comQP_Max"):
                data = pointsData[name]
                color = pointsColor[name]

                # plt.plot(data[0], data[1], '-', color=color)

        # plot the feet position...
        # NonConservativeContactSet_0 = self.computationPoints[0].contactSet
        # NonConservativeContactSet_end = self.computationPoints[-1].contactSet

        # ConservativeContactSet = Robot(NonConservativeContactSet_0.masse, NonConservativeContactSet_0.feet[:-5], NonConservativeContactSet_0.mu[:-5], flim=NonConservativeContactSet_0.flim[:-5], dim = 3)
                                     
        # x0, y0 = NonConservativeContactSet_0.support_polygon()
        # plt.plot(x0, y0, color="k", label="Initial Non Conservative Support Polygon", linestyle="-.")
        # x1, y1 = NonConservativeContactSet_end.support_polygon()
        # plt.plot(x1, y1, color="k", label="Final Non Conservative Support Polygon", linestyle=":")

        # x2, y2 = ConservativeContactSet.support_polygon()
        # plt.plot(x2, y2, color="k", label="Conservative Support Polygon", linestyle="--")

        # ax = plt.gca()
        
        # indexes = [0, 20, 40, 60, 80, 100]
        # colors = ["xkcd:green", "xkcd:lime green", "xkcd:pale yellow", "xkcd:goldenrod", "xkcd:orange", "xkcd:red"]
        # # labels = ["F=0N", "F=90N", "F=180N", "F=270N", "F=360N", "F=450N"]
        # labels = ["x=0.98m", "x=1.04m", "x=1.10m", "x=1.16m", "x=1.22m", "x=1.28m"]
        
        # for i in range(len(indexes)):
        #     ind = indexes[i]
        #     poly = self.computationPoints[ind].polytopes[0]
        #     poly.flatDisplay(ax, color=colors[i], name=labels[i])


        # ax.set_title("Evolution of the Equilibrium Region as function of the X position of the Third Contact for F=100N")
        
        # # ax.set_title("Evolution of the Equilibrium Region as function of the Contact Force")
        # ax.legend()
        # plt.xlabel('X')
        # plt.ylabel('Y')
        # ax.set_xlim(-0.0, 1.2)
        # ax.set_ylim(-0.6, 0.4)
        # ax.set_aspect('equal', 'box')
        # plt.show()

        # # plotting the forces...
        # fig = plt.figure()
        # axX = fig.add_subplot(3, 1, 1)
        # axY = fig.add_subplot(3, 1, 2)
        # axZ = fig.add_subplot(3, 1, 3)

        # forceLF = pointsData["forceLF"]
        # colorLF = pointsColor["forceLF"]
        # forceRF = pointsData["forceRF"]
        # colorRF = pointsColor["forceRF"]
        # forceRH = pointsData["forceRH"]
        # colorRH = pointsColor["forceRH"]

        # axX.plot(forceLF[0], color=colorLF, label="LF")
        # axY.plot(forceLF[1], color=colorLF, label="LF")
        # axZ.plot(forceLF[2], '.-', color=colorLF, label="LF")

        # axX.plot(forceRF[0], color=colorRF, label="RF")
        # axY.plot(forceRF[1], color=colorRF, label="RF")
        # axZ.plot(forceRF[2], color=colorRF, label="RF")
        
        # axX.plot(forceRH[0], color=colorRH, label="RH")
        # axY.plot(forceRH[1], color=colorRH, label="RH")
        # axZ.plot(forceRH[2], color=colorRH, label="RH")

        # axX.set_title("Forces on the X axis")
        # axX.grid()
        # axX.legend()

        # axY.set_title("Forces on the Y axis")
        # axY.grid()
        # axY.legend()

        # axZ.set_title("Forces on the Z axis")
        # axZ.grid()
        # axZ.legend()
        
        # plt.show()

        fig2 = plt.figure()
        ax2 = fig2.add_subplot(projection="3d")

        poly = self.computationPoints[0].polytopes[0]
        poly.sideDisplay(ax2)

        plt.show()
        
    def display_results(self):

        if self.mode == 1:
            self.display_mode_1()
        # elif self.mode == 2:
        #     self.display_mode_2()
        # elif self.mode == 3:
        #     self.display_mode_3()
        elif self.mode == 4:
            self.display_mode_4()
        elif self.mode == 5:
            # self.display_mode_4()
            self.display_mode_5()
        elif self.mode == 6:
            self.display_mode_4()
        else:
            assert False, "Unknown Mode {}".format(self.mode)


if __name__ == '__main__':
    postProcess = PostProcessor("/tmp/results.xml")

    # postProcess.computationPoints = postProcess.computationPoints[-179:]
    # postProcess.numComputedPoints = len(postProcess.computationPoints)
    
    postProcess.display_results()
