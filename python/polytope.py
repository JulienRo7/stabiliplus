#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from scipy.spatial import ConvexHull, HalfspaceIntersection
from random import choice
import time

import robot_description
import utils as ut

from cvxopt import matrix
import cvxopt
cvxopt.solvers.options['show_progress'] = False
cvxopt.solvers.options['glpk'] = {'msg_lev': 'GLP_MSG_OFF'}  # cvxopt 1.1.8

# from graphviz import Graphs

class Vertex():
    vertices_count = 0
    def __init__(self, coordinates = np.zeros((3,1)), direction = np.zeros((3,1))):
        self.index = self.vertices_count
        Vertex.vertices_count += 1
        self.coordinates = coordinates

        self.direction = direction
        self.offset = self.direction.T.dot(self.coordinates)

        self.edges = []
        self.faces = []

    def display(self, ax, name=False, direction=False, color='xkcd:leaf green'):
        scale = 0.5
        ax.plot(self.coordinates[0], self.coordinates[1], self.coordinates[2], color=color, marker = 'o')

        if name:
            ax.text(self.coordinates[0][0], self.coordinates[1][0], self.coordinates[2][0], self.index, None)

        if direction:
            ax.quiver(self.coordinates[0], self.coordinates[1], self.coordinates[2], scale*self.direction[0], scale*self.direction[1], scale*self.direction[2], color=color)

    def __eq__(self, other):
        return self.index == other.index

    def __str__(self):
        return "Vertex {}".format(self.index)

    def __repr__(self):
        return self.__str__()

    # def __repr__(self):
    #     return self.__str__()

class Edge():
    edges_count = 0
    def __init__(self, vertices, faces):
        self.index = self.edges_count
        Edge.edges_count += 1
        self.vertices = vertices
        self.faces = faces

        self.outer_validity = True

    def display(self, ax, name=False, color='xkcd:leaf green'):
        x = [pt.coordinates[0][0] for pt in self.vertices]
        y = [pt.coordinates[1][0] for pt in self.vertices]
        z = [pt.coordinates[2][0] for pt in self.vertices]


        ax.plot(x,y,z, color=color, marker = None, linewidth = 0.5)

        if name:
            ax.text(sum(x)/2, sum(y)/2, sum(z)/2, self.index, color='r')
    def __eq__(self, other):
        return self.index == other.index

    def __str__(self):
        return "Edge {}".format(self.index)

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return self.index

class Face():
    faces_count = 0
    def __init__(self, vertices, edges):
        # Description of the face
        self.index = self.faces_count
        Face.faces_count += 1
        self.vertices = vertices
        self.edges = edges

        # some characteristics of the face
        normal = np.cross(np.reshape(self.vertices[1].coordinates-self.vertices[0].coordinates, 3), np.reshape(self.vertices[2].coordinates-self.vertices[0].coordinates, 3))
        area = np.linalg.norm(normal)
        normal/=area
        area/=2
        self.normal = normal
        self.area = area
        self.offset = np.matmul(self.normal.T, self.vertices[0].coordinates)

        self.support_point = None
        self.support_function = 0


    def get_neighbors(self):
        neighbors = []

        for e in self.edges:
            if self == e.faces[0]:
                neighbors.append(e.faces[1])
            else:
                neighbors.append(e.faces[0])

        return neighbors

    def display(self, ax, name=False, normal=False, color='xkcd:leaf green'):
        x = [pt.coordinates[0][0] for pt in self.vertices]
        y = [pt.coordinates[1][0] for pt in self.vertices]
        z = [pt.coordinates[2][0] for pt in self.vertices]


        try:
            ax.plot_trisurf(x,y,z, color=color, alpha=0.2)
        except:
            # print(self, "failed")
            pass
        if normal:
            scale =0.3
            ax.quiver(sum(x)/len(x), sum(y)/len(y), sum(z)/len(z), scale*self.normal[0], scale*self.normal[1], scale*self.normal[2], color=color)

        if name:
            ax.text(sum(x)/len(x), sum(y)/len(y), sum(z)/len(z), self.index, color='m')

        # ax.plot()


    def __eq__(self, other):
        return self.index == other.index

    def __str__(self):
        return "Face {}".format(self.index)

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return self.index

class polytope():

    def __init__(self, robot, accelerations=[], eps=0.01, max_it=100, CoM_max=[1, 1, 1], linearization = False, friction_sides = 8 ):
        ############ Definition of the robot #######################
        self.robot = robot
        # In the 3D robuts case, the P matrix is the identity
        self.robot.set_P_matrix()
        self.CoM_max = np.array(CoM_max).reshape((3,1))
        # For now accelerations are given as a list of vertices -> the accelerations considered are the one within the convex shape described by these points.
        self.accelerations = accelerations
        self.n_acc = len(accelerations)

        ############ Initialisation of the polyhedron storage
        # Reprsentation of the inner polyhedron
        self.vertices = [] # list of Vertex
        self.edges = [] # list of Edges
        self.faces = [] # list of Faces
        self.inner_vertex = np.zeros((3,1))
        self.directions = [] # direction corresponding to the vertices with same index

        self.outer_faces = []
        self.outer_edges = []
        self.outer_vertices = []

        ############ Other variable
        self.use_linearized_friction = linearization
        self.friction_sides = friction_sides

        self.iterations = 0
        self.max_iteration = max_it

        self.problem_time = 0.0
        self.time_support = 0.0
        self.time_inner = 0.0
        self.time_outer = 0.0

    def init_robust_stability_problem(self):
        self.robot.set_P_matrix()

        size1 = self.robot.A1.shape
        size2 = self.robot.A2.shape

        self.Omega = np.zeros(
            (self.n_acc*size1[0], self.n_acc*size1[1]))
        self.Theta = np.zeros((self.n_acc*size2[0], size2[1]))
        self.Gamma = np.zeros((self.n_acc*size2[0], 1))

        for i in range(self.n_acc):
            g = self.accelerations[i]
            self.robot.set_acceleration(g)
            self.robot.update_t()
            self.robot.update_A2()

            self.Omega[size1[0]*i:size1[0]*(i+1), size1[1]*i:size1[1]*(i+1)] = self.robot.A1
            self.Theta[size2[0]*i:size2[0]*(i+1), :] = self.robot.A2[:, :]
            self.Gamma[size2[0]*i:size2[0]*(i+1), :] = self.robot.t[:, :]

        # self.Lambda = 0
        # self.Upsilon = 0

    def build_SOCP_problem(self):
        # print("Number of accelerations : ", self.n_acc)
        N_optim = 3*self.robot.n_feet*self.n_acc + 3

        # Building the equilibrium
        self.A = np.hstack((self.Omega, self.Theta))
        self.A = matrix(self.A)
        self.b = matrix(self.Gamma)


        # Settng the contact constraint and CoM position constraints
        f_max = self.robot.masse * 9.81  # -> Maybe this can be changed later
        self.Gl = np.zeros((2*self.robot.n_feet*self.n_acc+6, N_optim))
        self.hl = np.zeros((2*self.robot.n_feet*self.n_acc+6, 1))

        self.Gl[-6:-3, -3:] = np.eye(3)
        self.Gl[-3:, -3:] = -np.eye(3)
        self.hl[-6:] = np.vstack((self.CoM_max, self.CoM_max))

        self.Gk = []
        self.hk = []

        for i in range(self.robot.n_feet):
            R = np.array(self.robot.feet[i][:3, :3])
            lin_cons1 = np.matmul(np.array([0, 0, 1]), R.T)
            lin_cons2 = np.matmul(np.array([0, 0, -1]), R.T)

            F = np.array([[0, 0, self.robot.mu[i]], [1, 0, 0], [0, 1, 0]])
            cone_cons = np.matmul(F, R.T)

            for j in range(self.n_acc):
                # Linear constraints
                self.Gl[j*2*self.robot.n_feet + 2*i, 3*self.robot.n_feet *
                    j + 3*i: 3*self.robot.n_feet*j + 3*i+3] = lin_cons1
                self.Gl[j*2*self.robot.n_feet + 2*i+1, 3*self.robot.n_feet *
                    j + 3*i: 3*self.robot.n_feet*j + 3*i+3] = lin_cons2
                self.hl[j*2*self.robot.n_feet + 2*i] = f_max

                # Cone constraints
                Gi = np.zeros((3, N_optim))
                Gi[:, j*3*self.robot.n_feet + 3*i: j*3*self.robot.n_feet + 3*(i+1)] = cone_cons
                hi = np.zeros(3)

                self.Gk.append(matrix(-Gi))
                self.hk.append(matrix(hi))

        self.Gl = matrix(self.Gl)
        self.hl = matrix(self.hl)

        self.SOCP_built = True

    def linearized_friction_cone(self, N, mu, f_max):
        # Return the G matrix and h vector giving the linearized friction constraint for one foot in form:
        # G * f <= h
        # N is number of side of the polyhedron approximation, mu the friction coefficient and f_max the upper bound for the normal component of the contact force.
        G = np.zeros((N+2, 3))
        h = np.zeros((N+2, 1))

        h[1, 0] = f_max

        G[0, 2] = -1
        G[1, 2] = 1

        Dtheta = 2*np.pi/N
        for i in range(N):
            G[2+i, 0] = np.cos(i*Dtheta)/np.cos(Dtheta/2)
            G[2+i, 1] = np.sin(i*Dtheta)/np.cos(Dtheta/2)
            G[2+i, 2] = -mu#*np.sqrt(2)/2 #-> to have the restrictive linearized friction cone

        return G, h

    def build_LP_problem(self):
        # print("Number of accelerations : ", self.n_acc)
        N_optim = 3*self.robot.n_feet*self.n_acc + 3

        # Building the equilibrium
        self.A = np.hstack((self.Omega, self.Theta))
        self.A = matrix(self.A)
        self.b = matrix(self.Gamma)

        # Settng the contact constraint and CoM position constraints
        f_max = self.robot.masse * 9.81  # -> Maybe this can be changed later
        self.Gl = np.zeros(((self.friction_sides+2)*self.robot.n_feet*self.n_acc+6, N_optim))
        self.hl = np.zeros(((self.friction_sides+2)*self.robot.n_feet*self.n_acc+6, 1))

        self.Gl[-6:-3, -3:] = np.eye(3)
        self.Gl[-3:, -3:] = -np.eye(3)
        self.hl[-6:] = np.vstack((self.CoM_max, self.CoM_max))

        self.Gk = []
        self.hk = []

        for i in range(self.robot.n_feet):
            R = np.array(self.robot.feet[i][:3, :3])
            Gi, hi = self.linearized_friction_cone(self.friction_sides, self.robot.mu[i], f_max)
            Gi = np.matmul(Gi, R.T)

            hi = hi.reshape((self.friction_sides + 2,1))

            for j in range(self.n_acc):
                # Linear constraints
                self.Gl[j*(self.friction_sides+2)*self.robot.n_feet + (self.friction_sides+2)*i:j*(self.friction_sides+2)*self.robot.n_feet + (self.friction_sides+2)*i + (self.friction_sides+2),
                        3*self.robot.n_feet * j + 3*i: 3*self.robot.n_feet*j + 3*i+3] = Gi

                self.hl[j*(self.friction_sides+2)*self.robot.n_feet + (self.friction_sides+2)*i:j*(self.friction_sides+2)*self.robot.n_feet + (self.friction_sides+2)*i + (self.friction_sides+2)] = hi

        # ut.print_np_2d_array(self.Gl)
        self.Gl = matrix(self.Gl)
        self.hl = matrix(self.hl)

        self.LP_built = True

    def build_problem(self):
        self.init_robust_stability_problem()
        if self.use_linearized_friction:
            self.build_LP_problem()
        else:
            self.build_SOCP_problem()

    def solve_SOCP(self, c):
        if not self.SOCP_built:
            self.build_SOCP_problem()

        return cvxopt.solvers.socp(c, self.Gl, self.hl, self.Gk, self.hk, self.A, self.b)

    def solve_LP(self, c):
        if not self.LP_built:
            self.build_LP_problem()

        return cvxopt.solvers.lp(c, self.Gl, self.hl, self.A, self.b, solver='glpk')

    def solve_problem(self, c):
        if self.use_linearized_friction:
            return self.solve_LP(c)
        else:
            return self.solve_SOCP(c)

    def build_poly(self):
        ############ Building of the inner poly
        s = time.time()
        self.edges.append(Edge([self.vertices[0], self.vertices[1]], []))
        self.edges.append(Edge([self.vertices[1], self.vertices[2]], []))
        self.edges.append(Edge([self.vertices[0], self.vertices[2]], []))
        self.edges.append(Edge([self.vertices[0], self.vertices[3]], []))
        self.edges.append(Edge([self.vertices[1], self.vertices[3]], []))
        self.edges.append(Edge([self.vertices[2], self.vertices[3]], []))

        normal = np.cross(np.reshape(self.vertices[1].coordinates-self.vertices[0].coordinates, 3), np.reshape(self.vertices[2].coordinates-self.vertices[0].coordinates, 3))
        area = np.linalg.norm(normal)
        normal/=area
        area/=2

        if np.matmul(normal.T, self.vertices[3].coordinates-self.vertices[0].coordinates)>=0:
            self.faces.append(Face([self.vertices[0],self.vertices[2], self.vertices[1]], [self.edges[0], self.edges[1], self.edges[2]]))
            self.faces.append(Face([self.vertices[0],self.vertices[1], self.vertices[3]], [self.edges[0], self.edges[3], self.edges[4]]))
            self.faces.append(Face([self.vertices[0],self.vertices[3], self.vertices[2]], [self.edges[2], self.edges[3], self.edges[5]]))
            self.faces.append(Face([self.vertices[1],self.vertices[2], self.vertices[3]], [self.edges[1], self.edges[4], self.edges[5]]))
        else:
            self.faces.append(Face([self.vertices[0],self.vertices[1], self.vertices[2]], [self.edges[0], self.edges[1], self.edges[2]]))
            self.faces.append(Face([self.vertices[0],self.vertices[3], self.vertices[1]], [self.edges[0], self.edges[3], self.edges[4]]))
            self.faces.append(Face([self.vertices[0],self.vertices[2], self.vertices[3]], [self.edges[2], self.edges[3], self.edges[5]]))
            self.faces.append(Face([self.vertices[1],self.vertices[3], self.vertices[2]], [self.edges[1], self.edges[4], self.edges[5]]))


        self.edges[0].faces=[self.faces[0], self.faces[1]]
        self.edges[1].faces=[self.faces[0], self.faces[3]]
        self.edges[2].faces=[self.faces[0], self.faces[2]]
        self.edges[3].faces=[self.faces[1], self.faces[2]]
        self.edges[4].faces=[self.faces[1], self.faces[3]]
        self.edges[5].faces=[self.faces[2], self.faces[3]]

        self.inner_vertex = sum([v.coordinates for v in self.vertices])/4

        e = time.time()
        self.time_inner +=e-s
        ########## Building of the outer poly
        s = time.time()
        cores_vert = {}

        for f in self.faces:
            A = np.vstack([v.direction.T for v in f.vertices])
            b = np.vstack([v.offset for v in f.vertices])
            pt_coord = np.linalg.inv(A).dot(b)

            vertex = Vertex(pt_coord)
            self.outer_vertices.append(vertex)
            f.support_point=vertex
            f.support_function = f.normal.T.dot(vertex.coordinates)-f.offset

        cores_edge = {}
        for e in self.edges:
            edge = Edge([e.faces[0].support_point, e.faces[1].support_point], [])
            self.outer_edges.append(edge)
            edge.vertices[0].edges.append(edge)
            edge.vertices[1].edges.append(edge)
            cores_edge[e] = edge

        self.outer_faces.append(Face([self.faces[0].support_point, self.faces[2].support_point, self.faces[1].support_point],
                                     [cores_edge[self.edges[0]], cores_edge[self.edges[2]], cores_edge[self.edges[3]]]))
        self.outer_faces.append(Face([self.faces[0].support_point, self.faces[1].support_point, self.faces[3].support_point],
                                     [cores_edge[self.edges[0]], cores_edge[self.edges[1]], cores_edge[self.edges[4]]]))
        self.outer_faces.append(Face([self.faces[0].support_point, self.faces[3].support_point, self.faces[2].support_point],
                                     [cores_edge[self.edges[1]], cores_edge[self.edges[2]], cores_edge[self.edges[5]]]))
        self.outer_faces.append(Face([self.faces[1].support_point, self.faces[2].support_point, self.faces[3].support_point],
                                     [cores_edge[self.edges[3]], cores_edge[self.edges[4]], cores_edge[self.edges[5]]]))

        cores_edge[self.edges[0]].faces=[self.outer_faces[0], self.outer_faces[1]]
        cores_edge[self.edges[1]].faces=[self.outer_faces[1], self.outer_faces[2]]
        cores_edge[self.edges[2]].faces=[self.outer_faces[0], self.outer_faces[2]]
        cores_edge[self.edges[3]].faces=[self.outer_faces[0], self.outer_faces[3]]
        cores_edge[self.edges[4]].faces=[self.outer_faces[1], self.outer_faces[3]]
        cores_edge[self.edges[5]].faces=[self.outer_faces[2], self.outer_faces[3]]

        e = time.time()
        self.time_outer +=e-s

    def update_poly(self, vert, dir, dir_face):
        s = time.time()
        # adding the new point and direction
        self.vertices.append(vert)
        self.directions.append(dir)

        # Find the surfaces that v is outside:
        visible_faces = [dir_face]
        visible_edges = dir_face.edges[:]

        to_test = dir_face.get_neighbors()

        while to_test:
            # print("Visible faces:", visible_faces, "To test:", to_test)
            face = to_test.pop()
            # print("testing face:", face)
            if np.matmul(face.normal, vert.coordinates)-face.offset>0:
                visible_faces.append(face)
                visible_edges.extend([e for e in face.edges if e not in visible_edges])
                to_test.extend([face for face in face.get_neighbors() if face not in visible_faces and face not in to_test])

        # print("Visible faces:", visible_faces)
        # print("Visible edges:",visible_edges)

        # sort edges between those to delete and those to keep
        edges_to_delete = []
        edges_to_keep = []
        for e in visible_edges:
            if e.faces[0] in visible_faces and e.faces[1] in visible_faces:
                # if both faces of an edge are visible then the edge is inside the region to delete
                edges_to_delete.append(e)
            else:
                edges_to_keep.append(e)

        # print("Edges to delete :", edges_to_delete)
        # print("Edges to keep :", edges_to_keep)

        new_faces = []
        new_edges = []

        processed_vertex = []
        # computing the new stuff
        for e in edges_to_keep:
            # create a new face
            new_faces.append(Face([vert, e.vertices[0], e.vertices[1]], [e]))

            # add the new face to the edge (the old one will be removed later)
            e.faces.append(new_faces[-1])

            # create the new edges
            if e.vertices[0].index not in processed_vertex:
                new_edges.append(Edge([vert, e.vertices[0]],[]))
                processed_vertex.append(e.vertices[0].index)

                new_edges[-1].faces.append(new_faces[-1])
                new_faces[-1].edges.append(new_edges[-1])
            else:
                ind = processed_vertex.index(e.vertices[0].index)
                new_faces[-1].edges.append(new_edges[ind])
                new_edges[ind].faces.append(new_faces[-1])

            if e.vertices[1].index not in processed_vertex:
                new_edges.append(Edge([vert, e.vertices[1]], []))
                processed_vertex.append(e.vertices[1].index)

                new_edges[-1].faces.append(new_faces[-1])
                new_faces[-1].edges.append(new_edges[-1])
            else:
                ind = processed_vertex.index(e.vertices[1].index)
                new_faces[-1].edges.append(new_edges[ind])
                new_edges[ind].faces.append(new_faces[-1])

            # think about the orientation
            if np.matmul(new_faces[-1].normal, self.inner_vertex)-new_faces[-1].offset>0:
                new_faces[-1].normal*=-1
                new_faces[-1].offset*=-1



        # removing the old faces
        for f in visible_faces:
            for e in f.edges:
                e.faces.remove(f)

            self.faces.remove(f)

        # removing the old edges
        for e in edges_to_delete:
            try:
                self.edges.remove(e)
            except:
                print('\033[93m Warning',e,' has already been deleted\033[91m')
        # adding the new stuff
        self.faces.extend(new_faces)
        self.edges.extend(new_edges)

        e = time.time()
        self.time_inner +=e-s

        ########## Updating the outer Vertex
        s = time.time()
        # U_plus = [] -> Not needed: it is the outer vertex that are not modified
        U_0 = [] # outer vertex on the plane, they will be added to the outer points
        U_minus = [] # vertex that are out side the new half space: need to be removed

        # E_plus = [] -> Not needed: faces that are not intersected by the halfspace
        E_0 = [] # Edges that are intersected
        E_new = [] # New edges created on the plane
        E_minus = [] # Edges that are fully outside: need to be removed

        # F_plus -> not need: Faces that are fully inside
        F_0 = [] # Face that are intersecting with the plane
        F_new = [] # new face
        F_minus = [] # face that are fully outside there should not be

        # point_to_check = []
        edges_to_check = []

        pt0 = dir_face.support_point
        # print(vert.direction.T.dot(pt0.coordinates)-vert.offset)
        assert vert.direction.T.dot(pt0.coordinates)-vert.offset >= -1e5, "{}".format(vert.direction.T.dot(pt0.coordinates)-vert.offset)
        U_minus.append(pt0)

        edges_to_check = pt0.edges[:]
        checked_points = [pt0]
        checked_edges = []


        while edges_to_check:
            e = edges_to_check.pop()
            # print(edges_to_check, e)
            checked_edges.append(e)

            if e.vertices[0] not in checked_points:
                checked_points.append(e.vertices[0])
                check = vert.direction.T.dot(e.vertices[0].coordinates)-vert.offset
                if check > 0: # the point is out
                    U_minus.append(e.vertices[0]) # points added to the point to remove
                    edges_to_check.extend([edge for edge in e.vertices[0].edges if edge != e and edge not in checked_edges and edge not in edges_to_check]) # add new edges to check
                elif check == 0:
                    U_0.append(e.vertices[0])

            if e.vertices[1] not in checked_points:
                checked_points.append(e.vertices[1])
                check = vert.direction.T.dot(e.vertices[1].coordinates)-vert.offset
                if check > 0: # the point is out
                    U_minus.append(e.vertices[1]) # points added to the point to remove
                    edges_to_check.extend([edge for edge in e.vertices[1].edges if edge != e and edge not in checked_edges and edge not in edges_to_check]) # add new edges to check
                elif check == 0:
                    U_0.append(e.vertices[1])

            if e.vertices[0] in U_minus and e.vertices[1] in U_minus:
                E_minus.append(e)
                F_0.extend([f for f in e.faces if f not in F_0])
            elif (e.vertices[0] in U_minus and e.vertices[1] not in U_minus) or (e.vertices[0] not in U_minus and e.vertices[1] in U_minus):
                d0 = abs(vert.direction.T.dot(e.vertices[0].coordinates)-vert.offset)
                d1 = abs(vert.direction.T.dot(e.vertices[1].coordinates)-vert.offset)
                w = d0*e.vertices[1].coordinates + d1*e.vertices[0].coordinates
                w /= d0+d1
                w = Vertex(w)
                w.edges.append(e)
                w.faces.extend(e.faces)
                U_0.append(w)

                if e.vertices[0] in U_minus:
                    e.vertices[0] = U_0[-1]
                else:
                    e.vertices[1] = U_0[-1]
                E_0.append(e)
                F_0.extend([f for f in e.faces if f not in F_0])


        for f in F_0:
            f.edges = [e for e in f.edges if e not in E_minus]
            if not f.edges or not f.vertices:
                F_minus.append(f)
            else:
                new_points = []
                for e in f.edges:
                    if e in E_0:
                        if e.vertices[0] in U_0 and e.vertices[0] not in f.vertices:
                            new_points.append(e.vertices[0])
                            # e.vertices[0].edges.append(e)
                            # e.vertices[0].faces.extend([fa for fa in e.faces if fa not in e.vertices[0].faces])
                        elif e.vertices[1] in U_0 and e.vertices[1] not in f.vertices:
                            new_points.append(e.vertices[1])
                            # e.vertices[1].edges.append(e)
                            # e.vertices[1].faces.extend([fa for fa in e.faces if fa not in e.vertices[1].faces])


                f.vertices = [v for v in f.vertices if v not in U_minus]

                f.vertices.extend(new_points)
                assert len(new_points)==2, "{}: number of new points : {}, number of edges: {}".format(f,len(new_points), len(f.edges))
                new_edge = Edge(new_points, [f])
                f.edges.append(new_edge)
                E_new.append(new_edge)
                for pt in new_points:
                    pt.edges.append(new_edge)

        # pivot = U_0[0]
        # for i in range(1, len(U_0)):

        # print(U_minus)
        # print(E_minus)
        # print("Number of new points :", len(U_0))
        self.outer_vertices.extend(U_0)
        self.outer_edges.extend(E_new)
        new_face = Face(U_0, E_new)
        if np.matmul(new_face.normal, self.inner_vertex)-new_face.offset>0:
            new_face.normal*=-1
            new_face.offset*=-1

        for v in U_0:
            v.faces.append(new_face)

        for e in E_new:
            e.faces.append(new_face)


        self.outer_faces.append(new_face)

        for v in U_minus:
            self.outer_vertices.remove(v)

        for e in E_minus:
            self.outer_edges.remove(e)

        for f in F_minus:
            self.outer_faces.remove(f)

        e = time.time()
        self.time_outer +=e-s

    def compute_supports(self):

        for inner_face in self.faces:
            support_point_to_be_computed = False
            if not inner_face.support_point:
                support_point_to_be_computed = True
            else:
                if inner_face.support_point not in self.outer_vertices:
                    support_point_to_be_computed = True

            if support_point_to_be_computed:
                cpt_dist = 0
                current_support_point = choice(self.outer_vertices)
                current_distance = inner_face.normal.T.dot(current_support_point.coordinates)-inner_face.offset
                cpt_dist +=1
                current_neighbors = []
                for oe in current_support_point.edges:
                    current_neighbors.extend([ov for ov in oe.vertices if ov not in current_neighbors])

                outer_points_to_visit = current_neighbors

                visited_points = [current_support_point]+outer_points_to_visit
                current_point_is_support_point = False
                while not current_point_is_support_point:
                    point_to_visit_value = [inner_face.normal.T.dot(opv.coordinates)-inner_face.offset for opv in outer_points_to_visit]
                    cpt_dist +=len(point_to_visit_value)
                    val_max = max(point_to_visit_value)
                    if val_max <= current_distance:
                        current_point_is_support_point = True
                    else:
                        ind_max = point_to_visit_value.index(val_max)
                        current_distance = val_max
                        current_support_point = outer_points_to_visit[ind_max]

                        current_neighbors = []
                        for oe in current_support_point.edges:
                            current_neighbors.extend([ov for ov in oe.vertices if ov not in current_neighbors and ov not in visited_points])

                        if current_neighbors:
                            outer_points_to_visit = current_neighbors
                            visited_points.extend(outer_points_to_visit)
                        else:
                            current_point_is_support_point = True
                inner_face.support_point = current_support_point
                inner_face.support_function = current_distance
                # print("Support function {}/{}".format(cpt_dist, len(self.outer_vertices)))

    def display(self, subplot_conf=111, outer=False):
        ax = self.robot.display_robot_configuration(None, subplot_conf)
        for v in self.vertices:
            v.display(ax, name=False)

        for e in self.edges:
            e.display(ax, False)

        for f in self.faces:
            f.display(ax, name=False, normal=True)

        if outer:
            for v in self.outer_vertices:
                v.display(ax, name=False, color='xkcd:golden rod')

            for e in self.outer_edges:
                e.display(ax, name=False, color='xkcd:golden rod')

            for f in self.outer_faces:
                f.display(ax, name=False, normal=False, color='xkcd:golden rod')

    def display_outer_spherical_dual(self, subplot_conf=111):
        ax = plt.subplot(subplot_conf, projection='3d')
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_zlim(-1.5, 1.5)

        for v in self.vertices:
            ax.quiver(0, 0, 0, v.direction[0], v.direction[1], v.direction[2], color="xkcd:robin's egg blue")
            ax.text(v.direction[0][0], v.direction[1][0], v.direction[2][0], v.index, None)

        for e in self.edges:
            d1 = e.vertices[0].direction
            d2 = e.vertices[1].direction

            tps = np.linspace(0,1, 20)
            points = [t*d2 + (1-t)*d1 for t in tps]
            pts = [pt/np.linalg.norm(pt) for pt in points]

            x=[pt[0][0] for pt in pts]
            y=[pt[1][0] for pt in pts]
            z=[pt[2][0] for pt in pts]

            ax.plot(x,y,z, color="xkcd:medium blue")

    def projection_robust_polyhedron(self):

        self.build_problem()
        print(self.Gl, self.hl, self.A, self.b)
        # Choosing the initial directions
        initial_directions = [[[0], [0], [1.]]]
        n = 3
        for i in range(n):
            d = np.array([[np.cos(2*i*np.pi/n)], [np.sin(2*i*np.pi/n)], [-1]])
            d /= np.linalg.norm(d)
            initial_directions.append(d)

        # Finding the points corresponding to the initial direction
        for d in initial_directions:
            d = np.array(d)
            c = matrix(np.vstack((np.zeros((3*self.robot.n_feet*self.n_acc, 1)), -d)))

            s = time.time()
            result = self.solve_problem(c)
            e = time.time()
            self.problem_time += e-s


            v = Vertex(np.array(result['x'][-3:]), d)

            self.directions.append(d)
            self.vertices.append(v)
            self.iterations += 1

        ########
        # Initialisation of inner and outer hulls
        ########
        self.build_poly()
        s = time.time()
        self.compute_supports()
        e =time.time()
        self.time_support += e-s
        ########
        # iterations
        ########
        stop = sum([f.support_function*f.area for f in self.faces])
        while(self.iterations<self.max_iteration) and stop > 0.0001:
            print("Iteration ", self.iterations, "Error :", stop)

            face_area_max = max(self.faces, key=lambda face: face.support_function)

            d = np.reshape(face_area_max.normal, (3,1))
            c = matrix(np.vstack((np.zeros((3*self.robot.n_feet*self.n_acc, 1)), -d)))

            s = time.time()
            result = self.solve_problem(c)
            e = time.time()
            self.problem_time += e-s

            v = Vertex(np.array(result['x'][-3:]), d)

            self.update_poly(v, d, face_area_max)
            s = time.time()
            self.compute_supports()
            e =time.time()
            self.time_support += e-s
            # self.compute_supports()
            stop = sum([f.support_function*f.area for f in self.faces])
            self.iterations += 1


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

if __name__ == '__main__':
    #########################################
    # Initialization
    #########################################
    # Description of the robot
    robot = generate_robot(2, 0.5)

    # accelerations = [np.array([[0], [1], [-9.81]])]
    gm=0
    accelerations = [np.array([[0], [0], [-9.81]])]
    # accelerations = [np.array([[gm], [0], [-9.81]]),
    #                  np.array([[-gm], [0], [-9.81]]),
    #                  np.array([[0], [gm], [-9.81]]),
    #                  np.array([[0], [-gm], [-9.81]])]

    max_it = 50
    eps = 0.001

    linearize_friction = True
    number_of_sides = 32

    poly = polytope(robot, accelerations, max_it=max_it,  linearization=linearize_friction, friction_sides = number_of_sides)
    start = time.time()
    poly.projection_robust_polyhedron()
    end = time.time()
    # times.append(1000*(end-start))
    print("Computation time: {:3.4}ms for {} iterations".format(1000*(end-start), poly.iterations))
    print("Solving the SOCP or LP problems took {:3.4}ms".format(1000*poly.problem_time))
    print("Computing support functions took {:3.4}ms".format(1000*poly.time_support))
    print("Computing the inner polyhedron took {:3.4}ms".format(1000*poly.time_inner))
    print("Computing the outer polyhedron took {:3.4}ms".format(1000*poly.time_outer))

    # tree = BSPBranch()
    # tree.build_tree(poly.edges, poly.faces)
    #
    #
    # print(tree.branch_str())
    poly.display(111, outer=True)
    # poly.display_outer_spherical_dual(122)
    plt.show()


    # dot = Graph(comment="Polyhedron graph: node=face edge=edge")
    #
    # for face in poly.faces:
    #     dot.node(str(face.index), str(face))
    #
    # for edge in poly.edges:
    #     dot.edge(str(edge.faces[0].index), str(edge.faces[1].index), label=str(edge.index))
    #
    # # print(dot.source)
    # dot.render("Results/test_graph2.gv", view=True)
