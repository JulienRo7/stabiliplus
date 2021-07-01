#!/usr/bin/env python3

import robot_description
import utilities as ut
import static_stability

from random import randint, choice
import matplotlib.pyplot as plt

import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection
from scipy.spatial.qhull import QhullError
from scipy.optimize import linprog

from cvxopt import matrix
import cvxopt

import operator
import time

from enum import Enum, unique


# TODO: For now acceleration must be given as a list of np.array of shape (3,1): maybe some checks sould be added

@unique
class Mode(Enum):
  """Enum that shows the three modes of functionment."""
  precision = 1
  iteration = 2
  best = 3

@unique
class Measure(Enum):
    """ Enum that allows to choose the desired type of measure"""
    RANDOM = 0
    VOLUME = 1
    SUPPORT = 2
    FACET_AREA = 3

class robust_stability_polyhedron():

    def __init__(self, robot, accelerations=[], eps=0.01, max_it=100, CoM_max=[1, 1, 1], measure = Measure.VOLUME,
                    mode = Mode.best, linearization = False, friction_sides = 8 ):

        self.robot = robot
        # In the 3D robuts case, the P matrix is the identity
        self.robot.set_P_matrix()
        self.CoM_max = np.array(CoM_max).reshape((3,1))
        # For now accelerations are given as a list of vertices -> the accelerations considered are the one within the convex shape described by these points.
        self.accelerations = accelerations
        self.n_acc = len(accelerations)

        # definition of the inner and outer hull. The vertices are given in counter clockwise order
        self.inner_vertices = []
        self.directions = []
        self.offsets = []
        self.Y_inner = None
        self.Y_outer = None

        self.halfspaces = []
        self.outer_vertices = []
        self.outer_vertices_up_to_date = False

        self.measures = {}
        self.facets_points = {}
        self.validity = {}

        # self.measure = "volume"
        self.measure = measure


        self.current_facet = (0.,0.,0.,0.)
        self.new_dir = np.zeros((3,1))
        self.new_point = np.zeros((3,1))
        # stop criterions
        assert eps > 0 and max_it > 0
        self.eps = eps
        self.max_it = max_it
        self.iterations = 0
        self.error = 1000

        # self.stop_condition = False
        if (mode == Mode.precision):
            self.stop_condition = lambda: self.Y_outer.volume-self.Y_inner.volume > self.eps
        elif(mode == Mode.iteration):
            self.stop_condition = lambda: self.iterations < self.max_it
        elif(mode == Mode.best):
            self.stop_condition = lambda: (self.Y_outer.volume-self.Y_inner.volume > self.eps) and (self.iterations < self.max_it)
        else:
            raise ValueError("Unknown mode, please use a value supplied in enum")

        self.use_linearized_friction = linearization
        assert friction_sides >= 3
        self.friction_sides = friction_sides
        self.LP_built = False
        self.SOCP_built = False

        self.SOCP_time = 0
        self.init_time = 0
        self.Convex_time = 0
        self.invalidate_time = 0
        self.measure_time = 0

        self.total_time = 0

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

    def init_facet_validity(self):

        for ineq, points in zip(self.Y_inner.equations, self.Y_inner.simplices ):
            key = tuple(ineq)
            if self.measure == Measure.VOLUME or self.measure == Measure.SUPPORT:
                self.facets_points[key] = [np.array(pt) for pt in self.Y_inner.points[points].tolist()]

            if self.measure == Measure.RANDOM:
                self.validity[key] = True
            else:
                self.validity[key] = False

    def check_facet_validity(self):

        if self.measure == Measure.VOLUME:
            hs = self.halfspaces[-1]
            for ineq, points in zip(self.Y_inner.equations, self.Y_inner.simplices ):
                key = tuple(ineq)

                if key in self.facets_points:

                    bad_points = []
                    for i in range(len(self.facets_points[key])):
                        pt = self.facets_points[key][i]
                        if not ut.pointinHS(pt, hs, eps=1e-5):
                            bad_points.append(i)
                            self.validity[key] = False

                    bad_points.reverse()
                    for i in bad_points:
                        self.facets_points[key].pop(i)

                else: # if it is a new key added:
                    self.facets_points[key] = [np.array(pt) for pt in self.Y_inner.points[points].tolist()]
                    self.validity[key] = False


        elif self.measure == Measure.SUPPORT:
            hs = self.halfspaces[-1]
            for ineq, points in zip(self.Y_inner.equations, self.Y_inner.simplices ):
                key = tuple(ineq)

                if key in self.facets_points:

                    bad_points = []
                    for i in range(len(self.facets_points[key])):
                        pt = self.facets_points[key][i]
                        if not ut.pointinHS(pt, hs, eps=1e-5):
                            bad_points.append(i)
                            self.validity[key] = False

                    bad_points.reverse()
                    for i in bad_points:
                        self.facets_points[key].pop(i)

                else: # if it is a new key added:
                    self.facets_points[key] = [np.array(pt) for pt in self.Y_inner.points[points].tolist()]
                    self.validity[key] = False

        elif self.measure == Measure.RANDOM:
            pass
            # for ineq in self.Y_inner.equations:
            #     key = tuple(ineq)
            #     if key not in self.facets_points:
            #         self.validity[key] = False

        elif self.measure == Measure.FACET_AREA:
            pass
        else:
            raise ValueError("Unknown measure, please use a value supplied in enum")

    def update_measures(self):
        if self.measure == Measure.FACET_AREA:
            for ineq, facet_points in zip(self.Y_inner.equations, self.Y_inner.simplices):
                key = tuple(ineq)
                if key not in self.measures:
                    if len(facet_points)== 3:
                        # area of a 3d triangle -> slightly more complex than 2d
                        c1 = self.Y_inner.points[facet_points[1]]-self.Y_inner.points[facet_points[0]]
                        c2 = self.Y_inner.points[facet_points[2]]-self.Y_inner.points[facet_points[0]]

                        self.measures[key] = np.linalg.norm(np.cross(c1,c2))
                        # print(self.measures[key])

                    else:
                        raise ValueError("Facet with {} vertices".format(len(self.facets_points[key])))
                        # raise(ValueError)

        elif self.measure == Measure.RANDOM:
            for ineq in self.Y_inner.equations:
                key = tuple(ineq)
                if key not in self.validity:
                    self.validity[key] = True


        elif self.measure == Measure.VOLUME:
            keys = [key for key in self.facets_points.keys() if not self.validity[key]]

            for key in keys:
                for pt in self.outer_vertices:
                    if (not ut.pointinHS(pt, key)):# and (not pt in self.facets_points[key]):
                        self.facets_points[key].append(pt)

                self.halfspaces.append(-1*np.array(key))
                feasible_point = sum(self.facets_points[key])/len(self.facets_points[key])
                try:
                    vertices = ut.compute_outer_convex(self.halfspaces, feasible_point)
                    self.measures[key] = ConvexHull(np.array(vertices)).volume
                except (QhullError, RuntimeError):
                    self.measures[key] = 0

                self.halfspaces.pop()

                self.validity[key] = True

        elif self.measure == Measure.SUPPORT:

            keys = [key for key in self.facets_points.keys() if not self.validity[key]]

            for key in keys:
                for pt in self.outer_vertices:
                    if (not ut.pointinHS(pt, key)):# and (not pt in self.facets_points[key]):
                        self.facets_points[key].append(pt)


                normal = np.array(key[:-1]).reshape((1,3))
                norm  = np.linalg.norm(normal)


                dist = [np.matmul(normal, pt)+key[-1] for pt in self.facets_points[key]]
                dist.append(0)
                self.measures[key]=max(dist)/norm
                self.validity[key] = True

        else:
            raise ValueError("Unknown measure: {}, please use a value supplied in enum".format(self.measure))

    def build_inner_poly(self):
        self.Y_inner = ConvexHull(np.hstack(self.inner_vertices).T, incremental=True)

    def build_outer_poly(self):
        for i in range(len(self.inner_vertices)):
            d = self.directions[i].T.tolist()[0]
            offset = np.matmul(self.directions[i].T, self.inner_vertices[i])[0][0]
            self.offsets.append(offset)
            self.halfspaces.append([d[0], d[1], d[2], -offset])

        # the barycenter of the vertices is in the convex thus it is a feasible point
        main_feasible_point = sum(self.inner_vertices)/len(self.inner_vertices)
        try:
            self.outer_vertices = ut.compute_outer_convex(self.halfspaces, main_feasible_point)
            self.Y_outer = ConvexHull(self.outer_vertices)
            self.outer_vertices_up_to_date = True
        except QHullError:
            self.outer_vertices_up_to_date = False

    def build_polys(self):
        self.build_inner_poly()
        # self.build_outer_poly()
        if self.measure == Measure.VOLUME:
            self.build_outer_poly()
        if self.measure == Measure.SUPPORT:
            self.build_outer_poly()



    def find_new_direction(self):
        if self.measure == Measure.VOLUME:
            self.current_facet = max(self.measures.items(), key=operator.itemgetter(1))[0]
        if self.measure == Measure.SUPPORT:
            self.current_facet = max(self.measures.items(), key=operator.itemgetter(1))[0]
        if self.measure == Measure.FACET_AREA:
            self.current_facet = max(self.measures.items(), key=operator.itemgetter(1))[0]
        if self.measure == Measure.RANDOM:
            possible = [facet for facet in self.validity.keys() if self.validity[facet]]
            # print("Number of valid facet {}/{}".format(len(possible), len(self.validity.keys())))
            if len(possible)!=0:
                self.current_facet = choice(possible)
            else:
                self.current_facet = choice(self.validity.keys())

        self.new_dir = np.array(self.current_facet[:3]).reshape((3,1))

    def update_inner_poly(self):
        offset = np.matmul(self.new_dir.T, self.new_point)[0][0]
        EPS = 0.01 # -> the new point must be at least one cm away from the max facet

        if self.measure == Measure.FACET_AREA:
            if offset+self.current_facet[-1]<EPS:
                self.measures[self.current_facet]=0
            else:
                del self.measures[self.current_facet]

                self.directions.append(self.new_dir)
                self.inner_vertices.append(self.new_point)

                # compute the new self.error
                self.Y_inner.add_points(self.new_point.T)
                innerVolume = self.Y_inner.volume

        elif self.measure == Measure.RANDOM:
            if offset+self.current_facet[-1]<EPS:
                self.validity[self.current_facet]=False
            else:
                del self.validity[self.current_facet]

                self.directions.append(self.new_dir)
                self.inner_vertices.append(self.new_point)

                # compute the new self.error
                self.Y_inner.add_points(self.new_point.T)
                innerVolume = self.Y_inner.volume

        elif self.measure == Measure.VOLUME or self.measure == Measure.SUPPORT:
            del self.measures[self.current_facet]
            del self.facets_points[self.current_facet]
            del self.validity[self.current_facet]

            self.directions.append(self.new_dir)
            self.inner_vertices.append(self.new_point)

            # compute the new self.error
            self.Y_inner.add_points(self.new_point.T)
            innerVolume = self.Y_inner.volume

        else:
            raise ValueError("Unknown measure, please use a value supplied in enum")

    def update_outer_poly(self):
        offset = np.matmul(self.new_dir.T, self.new_point)[0][0]
        hs = np.array([self.new_dir[0][0], self.new_dir[1][0], self.new_dir[2][0], -offset])

        self.offsets.append(offset)
        self.halfspaces.append(hs.T)

        main_feasible_point = sum(self.inner_vertices)/len(self.inner_vertices)
        self.outer_vertices = ut.compute_outer_convex(self.halfspaces, main_feasible_point)
        self.Y_outer = ConvexHull(self.outer_vertices)

    def update_polys(self):
        self.update_inner_poly()
        # self.update_outer_poly()

        if self.measure == Measure.VOLUME:
            self.update_outer_poly()
        if self.measure == Measure.SUPPORT:
            self.update_outer_poly()

    def projection_robust_polyhedron(self):
        mainTimeStart = time.time()
        start = time.time()
        self.build_problem()

        # Choosing the initial directions
        initial_directions = [[[0], [0], [1.]]]
        n = 3
        for i in range(n):
            initial_directions.append([[np.cos(2*i*np.pi/n)], [np.sin(2*i*np.pi/n)], [-1]])

        self.init_time += time.time()-start
        # Finding the points corresponding to the initial direction
        for d in initial_directions:
            c = matrix(np.vstack((np.zeros((3*self.robot.n_feet*self.n_acc, 1)), -np.array(d))))

            start = time.time()
            result = self.solve_problem(c)
            end = time.time()
            self.SOCP_time += end-start

            v = np.array(result['x'][-3:])

            self.directions.append(np.array(d))
            self.inner_vertices.append(v)
            self.iterations += 1

        ########
        # Initialisation of inner and outer hulls
        ########
        start = time.time()
        self.build_polys()
        end = time.time()
        self.Convex_time += end-start

        ########
        # Computations of the differents volumes between the inner and outer approximation
        ########
        if self.measure != Measure.FACET_AREA:
            start = time.time()
            self.init_facet_validity()
            end = time.time()
            self.invalidate_time += end-start

        start = time.time()
        self.update_measures()
        end = time.time()
        self.measure_time += end-start
        ########
        # iterations
        ########
        while(self.stop_condition()):
            self.find_new_direction()

            # use socp to find the new point
            c = matrix(np.vstack((np.zeros((3*self.robot.n_feet*self.n_acc, 1)), -self.new_dir)))

            start = time.time()
            result = self.solve_problem(c)
            end = time.time()
            self.SOCP_time += end-start

            self.new_point = np.array(result['x'][-3:])

            # update the volumes for each facet
            start = time.time()
            self.update_polys()
            end = time.time()
            self.Convex_time += end-start

            start = time.time()
            self.check_facet_validity()
            end = time.time()
            self.invalidate_time += end - start

            start = time.time()
            self.update_measures()
            end = time.time()
            self.measure_time += end-start

            self.iterations += 1

        mainTimeStop = time.time()
        self.total_time  = mainTimeStop - mainTimeStart


    def prism_intersection_polyhedron(self):
        self.robot.set_P_matrix(np.array([[1, 0, 0],
                                          [0, 1, 0]]))

        self.halfspaces = [np.array([-1, 0, 0, -self.CoM_max[0]]), np.array([1, 0, 0, -self.CoM_max[0]]),
                            np.array([0, -1, 0, -self.CoM_max[1]]), np.array([0, 1, 0, -self.CoM_max[1]]),
                            np.array([0, 0, -1, -self.CoM_max[2]]), np.array([0, 0, 1, -self.CoM_max[2]])] # limitation of the CoM position

        disp = False
        if disp:
            # ax = plt.subplot(111, projection='3d')
            ax = plt.subplot(111)

        for acc in self.accelerations:
            self.robot.set_acceleration(acc)
            self.robot.update_static_model()

            static_poly = static_stability.static_stability_polyhedron(self.robot, self.eps, self.max_it, measure=static_stability.Measure.AREA, linearization=self.use_linearized_friction, friction_sides = self.friction_sides, mode = static_stability.Mode.best)
            static_poly.project_static_stability()

            if disp:
                x = [pt[0] for pt in static_poly.inner_vertices]
                x.append(x[0])
                y = [pt[1] for pt in static_poly.inner_vertices]
                y.append(y[0])
                ax.plot(x,y)

            for i in range(len(static_poly.inner_vertices)):
                pt1 = static_poly.inner_vertices[i-1]
                pt2 = static_poly.inner_vertices[i]

                dir_2d = np.zeros((3,1))
                dir_2d[:2] = pt2-pt1

                if np.linalg.norm(dir_2d)>10e-5:
                    hs_dir = np.cross(acc.reshape(3), dir_2d.reshape(3))
                    hs_dir /= np.linalg.norm(hs_dir)

                    pt=np.zeros((3,1))
                    pt[:2] = (pt1 + pt2)/2

                    hs_offset = np.matmul(hs_dir.T, pt)

                    self.halfspaces.append(np.hstack((hs_dir.T, -hs_offset)))

                    if disp:
                        scale = 0.1
                        dir_2d /= np.linalg.norm(dir_2d)
                        direc = static_poly.directions[i]
                        ax.arrow((pt2[0][0]+pt1[0][0])/2, (pt2[1][0]+pt1[1][0])/2, scale*dir_2d[0][0], scale*dir_2d[1][0], color='m')
                        ax.text(pt1[0][0], pt1[1][0], i)
                        ax.arrow(pt1[0][0], pt1[1][0], scale*direc[0][0], scale*direc[1][0], color = 'y')
                        # ax.quiver((pt2[0][0]+pt1[0][0])/2, (pt2[1][0]+pt1[1][0])/2, 0, scale*dir_2d[0][0], scale*dir_2d[1][0], 0, color='m')
                        # ax.quiver((pt2[0][0]+pt1[0][0])/2, (pt2[1][0]+pt1[1][0])/2, 0, scale*self.halfspaces[-1][0], scale*self.halfspaces[-1][1], scale*self.halfspaces[-1][2], color='c')
                        # ax.text(pt1[0][0], pt1[1][0], 0, i)
                        # ax.quiver(pt1[0][0], pt1[1][0], 0, scale*direc[0][0], scale*direc[1][0], 0, color = 'y')

        # Halfspace
        feasible_point = np.zeros((3,1))
        feasible_point[:2] = sum(static_poly.inner_vertices)/len(static_poly.inner_vertices)
        if disp:
            ax.plot([feasible_point[0]], [feasible_point[1]], 'yo')
        # print("Composant z:", feasible_point[2])
        try:
            HS = HalfspaceIntersection(np.array(self.halfspaces), feasible_point.reshape(3))
        except:
            print("first try failled")
            c = np.zeros((4,))
            c[-1] = -1
            A = np.hstack((np.array(self.halfspaces)[:, :-1], np.ones((len(self.halfspaces), 1))))
            b = -np.array(self.halfspaces)[:, -1:]
            res = linprog(c, A_ub=A, b_ub=b, bounds=[None, None])

            feasible_point = np.array(res['x'][:3])
            HS = HalfspaceIntersection(np.array(self.halfspaces), feasible_point.reshape(3))

        if disp:
            ax.plot([feasible_point[0]], [feasible_point[1]], 'ko')
            plt.axis('scaled')
            plt.show()

        # print("Composant z:", feasible_point[2])

        # Convex
        self.inner_vertices = HS.intersections
        self.Y_inner = ConvexHull(self.inner_vertices)
        self.robot.set_P_matrix()
        self.robot.reset_gravity()
        self.robot.update_static_model()

