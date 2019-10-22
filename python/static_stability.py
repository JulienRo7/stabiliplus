#!/usr/bin/env python3

import robot_description
import utils as ut
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import numpy as np
from cvxopt import matrix
from random import randint, choice

import cvxopt.solvers
cvxopt.solvers.options['glpk'] = {'msg_lev': 'GLP_MSG_OFF'}  # to turn off the displays
cvxopt.solvers.options['show_progress'] = False

import time

from enum import Enum, unique
# from scipy.optimize import linprog
@unique
class Measure(Enum):
    """ Enum that allows to choose the desired type of measure"""
    RANDOM = 0
    AREA = 1
    SIDE_LENGTH = 2

@unique
class Mode(Enum):
  """Enum that shows the three modes of functionment."""
  precision = 1
  iteration = 2
  best = 3

class static_stability_polyhedron():

    def __init__(self, robot, eps=0.01, max_it=100, measure=Measure.AREA, linearization=False, friction_sides = 8, mode = Mode.best):

        self.robot = robot

        # definition of the inner and outer hull. The vertices are given in counter clockwise order
        self.inner_vertices = []
        self.directions = []
        self.outer_vertices = []
        self.measures = []
        # self.Polyhedron_inner = []
        # self.Polyhedron_outer = []

        self.iterations = 0
        self.error = 1000

        self.new_dir = np.array([[1],[1]])
        self.new_point = np.array([[1],[1]])
        self.current_side = 0

        self.measure = measure
        self.use_linearized_friction = linearization
        assert friction_sides >= 3
        self.friction_sides = friction_sides

        # stop criterions
        assert eps > 0 and max_it > 0
        self.eps = eps
        self.max_it = max_it
        self.alpha0 = 1

        self.set_stop_condition(mode)

        self.problem_initialized = False

    def set_stop_condition(self, mode):
        # self.stop_condition = False
        if (mode == Mode.precision):
            if self.measure == Measure.AREA:
                self.stop_condition = lambda: self.error > self.eps
            else:
                est_iteration = int(3*(np.sqrt(1.412*self.alpha0/self.eps)-1))# -> this criteria is very rough but fast and simple
                self.max_it = min(self.max_it, est_iteration)
                self.stop_condition = lambda: self.iterations < self.max_it

        elif(mode == Mode.iteration):
            self.stop_condition = lambda: self.iterations < self.max_it
        elif(mode == Mode.best):
            if self.measure == Measure.AREA:
                self.stop_condition = lambda: (self.error > self.eps) and (self.iterations < self.max_it)
            else:
                est_iteration = int(3*(np.sqrt(1.412*self.alpha0/self.eps)-1))# -> this criteria is very rough but fast and simple
                self.max_it = min(self.max_it, est_iteration)
                self.stop_condition = lambda: self.iterations < self.max_it
            self.stop_condition = lambda: (self.error > self.eps) and (self.iterations < self.max_it)
        else:
            raise ValueError("Unknown mode, please use a value supplied in enum")

    def init_SOCP_problem(self):

        N_optim = 3*self.robot.n_feet+2  # dimension of the optimization problem

        # Linear equalities
        self.A = matrix(np.hstack([self.robot.A1, self.robot.A2]))
        self.b = matrix(self.robot.t)

        # linear constraints to bound the forces: Gl * x + s0 = hl and s0 >= 0
        # -Gl * x <= -hl
        f_max = self.robot.masse * np.linalg.norm(self.robot.gravity)
        self.Gl = np.zeros((2*self.robot.n_feet, N_optim))
        self.hl = np.zeros((2*self.robot.n_feet, 1))

        self.Gk = []
        self.hk = []

        for i in range(self.robot.n_feet):
            R = np.array(self.robot.feet[i][:3, :3])
            # Linear constraints
            self.Gl[2*i, 3*i:3*i+3] = np.matmul(np.array([0, 0, 1]), R.T)
            self.Gl[2*i+1, 3*i:3*i+3] = np.matmul(np.array([0, 0, -1]), R.T)
            self.hl[2*i] = f_max

            # Cone constraints
            Gi = np.zeros((3, N_optim))
            F = np.array([[0, 0, self.robot.mu[i]], [1, 0, 0], [0, 1, 0]])
            Gi[:, 3*i:3*(i+1)] = np.matmul(F, R.T)
            hi = np.zeros(3)

            self.Gk.append(matrix(-Gi))
            self.hk.append(matrix(hi))

        self.Gl = matrix(self.Gl)
        self.hl = matrix(self.hl)
        # Gk = matrix(Gk)
        # hk = matrix(hk)
        self.problem_initialized = True

    def init_LP_problem(self):
        N_optim = 3*self.robot.n_feet+2
        self.A = matrix(np.hstack([self.robot.A1, self.robot.A2]))
        self.b = matrix(self.robot.t)

        self.G = np.zeros(((self.friction_sides+2)*self.robot.n_feet, N_optim))
        self.h = np.zeros((self.friction_sides+2)*self.robot.n_feet)

        sqrt2 = np.sqrt(2)

        for i in range(self.robot.n_feet):

            Gi, hi = self.linearized_friction_cone(
                self.friction_sides, self.robot.mu[i], self.robot.masse*np.linalg.norm(self.robot.gravity))

            hi = hi.reshape((self.friction_sides + 2))

            R = np.array(self.robot.feet[i][:3, :3])
            self.G[(self.friction_sides + 2)*i:(self.friction_sides + 2)*(i+1), 3*i:3*(i+1)] = np.matmul(Gi, R.T)
            self.h[(self.friction_sides + 2)*i:(self.friction_sides + 2)*(i+1)] = hi

        self.G = matrix(self.G)
        self.h = matrix(self.h)

        self.problem_initialized = True

    def init_problem(self):
        if not self.use_linearized_friction:
            self.init_SOCP_problem()
        else:
            self.init_LP_problem()

    def solve_SOCP_problem(self, c):
        return cvxopt.solvers.socp(c, self.Gl, self.hl, self.Gk, self.hk, self.A, self.b)

    def solve_LP_problem(self, c):
        return cvxopt.solvers.lp(c, self.G, self.h, self.A, self.b, solver='glpk')

    def solve_problem(self, c):
        if not self.problem_initialized:
            self.init_problem()

        if not self.use_linearized_friction:
            return self.solve_SOCP_problem(c)
        else:
            return self.solve_LP_problem(c)

    def compute_outer_vertices(self):
        for i in range(len(self.inner_vertices)):
            D = np.vstack((self.directions[i-1].T, self.directions[i].T))
            e = np.vstack((np.matmul(self.directions[i-1].T, self.inner_vertices[i-1]),
                           np.matmul(self.directions[i].T, self.inner_vertices[i])))

            self.outer_vertices.append(np.matmul(np.linalg.inv(D), e))

    def compute_inner_outer_areas(self, index):

        return ut.triangleArea( [self.inner_vertices[index-1], self.inner_vertices[index], self.outer_vertices[index]])

    def compute_measures(self):
        for i in range(len(self.inner_vertices)):
            self.update_measure(i)

    def update_measure(self, index):
        if self.measure == Measure.AREA:
            mes = self.compute_inner_outer_areas(index)

        if self.measure == Measure.SIDE_LENGTH:
            if np.matmul(self.new_dir.T, self.new_point-self.inner_vertices[self.current_side-1])>0.001:
                mes = np.linalg.norm(self.inner_vertices[index-1]-self.inner_vertices[index])
            else :
                mes = 0
        if self.measure == Measure.RANDOM:
            if np.matmul(self.new_dir.T, self.new_point-self.inner_vertices[self.current_side-1])>0.001:
                mes = 1
            else:
                mes = 0

        if len(self.measures)==len(self.inner_vertices):
            self.measures[index] = mes
        elif index == len(self.measures)+1:
            self.measures.append(mes)
        else:
            self.measures.insert(index, mes)

    def find_new_direction(self):
        if self.measure == Measure.AREA:
            self.current_side = self.measures.index(max(self.measures))

        if self.measure == Measure.SIDE_LENGTH:
            if sum(self.measures) == 0:
                self.current_side = -1
                return True

            self.current_side = self.measures.index(max(self.measures))

        if self.measure == Measure.RANDOM:
            possible = [i for i in range(len(self.measures)) if self .measures[i] == 1]
            if len(possible)!=0:
                self.current_side = choice(possible)
            else:
                # self.current_side = randint(0, len(self.measures)-1)
                self.current_side = -1
                return True

        p1 = self.inner_vertices[self.current_side-1]
        p2 = self.inner_vertices[self.current_side]
        perp = p2-p1
        perp /= np.linalg.norm(perp)

        self.new_dir = np.reshape(np.array([[perp[1]], [-perp[0]]]), [2, 1])

    def update_inner(self):
        self.directions.insert(self.current_side, self.new_dir)
        self.inner_vertices.insert(self.current_side, self.new_point)

    def update_outer(self):
        # computation of the new points of the outer polyhedron and aeras

        d1 = self.directions[self.current_side-1]
        p1 = self.inner_vertices[self.current_side-1]

        if self.current_side == len(self.inner_vertices):
            d2 = self.directions[-1]
            p2 = self.inner_vertices[-1]
        else:
            d2 = self.directions[self.current_side+1]
            p2 = self.inner_vertices[self.current_side+1]

        # i = areaMax for simplification
        D_i = np.vstack((d1.T, self.new_dir.T))
        e_i = np.vstack((np.matmul(d1.T, p1), np.matmul(self.new_dir.T, self.new_point)))
        newVertex_i = np.matmul(np.linalg.inv(D_i), e_i)

        D_ip1 = np.vstack((self.new_dir.T, d2.T))
        e_ip1 = np.vstack((np.matmul(self.new_dir.T, self.new_point), np.matmul(d2.T, p2)))
        newVertex_ip1 = np.matmul(np.linalg.inv(D_ip1), e_ip1)

        self.outer_vertices[self.current_side] = newVertex_i
        self.outer_vertices.insert(self.current_side+1, newVertex_ip1)

    def update_polys(self):
        self.update_inner()
        self.update_outer()

    def project_static_stability(self):
        # Compute the static stbility polyhedron based on the method presented by Bretl and Lall

        # initilization of the problem
        self.init_problem()

        # Initialization of the first points and search direction of the algorithm
        # np.random.seed(0)
        theta = np.pi/2
        while len(self.inner_vertices) < 3:
            d = np.reshape(np.array([np.cos(theta), np.sin(theta)]), [2, 1])
            c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), -d)))

            result = self.solve_problem(c)
            v = np.array(result['x'][-2:])

            self.directions.append(d)
            self.inner_vertices.append(v)

            theta += 2*np.pi/3
            self.iterations += 1

        # compute the outer vertices and the ares
        self.compute_outer_vertices()
        self.compute_measures()

        # stop criterion
        self.error = sum(self.measures)

        self.alpha0 = 3*ut.triangleArea(self.inner_vertices)

        #########################################
        # Iterations
        #########################################
        while self.stop_condition():

            self.find_new_direction()
            if self.current_side == -1:
                return True

            # Finding the new point
            c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), -self.new_dir)))

            result = self.solve_problem(c)
            self.new_point = np.array(result['x'][-2:])

            self.update_polys()
            self.update_measure(self.current_side)
            self.update_measure(self.current_side+1)

            self.error = sum(self.measures)
            self.iterations += 1


    def project_static_stability_timed(self):
        # Compute the static stbility polyhedron based on the method presented by Bretl and Lall
        # with timers

        init_time = 0
        convex_time = 0
        socp_time = 0
        measure_time = 0

        # initilization of the problem
        start = time.time()
        self.init_problem()
        end = time.time()
        init_time += end-start

        # Initialization of the first points and search direction of the algorithm
        # np.random.seed(0)
        theta = np.pi/2
        while len(self.inner_vertices) < 3:
            d = np.reshape(np.array([np.cos(theta), np.sin(theta)]), [2, 1])
            c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), -d)))

            start = time.time()
            result = self.solve_problem(c)
            end = time.time()
            v = np.array(result['x'][-2:])
            socp_time += end-start

            self.directions.append(d)
            self.inner_vertices.append(v)

            theta += 2*np.pi/3
            self.iterations += 1



        # compute the outer vertices and the ares
        start = time.time()
        self.compute_outer_vertices()
        end = time.time()
        convex_time += end-start

        start = time.time()
        self.compute_measures()
        end = time.time()
        measure_time += end-start

        # stop criterion
        self.error = sum(self.measures)

        self.alpha0 = 3*ut.triangleArea(self.inner_vertices)

        #########################################
        # Iterations
        #########################################
        while self.stop_condition():

            self.find_new_direction()
            if self.current_side== -1:
                return (init_time, convex_time, socp_time, measure_time)

            # Finding the new point
            c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), -self.new_dir)))

            start = time.time()
            result = self.solve_problem(c)
            end = time.time()
            self.new_point = np.array(result['x'][-2:])
            socp_time += end-start

            start = time.time()
            self.update_polys()
            end = time.time()
            convex_time += end-start

            start = time.time()
            self.update_measure(self.current_side)
            self.update_measure(self.current_side+1)
            end = time.time()
            measure_time += end-start

            self.error = sum(self.measures)
            self.iterations += 1

        return (init_time, convex_time, socp_time, measure_time)

    def static_stability_honda(self):
        # initilization of the LP problem
        self.init_problem()

        # research the min and max point following the x axis:
        dx = np.array([[1], [0]])
        dy = np.array([[0], [1]])

        c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), -dx)))
        result = self.solve_problem(c)
        x_max = np.array(result['x'][-2:])
        self.iterations += 1

        c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), dx)))
        result = self.solve_problem(c)
        x_min = np.array(result['x'][-2:])
        self.iterations += 1

        # discretization of the x axis
        N_discretization = 30
        X = np.linspace(x_min[0], x_max[0], N_discretization)
        Y_min = []
        Y_max = []
        A_bis = np.vstack((np.array(A), np.zeros((1, 3*self.robot.n_feet+2))))
        A_bis[-1, -2] = 1
        b_bis = np.vstack((np.array(b), np.zeros((1, 1))))
        for i in range(N_discretization):
            # Adding a constraint: x = X[i]
            b_bis[-1] = X[i]

            self.A = matrix(A_bis)
            self.b = matrix(b_bis)
            c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), -dy)))
            result = self.solve_problem(c)
            Y_max.append(np.array(result['x'][-2:]))
            self.iterations += 1

            c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), dy)))
            result = self.solve_problem(c)
            Y_min.append(np.array(result['x'][-2:]))
            self.iterations += 1

        self.inner_vertices.append(x_min)
        self.inner_vertices.extend(Y_min)
        self.inner_vertices.append(x_max)
        Y_max.reverse()
        self.inner_vertices.extend(Y_max)

    def display_polyhedron(self):
        x_inner = [v[0] for v in self.inner_vertices]
        y_inner = [v[1] for v in self.inner_vertices]

        x_outer = [v[0] for v in self.outer_vertices]
        y_outer = [v[1] for v in self.outer_vertices]

        sup = self.robot.support_polygon()

        ax = plt.subplot('111')
        ax.plot(sup[0], sup[1])
        ax.plot(x_inner, y_inner)
        ax.plot(x_outer, y_outer)

        for v, d in zip(self.inner_vertices, self.directions):
            print("d: ", d)
            ax.arrow(v[0][0], v[1][0], d[0][0], d[1][0], color='k')

        plt.axis('scaled')
        plt.show()

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
            # *np.cos(Dtheta/2) -> to have the restrictive linearized friction cone
            G[2+i, 2] = -mu

        return G, h

####################################################
# Old version of the different functions
####################################################

# j = [v[0] for v in self.vertex]
# k = [v[1] for v in self.vertex]
#
# sup = self.robot.support_polygon()
#
# ax = plt.subplot('111')
# ax.plot(sup[0], sup[1])
# ax.plot(j, k)
# ax.plot(feasible_point[0], feasible_point[1], 'ko')
#
# for i in range(len(self.vertex)):
#     d = self.directions[i]
#     print(j[i][0], k[i][0], d[0][0], d[1][0])
#     ax.arrow(j[i][0], k[i][0], d[0][0], d[1][0], color='y')

# def project_static_stability(self):
#     # Compute the static stbility polyhedron based on the method presented by Bretl and Lall
#
#     # initilization of the SOCP problem
#     c = matrix(-self.robot.t)
#
#     A = matrix(self.robot.A2)
#     A = matrix(A.T)
#
#     Gl = matrix(np.zeros((3, 6)))
#     hl = matrix(np.zeros([3, 1]))
#     Gq = []
#     hq = []
#     for i in range(self.robot.n_feet):
#         Pi = np.matmul(np.array(self.robot.feet[i][:3, :3]), np.array([[0, 1, 0],
#                                                                        [0, 0, 1],
#                                                                        [1/self.robot.mu[i], 0, 0]]))
#
#         Gi = np.zeros((6, 3))
#         Gi[:6, :] = np.matmul(np.array(self.robot.A1[:, 3*i:3*i+3]), Pi)
#
#         Gq.append(matrix(Gi.T))
#         hq.append(matrix(np.zeros([3, 1])))
#
#     # directions = []
#     # vertex = []
#     # forces = []
#     # fails = 0
#
#     # Initialization of the first points and search direction of the algorithm
#     # np.random.seed(0)
#     theta = np.random.random()*np.pi
#     while len(self.vertex) < 3:
#         # for d in initDirections:
#         d = np.reshape(np.array([np.cos(theta), np.sin(theta)]), [2, 1])
#         b = matrix(-d)
#
#         result = cvxopt.solvers.socp(c, Gl, hl, Gq, hq, A, b)
#         # print(result)
#
#         self.directions.append(d)
#         self.vertex.append(np.array(list(result['y'])))
#         # forces.append(np.array(list(result['zl'])))
#
#         theta += 2
#
#     # Computation of Y_inner using ConvexHull from Scipy.spatial
#     Y_inner = ConvexHull(np.array(self.vertex))
#     innerArea = Y_inner.area
#
#     # Computation of Y_outer using HalfspaceIntersection from Scipy.spatial which is the dual of ConvexHull
#     halfspaces = []
#     for i in range(len(self.vertex)):
#         d = self.directions[i].T.tolist()[0]
#         halfspaces.append([d[0], d[1], -np.matmul(self.directions[i].T, self.vertex[i])[0]])
#
#     # the barycenter of the vertex is in the convex thus it is a feasible point
#     feasible_point = sum(self.vertex)/len(self.vertex)
#     Y_outer = HalfspaceIntersection(np.array(halfspaces), feasible_point)
#     outerArea = Y_outer.dual_area
#
#     # stop criterion
#     eps = 0.01
#     self.iterations = 0
#     max_iter = 100
#
#     #########################################
#     # Iterations
#     #########################################
#     while (np.abs(outerArea-innerArea) > eps) and (self.iterations < max_iter):
#
#         areas = []
#         for i in range(len(Y_inner.vertices)):
#             v1 = np.reshape(self.vertex[Y_inner.vertices[i-1]], (2, 1))
#             v2 = np.reshape(self.vertex[Y_inner.vertices[i]], (2, 1))
#
#             mid = (v2-v1)/2
#             perp = mid/np.linalg.norm(mid)
#             dir = np.array([perp[1], -perp[0]])
#
#             area = 0
#             v = v1
#             for v3 in Y_outer.intersections:
#                 v4 = np.reshape(v3, (2, 1))
#                 if np.matmul(dir.T, (v4-v2))[0][0] > 0:
#                     s = ut.triangleArea([v1, v2, v4])
#                     if s > area:
#                         area = s
#                         v = v4
#
#             areas.append(np.reshape(area, (1,))[0])
#
#         # print(areas)
#         areaMax = areas.index(max(areas))
#         # print("Area Max: ", areaMax)
#
#         # Choosing  direction
#         p1 = np.reshape(self.vertex[Y_inner.vertices[areaMax-1]], [2, 1])
#         p2 = np.reshape(self.vertex[Y_inner.vertices[areaMax]], [2, 1])
#         perp = p2-p1
#         perp /= np.linalg.norm(perp)
#
#         newDir = np.reshape(np.array([[perp[1]], [-perp[0]]]), [2, 1])
#
#         # add point
#         # print(newDir)
#         b = matrix(-newDir)
#         result = cvxopt.solvers.socp(c, Gl, hl, Gq, hq, A, b)
#
#         self.directions.append(newDir)
#         v = np.array(list(result['y']))
#         self.vertex.append(v)
#         # forces.append(np.array(list(result['zl'])))
#         feasible_point = sum(self.vertex)/len(self.vertex)
#
#         # update outer
#         hs = [newDir.tolist()[0][0], newDir.tolist()[1][0], -np.matmul(newDir.T, v)[0]]
#         halfspaces.append(hs)
#
#         Y_outer = HalfspaceIntersection(np.array(halfspaces), feasible_point)
#         outerHull = ConvexHull(Y_outer.intersections)
#         outerArea = outerHull.area
#
#         # update inner
#         Y_inner = ConvexHull(np.array(self.vertex))
#         innerArea = Y_inner.area
#         # print(innerArea)
#
#         self.iterations += 1
#
#     self.Polyhedron_inner = Y_inner
#     self.Polyhedron_outer = Y_outer
#     self.error = outerArea-innerArea
#     self.area_error = 2*(outerArea-innerArea)/(outerArea+innerArea)
#
# def random_static_stability(self):
#     # Compute the static stbility polyhedron based on the method presented by Bretl and Lall
#     # initilization of the SOCP problem
#
#     self.init_problem()
#
#     # Initialization of the first points and search direction of the algorithm
#     # np.random.seed(0)
#     theta = np.random.random()*np.pi
#     while len(self.inner_vertices) < 3:
#         d = np.reshape(np.array([np.cos(theta), np.sin(theta)]), [2, 1])
#         c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), -d)))
#
#         result = self.solve_problem(c)
#         v = np.array(result['x'][-2:])
#
#         self.directions.append(d)
#         self.inner_vertices.append(v)
#
#         theta += 2*np.pi/3
#
#     # Computing the perimeter as a stop criterion: the perimeter should be smaller than the perimeter of the convex that we try to approximate and increasing/ monotonous thus it should converge... -> to be prouved
#     self.area = 0  # -> will actually contain the sum of the squares of the distance between consecutive vertices of the convex hull
#     for i in range(len(self.inner_vertices)):
#         side = self.inner_vertices[i]-self.inner_vertices[i-1]
#         self.area += side[0]*side[0] + side[1]*side[1]
#
#     self.error = self.area
#
#     #########################################
#     # Iterations
#     #########################################
#
#     # stop criterion
#     self.iterations = 3  # to actually count the number of SOCP
#     stop = self.eps**2
#
#     while (self.error > stop) and (self.iterations < self.max_it):
#
#         previous_length = 0
#         cpt = 0
#         while previous_length < 0.0001 and cpt < 100:
#             areaMax = randint(0, len(self.inner_vertices)-1)
#             p1 = self.inner_vertices[areaMax-1]
#             p2 = self.inner_vertices[areaMax]
#             side = p2-p1
#             previous_length = side[0]*side[0] + side[1]*side[1]
#             cpt += 1
#
#         # Choosing  direction
#         perp = side/np.sqrt(previous_length)
#
#         newDir = np.array([[perp[1][0]], [-perp[0][0]]])
#
#         # Finding the new point
#         c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), -newDir)))
#
#         result = self.solve_problem(c)
#         v = np.array(result['x'][-2:])
#
#         self.inner_vertices.insert(areaMax, v)
#
#         # update of the stop criterion
#         side1 = v-p1
#         side2 = p2-v
#         new_length = side1[0]*side1[0] + side1[1] * \
#             side1[1] + side2[0]*side2[0] + side2[1]*side2[1]
#         self.area -= previous_length
#         self.area += new_length
#
#         self.error = np.abs(new_length - previous_length)
#
#         self.iterations += 1
#
# def max_length_static_stability(self):
#     # Compute the static stbility polyhedron based on the method presented by Bretl and Lall
#
#     # initilization of the SOCP problem
#     self.init_problem()
#
#     # Initialization of the first points and search direction of the algorithm
#     # np.random.seed(0)
#     # theta = np.random.random()*np.pi
#     theta = np.pi/2
#     while len(self.inner_vertices) < 3:
#         d = np.reshape(np.array([np.cos(theta), np.sin(theta)]), [2, 1])
#         c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), -d)))
#
#         result = self.solve_problem(c)
#         v = np.array(result['x'][-2:])
#
#         self.directions.append(d)
#         self.inner_vertices.append(v)
#
#         theta += 2*np.pi/3
#
#     #
#     sides_lenghts_squared = []
#     for i in range(len(self.inner_vertices)):
#         side = self.inner_vertices[i]-self.inner_vertices[i-1]
#         sides_lenghts_squared.append(side[0]*side[0] + side[1]*side[1])
#
#     self.area = sum(sides_lenghts_squared)
#
#     #########################################
#     # Iterations
#     #########################################
#
#     # stop criterion
#     self.iterations = 3  # to actually count the number of SOCP
#     stop = self.eps**2
#
#     while (max(sides_lenghts_squared) > self.eps) and (self.iterations < self.max_it):
#         # print(self.iterations)
#         # print("Side Lengths :", sides_lenghts_squared)
#         areaMax = sides_lenghts_squared.index(max(sides_lenghts_squared))
#
#         # Choosing  direction
#         p1 = self.inner_vertices[areaMax-1]
#         p2 = self.inner_vertices[areaMax]
#         side = p2-p1
#         previous_length = sides_lenghts_squared[areaMax]
#         perp = side/np.sqrt(previous_length)
#
#         # print("previous_length :", previous_length)
#
#         newDir = np.array([[perp[1][0]], [-perp[0][0]]])
#
#         # Finding the new point
#         c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), -newDir)))
#
#         result = self.solve_problem(c)
#         v = np.array(result['x'][-2:])
#
#         # update of the stop criterion
#         side1 = v-p1
#         side2 = p2-v
#         length1 = side1[0]*side1[0] + side1[1]*side1[1]
#         length2 = side2[0]*side2[0] + side2[1]*side2[1]
#
#         # print("New Lengths : ", length1, length2)
#
#         ratio = length1/length2
#         # print("Ratio :", ratio)
#         # variation = np.abs(length1 + length2 - previous_length)/previous_length
#         # print("Variation :", variation)
#         if ratio > 0.001 and ratio < 1000 and np.matmul(side1.T, newDir)[0] > self.eps:
#             self.area -= previous_length
#             self.area += length1 + length2
#             self.inner_vertices.insert(areaMax, v)
#             sides_lenghts_squared.insert(areaMax, length1)
#             sides_lenghts_squared[areaMax+1] = length2
#         else:
#
#             self.inner_vertices.insert(areaMax, v)
#             sides_lenghts_squared[areaMax] = 0
#             sides_lenghts_squared.insert(areaMax, 0)
#             # print("Refused Ratio :", ratio)
#
#         self.iterations += 1

# def support_point_static_stability(self):
#     # Compute the static stbility polyhedron based on the method presented by Bretl and Lall
#
#     # initilization of the SOCP problem
#
#     N_optim = 3*self.robot.n_feet+2  # dimension of the optimization problem
#
#     # Linear equalities
#     A = matrix(np.hstack([self.robot.A1, self.robot.A2]))
#     b = matrix(self.robot.t)
#
#     # linear constraints to bound the forces: Gl * x + s0 = hl and s0 >= 0
#     # -Gl * x <= -hl
#     Gl = np.zeros((self.robot.n_feet, N_optim))
#     for i in range(self.robot.n_feet):
#         Gl[i, 3*i+2] = 1
#
#     f_max = self.robot.masse * np.linalg.norm(self.robot.gravity)
#     hl = -f_max*np.ones((self.robot.n_feet, 1))
#
#     Gl = matrix(-Gl)
#     hl = matrix(-hl)
#     # Gl = matrix(np.zeros((1, N_optim)))
#     # hl = matrix(np.zeros((1, 1)))
#
#     # Cone constraints
#     Gk = []
#     hk = []
#
#     for i in range(self.robot.n_feet):
#         Gi = np.zeros((3, N_optim))
#         R = np.array(self.robot.feet[i][:3, :3])
#         F = np.array([[0, 0, self.robot.mu[i]], [1, 0, 0], [0, 1, 0]])
#
#         Gi[:, 3*i:3*(i+1)] = np.matmul(F, R.T)
#
#         hi = np.zeros(3)
#
#         Gk.append(matrix(-Gi))
#         hk.append(matrix(-hi))
#
#     # Gk = matrix(Gk)
#     # hk = matrix(hk)
#
#     # Initialization of the first points and search direction of the algorithm
#     # np.random.seed(0)
#     theta = np.pi/2
#     while len(self.inner_vertices) < 3:
#         d = np.reshape(np.array([np.cos(theta), np.sin(theta)]), [2, 1])
#         c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), -d)))
#
#         result = cvxopt.solvers.socp(c, Gl, hl, Gk, hk, A, b)
#         v = np.array(result['x'][-2:])
#
#         self.directions.append(d)
#         self.inner_vertices.append(v)
#
#         theta += 2*np.pi/3
#
#     # compute the outer vertices and the ares
#     for i in range(3):
#         D = np.vstack((self.directions[i-1].T, self.directions[i].T))
#         e = np.vstack((np.matmul(self.directions[i-1].T, self.inner_vertices[i-1]),
#                        np.matmul(self.directions[i].T, self.inner_vertices[i])))
#
#         # print(D, e)
#
#         self.outer_vertices.append(np.matmul(np.linalg.inv(D), e))
#
#         self.measures.append(2*ut.triangleArea(
#             [self.inner_vertices[i-1], self.inner_vertices[i], self.outer_vertices[-1]])/(np.linalg.norm(self.inner_vertices[i-1]-self.inner_vertices[i])))
#
#     # print("inner vertices :", self.inner_vertices)
#     # print("outer vertices :", self.outer_vertices)
#     # print("areas :", self.measures)
#
#     # stop criterion
#     self.iterations = 3  # to actually count the number of SOCP
#     self.error = sum(self.measures)
#
#     #########################################
#     # Iterations
#     #########################################
#     while (self.error > self.eps) and (self.iterations < self.max_it):
#
#         # Finding the max area:
#         areaMax = self.measures.index(max(self.measures))
#         # print("Area Max: ", areaMax)
#
#         # Choosing  direction
#         p1 = self.inner_vertices[areaMax-1]
#         p2 = self.inner_vertices[areaMax]
#         perp = p2-p1
#         perp /= np.linalg.norm(perp)
#
#         newDir = np.reshape(np.array([[perp[1]], [-perp[0]]]), [2, 1])
#
#         # Finding the new point
#         c = matrix(np.vstack((np.zeros((3*self.robot.n_feet, 1)), -newDir)))
#
#         result = cvxopt.solvers.socp(c, Gl, hl, Gk, hk, A, b)
#         v = np.array(result['x'][-2:])
#
#         # computation of the new points of the outer polyhedron and aeras
#         d1 = self.directions[areaMax-1]
#         d2 = self.directions[areaMax]
#
#         # i = areaMax for simplification
#         D_i = np.vstack((d1.T, newDir.T))
#         e_i = np.vstack((np.matmul(d1.T, p1), np.matmul(newDir.T, v)))
#         newVertex_i = np.matmul(np.linalg.inv(D_i), e_i)
#
#         # newSupport_i =
#
#         D_ip1 = np.vstack((newDir.T, d2.T))
#         e_ip1 = np.vstack((np.matmul(newDir.T, v), np.matmul(d2.T, p2)))
#         newVertex_ip1 = np.matmul(np.linalg.inv(D_ip1), e_ip1)
#         newSupport_ip1 = np.matmul((newVertex_i-p1).T, newDir)
#
#         self.directions.insert(areaMax, newDir)
#         self.inner_vertices.insert(areaMax, v)
#
#         self.outer_vertices[areaMax] = newVertex_i
#         self.measures[areaMax] = newArea_i
#
#         self.outer_vertices.insert(areaMax+1, newVertex_ip1)
#         self.measures.insert(areaMax+1, newArea_ip1)
#
#         self.error = sum(self.measures)
#         self.iterations += 1
