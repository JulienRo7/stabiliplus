#!/usr/bin/env python3

import numpy as np
from cvxopt import matrix
from cvxopt.solvers import socp
from scipy.spatial import ConvexHull, HalfspaceIntersection
# from scipy.optimize import linprog

# import cdd

import matplotlib.pyplot as plt


def antisymMatrix(z):
    # return  antisymetric matrix T associated with z
    assert z.shape == (3, 1) or z.shape == (3,)
    z_1d_array = z if z.shape == (3,) else z.reshape(3)
    T = np.array([[0, -z_1d_array[2], z_1d_array[1]],
                  [z_1d_array[2], 0, -z_1d_array[0]],
                  [-z_1d_array[1], z_1d_array[0], 0]])
    return T


def euler2RotMat(phi, theta, psi):
    # return the rotation matrix corresponding to the rotation defined by the Euler angles phi, theta, psi
    # phi around x_0, theta around z_1 and psi around x_2

    # from here: phi -> 1, theta -> 2, psi -> 3 and ci = cos(i) and si = sin(i)
    c1 = np.cos(phi)
    s1 = np.sin(phi)

    c2 = np.cos(theta)
    s2 = np.sin(theta)

    c3 = np.cos(psi)
    s3 = np.sin(psi)

    rotMat = np.eye(3)

    rotMat[0, 0] = c2
    rotMat[0, 1] = -s2*c3
    rotMat[0, 2] = s2*s3
    rotMat[1, 0] = c1*s2
    rotMat[1, 1] = c1*c2*c3 - s1*s3
    rotMat[1, 2] = -c1*c2*s3 - s1*c3
    rotMat[2, 0] = s1*s2
    rotMat[2, 1] = s1*c2*c3 + c1*s3
    rotMat[2, 2] = -s1*c2*s3 + c1*c3

    return rotMat


def triangleArea(triangle):
    # return the area of the triangle discribed by the position of its corners
    # 5 substractions and 3 multiplications
    p1, p2, p3 = triangle
    c1 = p2 - p1
    c2 = p3 - p1
    # area computed from the cross product of c1 and c2
    # area computed from the cross product of c1 and c2
    return np.abs(c1[0]*c2[1]-c1[1]*c2[0])/2


def compute_outer(Y_inner, vertex, directions):
    # return the outer Hull given the inner Hull, the ve rtex and the search directions
    outerVertex = []
    for i in range(len(Y_inner.vertices)):
        # get two direction and the corresponding vertex
        d1 = directions[Y_inner.vertices[i-1]]
        v1 = np.reshape(vertex[Y_inner.vertices[i-1]], [2, 1])
        d2 = directions[Y_inner.vertices[i]]
        v2 = np.reshape(vertex[Y_inner.vertices[i]], [2, 1])

        # find the intersection of the line passing through v1 (resp. v2) and perpandicular to d1 (resp. d2)
        D = np.array([d1.T, d2.T])
        D = np.reshape(D, [2, 2])

        dv = np.array([[np.matmul(d1.T, v1)], [np.matmul(d2.T, v2)]])
        dv = np.reshape(dv, [2, 1])

        outerVertex.append(np.matmul(np.linalg.inv(D), dv))

    return outerVertex


def display_halfspace(hs, ax, begin=-1, end=1, pts=3):
    # Plot the halfspace hs given under the form [A, b] in the graph ax
    x = np.linspace(begin, end, pts)
    # plot the limit of the half space
    if hs[1] == 0:
        ax.axvline(-hs[2]/hs[0])
        y = 0
    else:
        ax.plot(x, (-hs[2]-hs[0]*x)/hs[1], 'm-')
        y = (-hs[2]-hs[0]*x[pts//2])/hs[1]

    # plot the direction of the halfspaces

    ax.arrow(x[pts//2], y, hs[0], hs[1], color='m')


def pointinHS(point, HS, eps =0):

    ps = 0
    dim = len(HS)-1
    for i in range(dim):
        ps += point[i]*HS[i]

    return ps + HS[-1] <= eps

def pointInHull(point, hull):
    # Tell if the point is in the hull

    for eq in hull.equations:
        if not pointinHS(point, eq):
            return False

    return True



def computeError(hull1, hull2):
    # compute the symmetric difference between hull1 and hull2

    vertices_intersection = []
    for v in hull1.points:
        if pointInHull(v, hull2):
            vertices_intersection.append(v)

    for v in hull2.points:
        if pointInHull(v, hull1):
            vertices_intersection.append(v)

    hull_intersection = ConvexHull(np.array(vertices_intersection))

    area1 = hull1.area
    area2 = hull2.area
    area_inter = hull_intersection.area
    # print("Area 1 = {}, Area 2 = {}, Intersection Area = {}".format(area1, area2, area_inter))

    return (area1 + area2 - 2*area_inter)/(area1 + area2)

def computeError3d(hull1, hull2):
    # compute the symmetric difference between hull1 and hull2

    hs = np.vstack((hull1.equations, hull2.equations))
    feasible_point = sum(hull2.points)/len(hull2.points)
    if not pointInHull(feasible_point, hull1) or not pointInHull(feasible_point, hull2):
        feasible_point = sum(hull1.points)/len(hull1.points)
    assert pointInHull(feasible_point, hull1)
    assert pointInHull(feasible_point, hull2)
    HS = HalfspaceIntersection(hs, feasible_point)

    hull_intersection = ConvexHull(HS.intersections)

    volume1 = hull1.volume
    volume2 = hull2.volume
    volume_inter = hull_intersection.volume
    # print("volume 1 = {}, volume 2 = {}, Intersection volume = {}".format(volume1, volume2, volume_inter))

    return (volume1 + volume2 - 2*volume_inter)/(volume1 + volume2)

def print_np_2d_array(A):
    assert(len(np.shape(A))==2)

    for a in A:
        for b in a:
            print("{: 4.2f}".format(float(b)), end=' ')
        print()

def compute_outer_convex(halfspaces, feasible_point):

    # hs = [[-v[3], -v[0], -v[1], -v[2]] for v in halfspaces]
    # mat = cdd.Matrix(hs, number_type='float')
    # mat.rep_type = cdd.RepType.INEQUALITY
    #
    # poly = cdd.Polyhedron(mat)
    #
    # generators = poly.get_generators()
    #
    # vertices = [np.array(g[1:]).reshape((3,1)) for g in generators if g[0]==1]

    HS = HalfspaceIntersection(np.array(halfspaces), np.reshape(feasible_point, 3))
    # Scipy is faster than cdd

    return HS.intersections
