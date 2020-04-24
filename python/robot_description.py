#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import utilities as ut

from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
import xml.etree.ElementTree as ET # phone home!


class Robot():

    def __init__(self, masse, feet, mu, flim=[], dim=2):

        # self.gravity = np.array([[0], [0], [-9.81]])
        self.reset_gravity()
        self.masse = masse

        # list containning the position and orientation of each of the contacts of the robot with its environnement w.r.t. the origin frame of the robot ? -> for now
        self.feet = feet
        self.n_feet = len(feet)  # Number of feet of the robot
        self.mu = mu  # list of the friction coeficients associated with each foot.
        if flim:
            self.flim = flim
        else:
            self.flim = [(0,0)]*len(mu)

        #########################################
        # Mathematical Static Model
        #########################################
        self.P = np.array([[1, 0, 0],
                           [0, 1, 0]])

        self.A1 = np.zeros([6, 3*self.n_feet])

        self.A2 = np.zeros([6, self.P.shape[0]])

        self.t = np.zeros([6, 1])

        self.u = np.zeros([3*self.n_feet, 1])
        self.B = np.zeros([3*self.n_feet, 3*self.n_feet])

        self.update_static_model()

    def set_acceleration(self, acceleration):
        self.gravity = np.array(acceleration)

    def reset_gravity(self):
        self.set_acceleration([[0], [0], [-9.81]])

    def set_P_matrix(self, P_matrix=np.eye(3)):
        self.P = P_matrix
        if self.P.shape[0] != self.A2.shape[1]:
            self.A2 = np.zeros([6, self.P.shape[0]])


    def update_A1(self):
        for i in range(self.n_feet):
            r = self.feet[i][0:3, 3]
            self.A1[:3, 3*i:3*i+3] = np.eye(3)
            self.A1[3:, 3*i:3*i+3] = ut.antisymMatrix(r)

    def update_friction(self):
        for i in range(self.n_feet):
            v = self.feet[i][0:3, 2]
            v = np.reshape(v, [3, 1])
            self.u[3*i:3*i+3] = self.mu[i]*v
            self.B[3*i:3*i+3, 3*i:3*i+3] = np.eye(3) - np.matmul(v, v.T)

    def update_A2(self):
        self.A2[3:, :] = np.matmul(-ut.antisymMatrix(self.masse*self.gravity), self.P.T)

    def update_t(self):
        self.t[0:3, :] = -self.masse*self.gravity

    def update_static_model(self):
        self.update_A2()
        self.update_t()

        self.update_A1()
        self.update_friction()

    def support_polygon(self):
        # support polygon
        support = [foot[:2,3] for foot in self.feet]
        conv_support= ConvexHull(np.reshape(np.array(support), (self.n_feet, 2)))

        x = []
        y = []
        for i in conv_support.vertices:
            x.append(support[i][0])
            y.append(support[i][1])

        x.append(x[0])
        y.append(y[0])
        return x, y

    def display_robot_configuration(self, ax=None, subplot_conf=111):
        # create a 3D view of the configuration
        # fig = plt.figure()
        if ax == None:
            ax = plt.subplot(subplot_conf, projection='3d')

        x_support, y_support = self.support_polygon()
        lines = []
        lines.extend(ax.plot(x_support, y_support, 0, label="Support Polygon"))

        x_feet = [foot[0, 3] for foot in self.feet]
        y_feet = [foot[1, 3] for foot in self.feet]
        z_feet = [foot[2, 3] for foot in self.feet]

        # ax.scatter(x_feet, y_feet, z_feet)

        radius = 0.1
        Thetas = np.linspace(0, 2*np.pi, 20)
        Circle = np.array([[radius*np.cos(theta) for theta in Thetas],
                           [radius*np.sin(theta) for theta in Thetas],
                           [0 for theta in Thetas],
                           [1 for theta in Thetas]])
        # ax.plot_trisurf(Circle[0, :], Circle[1, :], Circle[2, :])
        for i in range(self.n_feet):
            R = self.feet[i]
            circle = np.matmul(R, Circle)

            lines.append(ax.plot_trisurf(circle[0, :], circle[1, :], circle[2, :], color='k', alpha='0.5'))
            lines.append(ax.quiver(R[0, 3], R[1, 3], R[2, 3], -R[0, 2], -R[1, 2], -
                             R[2, 2], length=3*radius, arrow_length_ratio=0.1, pivot='tip', colors=['k']))

        # ax.axis('scaled')
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        # ax.set_zlim(-0.1, 2.9)
        ax.set_zlim(-1.5, 1.5)
        return ax, lines

    def generate_random_robot(masse, n_feet, dim=2):
        # generate a random robot configuration
        # masse and number of feet is given
        feet = [np.eye(4) for i in range(n_feet)]
        mu = [0.7*np.random.rand()+0.3 for i in range(n_feet)]

        for i in range(n_feet):
            feet[i][:3, 3] = 2*(np.random.rand(3)-0.5)
            # feet[i][2, 3] = 0.5*(np.random.rand(1)-0.5)
            feet[i][:3, :3] = ut.euler2RotMat(
                0, 2*np.pi*np.random.rand(), i*np.pi/n_feet*np.random.rand())

        random_robot = Robot(masse, feet, mu, dim)
        return random_robot


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

            robot = Robot(masse, feet, mu)
        if pos == 2:
            feet[0][0:3, 3] = np.array([0, 1, 0])
            feet[0][:3, :3] = ut.euler2RotMat(-np.pi/4, 0, 0)

            feet[1][0:3, 3] = np.array([np.sqrt(3)/2, -1/2, 0])
            feet[1][:3, :3] = ut.euler2RotMat(0, np.pi/3, np.pi/4)

            feet[2][0:3, 3] = np.array([-np.sqrt(3)/2, -1/2, 0])
            feet[2][:3, :3] = ut.euler2RotMat(0, -np.pi/3, np.pi/4)

            robot = Robot(masse, feet, mu)

        if pos == 3:
            feet[0][0:3, 3] = np.array([0, 1, 0])
            feet[0][:3, :3] = ut.euler2RotMat(-3*np.pi/4, 0, 0)

            feet[1][0:3, 3] = np.array([np.sqrt(3)/2, -1/2, 0])
            feet[1][:3, :3] = ut.euler2RotMat(0, np.pi/3, np.pi/4)

            feet[2][0:3, 3] = np.array([-np.sqrt(3)/2, -1/2, 0])
            feet[2][:3, :3] = ut.euler2RotMat(0, -np.pi/3, np.pi/4)
            robot = Robot(masse, feet, mu)

        if pos == 4:
            feet[0][0:3, 3] = np.array([0, 1, 0])
            feet[0][:3, :3] = ut.euler2RotMat(0, 0, 0)

            feet[1][0:3, 3] = np.array([np.sqrt(3)/2, -1/2, 0])
            feet[1][:3, :3] = ut.euler2RotMat(0, 0, 0)

            feet[2][0:3, 3] = np.array([-np.sqrt(3)/2, -1/2, 0])
            feet[2][:3, :3] = ut.euler2RotMat(0, 0, 0)

            robot = Robot(masse, feet, mu)

        if pos == 5:
            feet[0][0:3, 3] = np.array([1, 0, 1])
            feet[0][:3, :3] = ut.euler2RotMat(0, 0, 0)

            feet[1][0:3, 3] = np.array([-1/2, np.sqrt(3)/2, 0])
            feet[1][:3, :3] = ut.euler2RotMat(0, 0, 0)

            feet[2][0:3, 3] = np.array([-1/2, -np.sqrt(3)/2, 0])
            feet[2][:3, :3] = ut.euler2RotMat(0, 0, 0)

            robot = Robot(masse, feet, mu)

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

            robot = Robot(masse, feet, mu)

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

            robot = Robot(masse, feet, mu)


        if pos == 0:
            n_feet = randint(3, 5)
            robot = generate_random_robot(1, n_feet)

        return robot

    def parse_contactPoint(contactXML, feet, mu, flim):
        foot = np.eye(4)

        for child in contactXML:
            if child.tag == 'friction':
                mu.append(float(child.attrib['mu']))
            elif child.tag == 'matrix':
                if child.attrib['name'] == 'position':
                    for i in range(3):
                        # print(child[i][0].text)
                        foot[i, 3] = float(child[i][0].text)
                elif child.attrib['name'] == 'rotation':
                    for i in range(3):
                        for j in range(3):
                            # print(child[i][j].text)
                            foot[i, j] = float(child[i][j].text)
                else:
                    print("Unknown matrix name: {}".format(child.attrib['name']))
                    
            elif child.tag == 'rotation':
                phi = float(child.attrib['phi'])
                theta = float(child.attrib['theta'])
                psi = float(child.attrib['psi'])
                foot[:3,:3] = ut.euler2RotMat(phi, theta, psi)
                
            elif child.tag == 'flim':
                fmax = float(child.attrib['fmax'])
                fmin = float(child.attrib['fmin'])
                flim.append((fmax, fmin))
            elif child.tag == 'contactType':
                pass
            else:
                print("Error: Unrecognised tag: {}".format(child.tag))
        feet.append(foot)

    # <ContactPoint name='contact_1'>
    #     <friction mu='0.5'/>
    #     <matrix name='position' row='3' column='1'>
    #         <line><v>0.0</v></line>
    #         <line><v>1.0</v></line>
    #         <line><v>1.0</v></line>
    #     </matrix>
    #     <rotation phi='1.5708' theta='0' psi='0' />
    # </ContactPoint>

    def parse_robot(robotXML):
        feet = []
        mu = []
        flim = []
        mass = 0
        dim = 3

        for child in robotXML: #loop over robot and accelerations
            if child.tag == 'robot':
                for grandChild in child:

                    if grandChild.tag == 'Mass':
                        mass = float(grandChild.attrib['mass'])
                    elif grandChild.tag == 'ContactPoint':
                        Robot.parse_contactPoint(grandChild, feet, mu, flim)
                    else:
                        pass

        parsed_robot = Robot(mass, feet, mu, flim, dim)
        return parsed_robot

    def load_from_file(file_name):
        tree = ET.parse(file_name)
        root = tree.getroot()

        parsed_robot = Robot.parse_robot(root)

        return parsed_robot
