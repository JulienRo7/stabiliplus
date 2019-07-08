#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



file = open("build/timings.txt", 'r')

iterations = []
lp_times = []
inner_times = []
outer_times = []
support_times = []

for line in file:
    line = line.split(';')
    iterations.append(int(line[0])+4)
    lp_times.append(float(line[1]))
    inner_times.append(float(line[2]))
    outer_times.append(float(line[3]))
    support_times.append(float(line[4]))

ax = plt.subplot(111)

ax.plot(iterations, lp_times, label="time LP")
ax.plot(iterations, inner_times, label="time Inner Convex")
ax.plot(iterations, outer_times, label="time Outer Convex")
ax.plot(iterations, support_times, label="time Support Function")


ax.set_xlabel("Number of Iterations")
ax.set_ylabel("Time (microseconds)")
ax.legend()
plt.show()

file.close()
