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

ax1 = plt.subplot(221)
ax1.plot(iterations, lp_times, label="time LP", color="xkcd:crimson")
ax1.plot(iterations, inner_times, label="time Inner Convex Hull", color="xkcd:ochre")
ax1.plot(iterations, outer_times, label="time Outer Double Description Method", color="xkcd:grass")
ax1.plot(iterations, support_times, label="time Support Function", color="xkcd:electric blue")


ax1.set_xlabel("Number of Iterations")
ax1.set_ylabel("Time (microseconds)")
ax1.legend()

ax2 = plt.subplot(222)
ax2.plot(iterations, inner_times, label="time Inner Convex Hull", color="xkcd:ochre")
# ax2.plot(iterations, outer_times, label="time Outer Double Description Method", color="xkcd:grass")
ax2.set_xlabel("Number of Iterations")
ax2.set_ylabel("Time (microseconds)")
ax2.legend()

ax3 = plt.subplot(223)
ax3.plot(iterations, outer_times, label="time Outer Double Description Method", color="xkcd:grass")
ax3.set_xlabel("Number of Iterations")
ax3.set_ylabel("Time (microseconds)")
ax3.legend()

ax4 = plt.subplot(224)
ax4.plot(iterations, support_times, label="time Support Function", color="xkcd:electric blue")
ax4.set_xlabel("Number of Iterations")
ax4.set_ylabel("Time (microseconds)")
ax4.legend()

# nlogn = []
# nsquare = []
# nlognsquare = []
# for it in iterations:
#     nlogn.append(it*np.log(it))
#     nsquare.append(it*it)
#     nlognsquare.append(it*np.log(it)**2)
#
#
# ax1 = plt.subplot(221)
# ax1.plot(iterations, support_times, label="time Support Function", color="xkcd:electric blue")
# ax1.set_xlabel("n with n the number of iteration")
# ax1.set_ylabel("Time (microseconds)")
# ax1.legend()
# ax2 = plt.subplot(222)
# ax2.plot(nsquare, support_times, label="time Support Function", color="xkcd:electric blue")
# ax2.set_xlabel("n**2 with n the number of iteration")
# ax2.set_ylabel("Time (microseconds)")
# ax2.legend()
# ax3 = plt.subplot(223)
# ax3.plot(nlogn, support_times, label="time Support Function", color="xkcd:electric blue")
# ax3.set_xlabel("n*log(n) with n the number of iteration")
# ax3.set_ylabel("Time (microseconds)")
# ax3.legend()
# ax4 = plt.subplot(224)
# ax4.plot(nlognsquare, support_times, label="time Support Function", color="xkcd:electric blue")
# ax4.set_xlabel("n*log(n)**2 with n the number of iteration")
# ax4.set_ylabel("Time (microseconds)")
# ax4.legend()

plt.show()

file.close()
