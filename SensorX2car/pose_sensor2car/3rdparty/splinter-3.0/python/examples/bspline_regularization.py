# This file is part of the SPLINTER library.
# Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

# Add the SPLINTER directory to the search path, so we can include it
import numpy as np
import matplotlib.pyplot as plt
from os import sys, path, remove
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import splinter

# Only for dev purposes
# splinter.load("/home/bjarne/Code/C++/splinter4/splinter/bin/Release/libsplinter-3-0.so")
splinter.load("/home/anders/SPLINTER/build/debug/libsplinter-3-0.so")


# Example with one variable
def f1(x):
    return -1. + 2*x + 0.1*(x**2) + 10*np.random.rand(1)[0]

x = [0.]*11
y = [0.]*11
for i in range(11):
    x[i] = i
    y[i] = f1(i)

# Cubic B-spline that interpolates the data (note that NONE is the default smoothing setting)
b1 = splinter.BSplineBuilder(x, y, smoothing=splinter.BSplineBuilder.Smoothing.NONE).build()

# Cubic B-spline with regularization
b2 = splinter.BSplineBuilder(x, y, smoothing=splinter.BSplineBuilder.Smoothing.IDENTITY, alpha=0.1).build()

# Cubic P-spline
b3 = splinter.BSplineBuilder(x, y, smoothing=splinter.BSplineBuilder.Smoothing.PSPLINE, alpha=0.1).build()

n = 1000
xd = [0.]*n
yd1 = [0.]*n
yd2 = [0.]*n
yd3 = [0.]*n
for i in range(n):
    val = i/100.
    xd[i] = val
    yd1[i] = b1.eval([val])
    yd2[i] = b2.eval([val])
    yd3[i] = b3.eval([val])


plt.plot(x, y, '*', label='Data points')
plt.plot(xd, yd1, label='Interpolating B-spline')
plt.plot(xd, yd2, '--', label='Regularized B-spline')
plt.plot(xd, yd3, '-.', label='P-spline')
plt.legend(loc='upper left')
plt.show()
