import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import sqrt
# parameters
rR = 0.075
rL = rR
b = 0.3/2
tf = 3
Vmax=10
Wmax=50
alfa= 1
beta=19
# initial conditions
theta0 = 0
x0 = 0
y0 = 0
#PID
K = 4
Ki= 0.01
Kd= 0.001
# time vector and time increment
t = np.linspace(0, tf, 60)
dt = t[1] - t[0]