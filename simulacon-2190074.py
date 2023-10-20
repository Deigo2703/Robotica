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

# initialization and pre-allocation of pose variables
#x1 - y1 - theta son la referencia | x2 - y2 - theta_ los practicos
x1 = np.zeros(len(t))
x2 = np.zeros(len(t))
y1 = np.zeros(len(t))
y2 = np.zeros(len(t))

theta = np.zeros(len(t))
theta_ = np.zeros(len(t))

theta[0] = theta0
theta_[0] = theta0
x1[0] = x0
x2[0] = x0
y1[0] = y0
y2[0] = y0

# command signals (in RPM)
ind1 = np.where(t <= max(t)/3)[0]
ind2 = np.where((t > max(t)/3) & (t <= 2*max(t)/3))[0]
ind3 = np.where((t > 2*max(t)/3) & (t <= max(t)))[0]
vec = np.ones(len(t))

phidotR = np.concatenate((240*(2*np.pi/60)*vec[ind1], 
120*(2*np.pi/60)*vec[ind2], 
120*(2*np.pi/60)*vec[ind3]))
phidotL = np.concatenate((120*(2*np.pi/60)*vec[ind1], 
240*(2*np.pi/60)*vec[ind2], 
-120*(2*np.pi/60)*vec[ind3]))

# update loop
#Planta -- Datos teoricos
for ii in range(1, len(t)):
    
 J = np.array([[rR*np.cos(theta[ii-1])/2, rL*np.cos(theta[ii-1])/2],
[rR*np.sin(theta[ii-1])/2, rL*np.sin(theta[ii-1])/2],
[0.5*rR/b, -0.5*rL/b]])
 
 deltapose = dt * J @ np.array([[phidotR[ii-1]], [phidotL[ii-1]]])
 x1[ii] = x1[ii-1] + deltapose[0]
 y1[ii] = y1[ii-1] + deltapose[1]
 theta[ii] = theta[ii-1] + deltapose[2]
 
 
fig3, ax = plt.subplots()
ax.plot(x1, y1, 'k--')
ax.set_title('workspace')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_xlim(-1.25, 1.25)
ax.set_ylim(-0.5, 2)
wls = np.array([[0, 0], [-0.5*b, 0.5*b]])
wlsrot = np.array([[np.cos(theta[0]), -np.sin(theta[0])], [np.sin(theta[0]), np.cos(theta[0])]]) @ wls

h1, = ax.plot(wlsrot[0,0]+x1[0], wlsrot[1,0]+y1[0], 'ro', linewidth=2, markersize=6, markerfacecolor='r')
h2, = ax.plot(wlsrot[0,1]+x1[0], wlsrot[1,1]+y1[0], 'ro', linewidth=2, markersize=6, markerfacecolor='r')
h3, = ax.plot(x1[0], y1[0], 'bo', markersize=20)


# + Controlador
ii = 0 
while ii<60:
    e0 = theta[ii] - theta_[ii]
    # print(e0) #esto es solo para revisar el valor del error
    e0_1= e0
    # Implementacion del PID
    e0 = K*e0 + Ki*(e0 + e0_1) + Kd*(e0-e0_1)
    
    #Aplicacion de ley de control
    Wl = (Vmax *np.exp(-((e0**2)/(2*(alfa**2))))) - (b/2)*Wmax*((2/(1+np.exp(-(e0/beta))))-1)
    Wr = (Vmax *np.exp(-((e0**2)/(2*(alfa**2))))) + (b/2)*Wmax*((2/(1+np.exp(-(e0/beta))))-1)
    #__________________
    J = np.array([[rR*np.cos(theta_[ii-1])/2, rL*np.cos(theta_[ii-1])/2],
    [rR*np.sin(theta_[ii-1])/2, rL*np.sin(theta_[ii-1])/2],
    [0.5*rR/b, -0.5*rL/b]])
    
    deltapose_ = dt * J @ np.array([[Wr], [Wl]])
    x2[ii] = x2[ii-1] + deltapose_[0]
    y2[ii] = y2[ii-1] + deltapose_[1]
    theta_[ii] = theta_[ii-1] + deltapose_[2]  
    
    theta_[ii-1] = theta_[ii] 
    x2[ii-1] = x2[ii]
    y2[ii-1] = y2[ii]
    
    #_________________________________
    
    wlsrot = np.array([[np.cos(theta_[ii]), -np.sin(theta_[ii])],[np.sin(theta_[ii]), np.cos(theta_[ii])]]) @ wls
    h1.set_xdata([wlsrot[0,0] + x2[ii]])
    h1.set_ydata([wlsrot[1,0] + y2[ii]])
    h2.set_xdata([wlsrot[0,1] + x2[ii]])
    h2.set_ydata([wlsrot[1,1] + y2[ii]])
    h3.set_xdata([x2[ii]])
    h3.set_ydata([y2[ii]])
    plt.ion()
    plt.show()
    plt.pause(0.1)   
    
    #Calculo de errores de posicion
    dix = sqrt((x1[ii]- x2[ii])**2+ (y1[ii]-y2[ii])**2)
    if dix<0.2:
        ii= ii+1
    if dix>1:
          ii=60 
    
    

plt.draw()
plt.show()

