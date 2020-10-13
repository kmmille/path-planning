"""lyap_accel_controller controller."""
from math import *
import numpy as np
from numpy.linalg import norm
from scipy.integrate import odeint

# Car model
def model(q, t, wps):
    L = 1
    k = 100
    kappa = 10
    G = 100

    # q is the current position
    x, y, phi, v, w = q

    x1, y1, x2, y2, tdur = wps

    vref = sqrt((y2 - y1)**2 + (x2 - x1)**2)/tdur
    phiref = atan2(y2 - y1, x2 - x1)
    xref = x1 + vref*t*cos(phiref)
    yref = y1 + vref*t*sin(phiref)

    wref = 0
    aref = 0
    omegaref = 0

    z1diff = x + L*cos(phi) - (xref + L*cos(phiref))
    z2diff = y + L*sin(phi) - (yref + L*sin(phiref))

    gx = z1diff*(G/(1 + norm(np.array([z1diff, z2diff]))))
    gy = z2diff*(G/(1 + norm(np.array([z1diff, z2diff]))))

    vx = vref*cos(phi) - L*sin(phiref)*omegaref
    vy = vref*sin(phi) + L*cos(phiref)*omegaref

    ax = aref*cos(phi)
    ay = aref*sin(phi)

    u1 = ax + k*vx - gx
    u2 = ay + k*vy - gy

    f = cos(phi)*(u1 + v*w*sin(phi) + L*(w**2)*cos(phi)) + sin(phi)*(u2 - v*w*cos(phi) + L*(w**2)*sin(phi))
    tau = (-sin(phi)/L)*(u1 + v*w*sin(phi) + L*(w**2)*cos(phi)) + (cos(phi)/L)*(u2 - v*w*cos(phi) + L*(w**2)*sin(phi))

    xdot = v*cos(phi)
    ydot = v*sin(phi)
    phidot = w
    vdot = f - k*v
    wdot = tau - k*w

    return [xdot, ydot, phidot, vdot, wdot]

def run_model(q, t, wps):
    q1 = odeint(model, q, t, args = (wps,))
    return q1[-1]

def ref_pos(t, wps):
   x1, y1, x2, y2, tdur = wps
   vref = sqrt((y2 - y1)**2 + (x2 - x1)**2)/tdur
   phiref = atan2(y2 - y1, x2 - x1)
   xref = x1 + vref*t*cos(phiref)
   yref = y1 + vref*t*sin(phiref)

   return xref, yref, phiref

def set_waypoints(wps_list):
    x1, y1, t1 = wps_list[0]
    x2, y2, t2 = wps_list[1]

    T = t2 - t1

    return [x1, y1, x2, y2, T]
