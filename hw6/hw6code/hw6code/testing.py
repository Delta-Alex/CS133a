import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp
from hw5code.TransformHelpers   import *

def spline(t, T, p0, pf, v0, vf):
    # Compute the parameters.
    a = p0
    b = v0
    c =   3*(pf-p0)/T**2 - vf/T    - 2*v0/T
    d = - 2*(pf-p0)/T**3 + vf/T**2 +   v0/T**2
    # Compute the current (p,v).
    print("a : {}".format(a))
    print("b : {}".format(b))
    print("c : {}".format(c))
    print("d : {}".format(d))
    p = a + b * t +   c * t**2 +   d * t**3
    v =     b     + 2*c * t    + 3*d * t**2
    return (p,v)

def quin(t, T, p0, pf):
    # Compute the current (p,v).
    p = p0 + (pf-p0)   * (10*(t/T)**3 - 15*(t/T)**4 +  6*(t/T)**5)
    v =    + (pf-p0)/T * (30*(t/T)**2 - 60*(t/T)**3 + 30*(t/T)**4)

    a = p0
    b = 0
    c = 0

    d = 10*(pf-p0)/T**3 
    e = -15*(pf-p0)/T**4 
    f = 6*(pf-p0)/T**5 

    print(a,b,c,d,e,f)

    return (p,v)

if __name__ == "__main__":
    #if t < 3.0:
    
    #(s0, s0dot) = quin(0, 3.0, 0.0, 1.0)

    #if t1 < 1.25:
    # from pright to phigh
    #(sp, spdot) = quin(0, 1.25, -1.0,  1.0)
    #(sR, sRdot) = quin(0, 1.25, 0,  1.0)

    #elif t1 < 2.50:
    # from phigh to pleft
    #(sp, spdot) = quin(0, 1.25, -1.0,  1.0)
    #(sR, sRdot) = quin(0, 1.25, 0,  1.0)


    #elif t1 < 3.75:
    # from pleft to phigh
    #(sp, spdot) = quin(0, 1.25, -1.0,  1.0)
    #(sR, sRdot) = quin(0, 1.25, 1.0,  0)

    #else:
    # from phigh to pright
    #(sp, spdot) = quin(0, 1.25, -1.0,  1.0)
    #(sR, sRdot) = quin(0, 1.25, 1.0,  0)
    p, v = spline(pi/2, pi, 0,0, 1, -1)
    print(p, v)