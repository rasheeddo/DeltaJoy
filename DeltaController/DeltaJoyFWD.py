# Test servo with Maestro 

import time
import os
import numpy
import math
#from sympy import *
#from sympy.solvers import solve
#from sympy import Symbol

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def FWD(deg1, deg2, deg3):

    root3 = math.sqrt(3)

    A1v = numpy.array([0, -wb - L*math.cos(deg1) + up, -L*math.sin(deg1)])
    A2v = numpy.array([(root3/2)*(wb + L*math.cos(deg2)) - sp/2, 0.5*(wb+ L*math.cos(deg2)) - wp, -L*math.sin(deg2)])
    A3v = numpy.array([(-root3/2)*(wb + L*math.cos(deg3)) + sp/2, 0.5*(wb+ L*math.cos(deg3)) - wp, -L*math.sin(deg3)])

    r1 = l
    r2 = l
    r3 = l

    x1 = A1v[0]
    y1 = A1v[1]
    z1 = A1v[2]

    x2 = A2v[0]
    y2 = A2v[1]
    z2 = A2v[2]

    x3 = A3v[0]
    y3 = A3v[1]
    z3 = A3v[2]

    #Select the method to calculate
    #Depends on the height of virtual spheres center
    #Method 1 is used when the height of z1 z2 z3 are equal
    #Method 2, 3, 4 are trying to avoid 0 division at a13 and a23

    if ((z1==z2) and (z2==z3) and (z1==z3)):
        method = 1
    elif ((z1 != z3) and (z2 != z3)):
        method = 2
    elif ((z1 != z2) and (z1 != z3)):
        method = 3
    else:
        method = 4

    if method == 1:
        zn = z1  # z1 = z2 = z3 = zn

        a = 2*(x3 - x1)
        b = 2*(y3 - y1)
        c = r1**2 - r3**2 - x1**2 - y1**2 + x3**2 + y3**2
        d = 2*(x3 - x2)
        e = 2*(y3 - y2)
        f = r2**2 - r3**2 - x2**2 -y2**2 + x3**2 + y3**2

        numX = c*e - b*f
        denX = a*e - b*d
        x = numX/denX
        if x < 0.000001:
            x = 0

        numY = a*f - c*d
        denY = a*e - b*d
        y = numY/denY
        if y < 0.000001:
            y = 0

        A = 1
        B = -2*zn
        C = zn**2 - r1**2 + (x-x1)**2 + (y-y1)**2

        z = [None]*2

        z[0] = (-B + math.sqrt(B**2 - 4*C))/2;
        z[1] = (-B - math.sqrt(B**2 - 4*C))/2;

        realANS = [None]*3

        if z[0] < 0: 
            realANS = numpy.array([x,y,z[0]])
        elif z[1] < 0:
            realANS = numpy.array([x,y,z[1]])
        else:
            showError = "Error: height z is zero"
            print(showError)

        print("Method: %d" %method)
        print(realANS)

    elif method ==2:

        a11 = 2*(x3 - x1)
        a12 = 2*(y3 - y1)
        a13 = 2*(z3 - z1)
        
        a21 = 2*(x3 - x2)
        a22 = 2*(y3 - y2)
        a23 = 2*(z3 - z2)
        
        b1 = r1**2 - r3**2 - x1**2 - y1**2 - z1**2 + x3**2 + y3**2 + z3**2
        b2 = r2**2 - r3**2 - x2**2 - y2**2 - z2**2 + x3**2 + y3**2 + z3**2
        
        a1 = (a11/a13) - (a21/a23)
        a2 = (a12/a13) - (a22/a23)
        a3 = (b2/a23) - (b1/a13)

        a4 = -a2/a1
        a5 = -a3/a1
        a6 = (-a21*a4 - a22)/a23
        a7 = (b2 - a21*a5)/a23
        
        a = a4**2 + 1 + a6**2;
        b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1);
        c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2;
        '''
        YY = Symbol('YY')
        sol = solve(a*YY**2 + b*YY + c,YY)
        y = sol
        '''
        y = [None]*2
        y[0] = (-b + math.sqrt(b**2 - 4*a*c)) / (2*a)  
        y[1] = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)

        x = [None]*2
        z = [None]*2

        x[0] = a4*y[0] + a5;
        x[1] = a4*y[1] + a5;
        z[0] = a6*y[0] + a7;
        z[1] = a6*y[1] + a7;

        realANS = [None]*3
        
        if z[0] < 0:
            realANS = numpy.array([x[0],y[0],z[0]])
        elif z[1] < 0:
            realANS = numpy.array([x[1],y[1],z[1]])
        else:
            showError = "Error: height z is zero"
            print(showError)

        print("Method: %d" %method)
        print(realANS)

    elif method == 3:
        a11 = 2*(x1 - x2)
        a12 = 2*(y1 - y2)
        a13 = 2*(z1 - z2)
        
        a21 = 2*(x1 - x3)
        a22 = 2*(y1 - y3)
        a23 = 2*(z1 - z3)
        
        b1 = r2**2 - r1**2 - x2**2 - y2**2 - z2**2 + x1**2 + y1**2 + z1**2
        b2 = r3**2 - r1**2 - x3**2 - y3**2 - z3**2 + x1**2 + y1**2 + z1**2
        
        a1 = (a11/a13) - (a21/a23)
        a2 = (a12/a13) - (a22/a23)
        a3 = (b2/a23) - (b1/a13)

        a4 = -a2/a1
        a5 = -a3/a1
        a6 = (-a21*a4 - a22)/a23
        a7 = (b2 - a21*a5)/a23
        
        a = a4**2 + 1 + a6**2
        b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1)
        c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2
        '''
        YY = Symbol('YY')
        sol = solve(a*YY**2 + b*YY + c,YY);
        y = sol
        '''
        y = [None]*2
        y[0] = (-b + math.sqrt(b**2 - 4*a*c)) / (2*a)  
        y[1] = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)

        x = [None]*2
        z = [None]*2

        x[0] = a4*y[0] + a5
        x[1] = a4*y[1] + a5
        z[0] = a6*y[0] + a7
        z[1] = a6*y[1] + a7

        realANS = [None]*3

        if z[0] < 0: 
            realANS = numpy.array([x[0],y[0],z[0]])
        elif z[1] < 0:
            realANS = numpy.array([x[1],y[1],z[1]])
        else:
            showError = "Error: height z is zero"
            print(showError)

        print("Method: %d" %method)
        print(realANS)

    if method == 4:
        a11 = 2*(x2 - x1)
        a12 = 2*(y2 - y1)
        a13 = 2*(z2 - z1)
        
        a21 = 2*(x2 - x3)
        a22 = 2*(y2 - y3)
        a23 = 2*(z2 - z3)
        
        b1 = r1**2 - r2**2 - x1**2 - y1**2 - z1**2 + x2**2 + y2**2 + z2**2
        b2 = r3**2 - r2**2 - x3**2 - y3**2 - z3**2 + x2**2 + y2**2 + z2**2
        
        a1 = (a11/a13) - (a21/a23)
        a2 = (a12/a13) - (a22/a23)
        a3 = (b2/a23) - (b1/a13)

        a4 = -a2/a1
        a5 = -a3/a1
        a6 = (-a21*a4 - a22)/a23
        a7 = (b2 - a21*a5)/a23
        
        a = a4**2 + 1 + a6**2
        b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1)
        c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2
        '''
        YY = Symbol('YY')
        sol = solve(a*YY**2 + b*YY + c,YY);
        y = sol
        '''
        y = [None]*2
        y[0] = (-b + math.sqrt(b**2 - 4*a*c)) / (2*a)  
        y[1] = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)

        x = [None]*2
        z = [None]*2
       
        x[0] = a4*y[0] + a5
        x[1] = a4*y[1] + a5
        z[0] = a6*y[0] + a7
        z[1] = a6*y[1] + a7

        realANS = [None]*3

        if z[0] < 0:
            realANS = numpy.array([x[0],y[0],z[0]])
        elif z[1] < 0:
            realANS = numpy.array([x[1],y[1],z[1]])
        else:
            showError = "Error: height z is zero"
            print(showError)

        print("Method: %d" %method)
        print(realANS)


    return realANS

rad2deg = 180/math.pi
deg2rad = math.pi/180

########################## parameters ################################
## average time for calcultaiob is around 0.002 seconds
startTime = time.time()

sb = 242.487
sp = 60.622
L = 108         #270
l = 175         #545
h = 70
root3 = math.sqrt(3)
wb = (root3/6)*sb
ub = (root3/3)*sb
wp = (root3/6)*sp
up = (root3/3)*sp


deg1 = 0*deg2rad
deg2 = 0*deg2rad
deg3 = 0*deg2rad

while True:

    FWD(deg1,deg2,deg3)
    deg1 = deg1 + 1*deg2rad
    deg2 = deg2 + 2*deg2rad
    deg3 = deg3 + 3*deg2rad

    DEG1 = deg1*rad2deg
    DEG2 = deg2*rad2deg
    DEG3 = deg3*rad2deg

    print("deg1:%f"%DEG1)
    print("deg2:%f"%DEG2)
    print("deg3:%f"%DEG3)
    time.sleep(0.5)



endTime = time.time()
period = endTime - startTime
print("Period: %f" %period)

