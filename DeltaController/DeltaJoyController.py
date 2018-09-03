# Test servo with Maestro 

import time
import os
import numpy
import cmath
import math

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

rad2deg = 180/math.pi
deg2rad = math.pi/180

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def FWD(deg1, deg2, deg3):

    root3 = math.sqrt(3)

    deg1 = deg1*deg2rad
    deg2 = deg2*deg2rad
    deg3 = deg3*deg2rad

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
        x = [None]
        y = [None]
        z = [None]*2
        realANS = [None]*3
    elif ((z1 != z3) and (z2 != z3)):
        method = 2
        x = [None]*2
        y = [None]*2
        z = [None]*2
        realANS = [None]*3
    elif ((z1 != z2) and (z1 != z3)):
        method = 3
        x = [None]*2
        y = [None]*2
        z = [None]*2
        realANS = [None]*3
    else:
        method = 4
        x = [None]*2
        y = [None]*2
        z = [None]*2
        realANS = [None]*3

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

        z[0] = (-B + math.sqrt(B**2 - 4*C))/2
        z[1] = (-B - math.sqrt(B**2 - 4*C))/2

        if z[0] < 0: 
            realANS = numpy.array([x,y,z[0]])
        elif z[1] < 0:
            realANS = numpy.array([x,y,z[1]])
        else:
            showError = "Error: height z is zero"
            print(showError)
            print(z)

        print("Method: %d" %method)
        #print(realANS)

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
        c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2
        '''
        YY = Symbol('YY')
        sol = solve(a*YY**2 + b*YY + c,YY)
        y = sol
        '''
        
        y[0] = (-b + math.sqrt(b**2 - 4*a*c)) / (2*a)  
        y[1] = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)

        x[0] = a4*y[0] + a5;
        x[1] = a4*y[1] + a5;
        z[0] = a6*y[0] + a7;
        z[1] = a6*y[1] + a7;
        
        if z[0] < 0:
            realANS = numpy.array([x[0],y[0],z[0]])
        elif z[1] < 0:
            realANS = numpy.array([x[1],y[1],z[1]])
        else:
            showError = "Error: height z is zero"
            print(showError)
            print(z)

        print("Method: %d" %method)
        #print(realANS)

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

        y[0] = (-b + math.sqrt(b**2 - 4*a*c)) / (2*a)  
        y[1] = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)

        x[0] = a4*y[0] + a5
        x[1] = a4*y[1] + a5
        z[0] = a6*y[0] + a7
        z[1] = a6*y[1] + a7

        if z[0] < 0: 
            realANS = numpy.array([x[0],y[0],z[0]])
        elif z[1] < 0:
            realANS = numpy.array([x[1],y[1],z[1]])
        else:
            showError = "Error: height z is zero"
            print(showError)
            print(z)

        print("Method: %d" %method)
        #rint(realANS)

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

        y[0] = (-b + math.sqrt(b**2 - 4*a*c)) / (2*a)  
        y[1] = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)
       
        x[0] = a4*y[0] + a5
        x[1] = a4*y[1] + a5
        z[0] = a6*y[0] + a7
        z[1] = a6*y[1] + a7

        if z[0] < 0:
            realANS = numpy.array([x[0],y[0],z[0]])
        elif z[1] < 0:
            realANS = numpy.array([x[1],y[1],z[1]])
        else:
            showError = "Error: height z is zero"
            print(showError)
            print(z)

        print("Method: %d" %method)
        #print(realANS)


    return realANS

####################################################### Set Servo Configuration #############################################################
# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

ADDR_PRO_CURRENT_LIMIT      = 38
ADDR_PRO_GOAL_CURRENT       = 102
ADDR_PRO_PRESENT_CURRENT    = 126 

ADDR_PRO_OPERATING_MODE     = 11

ADDR_PRO_GOAL_VELOCITY      = 104

ADDR_PRO_ACCELERATION_LIMIT = 40
ADDR_PRO_VELOCITY_LIMIT     = 44
ADDR_PRO_PROFILE_ACCELERATION  = 108
ADDR_PRO_PROFILE_VELOCITY   = 112

ADDR_PRO_POSITION_D_GAIN    = 80
ADDR_PRO_POSITION_I_GAIN    = 82
ADDR_PRO_POSITION_P_GAIN    = 84

CURRENT_CONTROL                     = 0
POSITION_CONTROL                    = 3 # Default
CURRENT_BASED_POSITION_CONTROL      = 5
# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                      = 1                             # Dynamixel ID: 1
DXL2_ID                      = 2                             # Dynamixel ID: 2
DXL3_ID                      = 3                             # Dynamixel ID: 3
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

'''
# When need to change mode just remove block comment out
# Disable Dynamixel Torque 1
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel 1 has been successfully connected")
# Disable Dynamixel Torque 2   
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel 2 has been successfully connected")
# Disable Dynamixel Torque 3
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel 3 has been successfully connected")

    # Check Operating Mode
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE, CURRENT_BASED_POSITION_CONTROL)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE, CURRENT_BASED_POSITION_CONTROL)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_OPERATING_MODE, CURRENT_BASED_POSITION_CONTROL)


present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE)
if present_mode == 0:
    # Current (Torque) Control Mode
    print("Now Operating Mode is Torque Control")
elif present_mode == 3:
    # Position Control Mode
    print("Now Operating Mode is Position Control")
elif present_mode == 5:
    # Current-based Position Control Mode
    print("Now Operating Mode is Current-based Position Control")
else:
    print("In other Mode that didn't set!")
'''

'''
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Torque is enable")
'''

######################### Set Velocity / Acceleration Profile  ##############################
set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
set_V_Limit = 350       # 350 Default                  [0.229RPM]

final_pos = 90.0          # deg
#t3 = 3.0                  # second
#t1 = t3/3              # second
#t2 = 2*t1               # second

#dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
#start_pos = map(dxl_present_position,0.0,4095.0,0.0,360.0)
#start_pos = 0
#delta_pos = final_pos - start_pos       # deg.
#delta_pos_rev = delta_pos/360.0           # Rev
#set_V_PRFL = (64.0*delta_pos)/(t2*100)     # Rev/Min
#set_A_PRFL = (64.0*set_V_PRFL)/(t1*100)    # Rev/Min^2

set_A_PRFL = 500      # between 0 ~ set_A_limit      [214.577 rev/min^2]
set_V_PRFL = 300       # between 0 ~ set_V_Limit      [0.229RPM]

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT)
velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT)
#print("Initial Position: %f" %start_pos)
#print("Final Position: %f" %final_pos)
#print("Travel time: %d" %t3)
print("V PRFL: %f" %set_V_PRFL)
print("A PRFL: %f" %set_A_PRFL)
print("Acceleration Limited: %d" %acceleration_limit)
print("Velocity Limited: %d" %velocity_limit)
print("--------------------------------")

######################### Set PID Gain Position Loop  ##############################
set_P_Gain = 2000    #800 default
set_I_Gain = 100     #0 default
set_D_Gain = 2000   #4700 default

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
print("PID's Gain are set")

position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN)
position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN)
position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN)

print("Position D Gain: %d" %position_D_gain)
print("Position I Gain: %d" %position_I_gain)
print("Position P Gain: %d" %position_P_gain)
print("--------------------------------")

######################### Set Goal Current  ##############################
SetCur = 45
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Goal Current is set")

################################################################################################################################
# Parameters
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

'''
# Give normal desired angle
inputAngle1 = 30
inputAngle2 = 30
inputAngle3 = 30

# Stand by position must be 90deg

pos1 = 90 + inputAngle1
pos2 = 90 + inputAngle2
pos3 = 90 + inputAngle3
servo_com1 = map(pos1,0.0,360.0,0.0,4095.0)
servo_com2 = map(pos2,0.0,360.0,0.0,4095.0)
servo_com3 = map(pos3,0.0,360.0,0.0,4095.0)

dxl1_goal_position = int(servo_com1)
dxl2_goal_position = int(servo_com2)
dxl3_goal_position = int(servo_com3)
'''

waitForPush = True
OK = False

'''
####################################################### Run Servo #############################################################
# Write goal position 1
if  pos1 and pos2 and pos3 > 40:

    dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
    if dxl_comm_result1 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))
    elif dxl_error1 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error1))

    dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
    if dxl_comm_result2 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result2))
    elif dxl_error2 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error2))

    dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, dxl3_goal_position)
    if dxl_comm_result3 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result3))
    elif dxl_error3 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error3))


else:
    print("Servo Angle is less than 45degree")
    OK = False
    waitForPush = False
'''

time.sleep(1)

while waitForPush:
    ################# Read Starting Angles ###############################

    dxl_present_position1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position3, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PRESENT_POSITION)

    read_pul1 = dxl_present_position1
    read_pul2 = dxl_present_position2
    read_pul3 = dxl_present_position3

    read_deg1 = map(read_pul1, 0, 4095, 0.0, 360.0)
    read_deg2 = map(read_pul2, 0, 4095, 0.0, 360.0)
    read_deg3 = map(read_pul3, 0, 4095, 0.0, 360.0)

    use_ang1 = read_deg1 - 90
    use_ang2 = read_deg2 - 90
    use_ang3 = read_deg3 - 90

    print("Push the ball down to start!")
    print("-------------------------------")

    if ((use_ang1 < 80) and (use_ang2 < 80) and (use_ang3 < 80)):
        OK = True
        waitForPush = False
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

        pos1 = 90 
        pos2 = 90 
        pos3 = 90 
        servo_com1 = map(pos1,0.0,360.0,0.0,4095.0)
        servo_com2 = map(pos2,0.0,360.0,0.0,4095.0)
        servo_com3 = map(pos3,0.0,360.0,0.0,4095.0)

        dxl1_goal_position = int(servo_com1)
        dxl2_goal_position = int(servo_com2)
        dxl3_goal_position = int(servo_com3)

        dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
        dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
        dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, dxl3_goal_position)


time.sleep(0.5)

while OK:

    pos1 = 135 
    pos2 = 135 
    pos3 = 135 
    servo_com1 = map(pos1,0.0,360.0,0.0,4095.0)
    servo_com2 = map(pos2,0.0,360.0,0.0,4095.0)
    servo_com3 = map(pos3,0.0,360.0,0.0,4095.0)

    dxl1_goal_position = int(servo_com1)
    dxl2_goal_position = int(servo_com2)
    dxl3_goal_position = int(servo_com3)

    dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
    dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
    dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, dxl3_goal_position)


    ################# Read Final Angles ###############################

    dxl_present_position1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position3, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PRESENT_POSITION)

    read_pul1 = dxl_present_position1
    read_pul2 = dxl_present_position2
    read_pul3 = dxl_present_position3

    print("RawPul1: %d" %read_pul1)
    print("RawPul2: %d" %read_pul2)
    print("RawPul3: %d" %read_pul3)

    read_deg1 = map(read_pul1, 0, 4095, 0.0, 360.0)
    read_deg2 = map(read_pul2, 0, 4095, 0.0, 360.0)
    read_deg3 = map(read_pul3, 0, 4095, 0.0, 360.0)

    use_ang1 = read_deg1 - 90
    use_ang2 = read_deg2 - 90
    use_ang3 = read_deg3 - 90

    print("Joint1: %f" %use_ang1)
    print("Joint2: %f" %use_ang2)
    print("Joint3: %f" %use_ang3)

    if ((use_ang1 < 120) or (use_ang2 < 120) or (use_ang3 < 120)):
        XYZ = FWD(use_ang1,use_ang2,use_ang3)
    else:
        print("Close to singurality posture!")
        XYZ = numpu.array([0,0,0])

    X_dir = XYZ[0]
    Y_dir = XYZ[1]
    Z_dir = XYZ[2]

    
    print("X: %f" %X_dir)
    print("Y: %f" %Y_dir)
    print("Z: %f" %Z_dir)
    print("-------------------------------")

    if ((use_ang1 < 0) and (use_ang2 < 0) and (use_ang3 < 0)):
        print("Controller is near singularity posture...")
        ####### Drive servo back to 0 deg ############
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

        pos1 = 135 
        pos2 = 135 
        pos3 = 135 
        servo_com1 = map(pos1,0.0,360.0,0.0,4095.0)
        servo_com2 = map(pos2,0.0,360.0,0.0,4095.0)
        servo_com3 = map(pos3,0.0,360.0,0.0,4095.0)

        dxl1_goal_position = int(servo_com1)
        dxl2_goal_position = int(servo_com2)
        dxl3_goal_position = int(servo_com3)

        dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
        dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
        dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, dxl3_goal_position)

        handRest = True
        print("Controller is in hand-rest mode...")
        print("Pull the controller up to start reading data")

        while handRest:
            #### Check the angles ####
            dxl_present_position1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
            dxl_present_position2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION)
            dxl_present_position3, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PRESENT_POSITION)

            read_pul1 = dxl_present_position1
            read_pul2 = dxl_present_position2
            read_pul3 = dxl_present_position3

            read_deg1 = map(read_pul1, 0, 4095, 0.0, 360.0)
            read_deg2 = map(read_pul2, 0, 4095, 0.0, 360.0)
            read_deg3 = map(read_pul3, 0, 4095, 0.0, 360.0)

            use_ang1 = read_deg1 - 90
            use_ang2 = read_deg2 - 90
            use_ang3 = read_deg3 - 90

            
            if ((use_ang1 > 3) or (use_ang2 > 3) or (use_ang3 > 3)):
                ### user must pull the controller up, so the program can continue reading data... if not, it is rest hand position ####
                handRest = False
                #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
                #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
                #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
            


   # If user pull the ball up, then stop the program
    if ((use_ang1 > 65) and (use_ang2 > 65) and (use_ang3 > 65)):
        print("Delta Controller is Off...")
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        OK = False

    time.sleep(1) # Delay time for reading data in loop



time.sleep(2) # Delay time before end program



################## Go to stand by position before starting circle ###########################
'''
pos1 = 160
pos2 = 160
pos3 = 160

servo_ang1 = map(pos1, 0.0, 360.0, 0, 4095)
servo_ang2 = map(pos2, 0.0, 360.0, 0, 4095)
servo_ang3 = map(pos3, 0.0, 360.0, 0, 4095)

dxl1_goal_position = servo_ang1
dxl2_goal_position = servo_ang2
dxl3_goal_position = servo_ang3

dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))
time.sleep(3)

print("Back to home position")

'''