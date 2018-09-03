# Test servo with Maestro 

import time
import numpy
import math
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min



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
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)


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
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Torque is enable")
######################### Set Velocity / Acceleration Profile  ##############################
set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
set_V_Limit = 350       # 350 Default                  [0.229RPM]

#final_pos = 90.0          # deg
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

set_A_PRFL = 40      # between 0 ~ set_A_limit      [214.577 rev/min^2]
set_V_PRFL = 200       # between 0 ~ set_V_Limit      [0.229RPM]

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
set_P_Gain = 1000    #800 default
set_I_Gain = 20     #0 default
set_D_Gain = 1000   #4700 default

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


################################################################################################################################
rad2deg = 180/math.pi
deg2rad = math.pi/180
########################## parameters ################################
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

B1 = numpy.array([0, -wb, 0])
B2 = numpy.array([(root3/2)*wb, 0.5*wb, 0])
B3 = numpy.array([(-root3/2)*wb, 0.5*wb, 0])

b1 = numpy.array([sb/2, -wb, 0])
b2 = numpy.array([0, ub, 0])
b3 = numpy.array([-sb/2, -wb, 0])

P1 = numpy.array([0, -up, 0])
P2 = numpy.array([sp/2, wp, 0])
P3 = numpy.array([-sp/2, wp, 0])

########################### Draw circular path ############################
r = 60.0		# Input the radius of a circle 150 is max.
zeta = [None]*360
X = [None]*360
Y1 = [None]*180
Y2 = [None]*180
Y = [None]*360

for i in range(1,361):
    zeta[i-1]=i*deg2rad
    X[i-1] = r*math.cos(zeta[i-1])

for i in range(1,(len(X)/2)+1):
    Y1[i-1] = math.sqrt(r**2 - X[i-1]**2)

j = 0
for i in range((len(X)/2)+1,len(X)+1):
    Y2[j] = -math.sqrt(r**2 - X[j]**2)
    j = j+1

Y = numpy.concatenate((Y1,Y2))


####################### Inverse Kinematics ############################
delaySpeed = 0.05
rest = 3
pos1 = 90
pos2 = 90
pos3 = 90
################## Go to home position before starting circle ###########################
servo_ang1 = map(pos1, 0.0, 360.0, 0, 4095)
servo_ang2 = map(pos2, 0.0, 360.0, 0, 4095)
servo_ang3 = map(pos3, 0.0, 360.0, 0, 4095)

dxl1_goal_position = servo_ang1
dxl2_goal_position = servo_ang2
dxl3_goal_position = servo_ang3

dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))
time.sleep(4)
############################################################################################################################

height = -150
increment = 10
i = 0
while i<360:
	startTime = time.time()
	x = X[i]
	y = Y[i]
	z = height   # home is -485 but actual length is -460 but the difference 

	a = wb - up
	b = sp/2 - (root3/2)*wb
	c = wp - 0.5*wb

	E1 = 2*L*(y + a)
	F1 = 2*z*L
	G1 = x**2 + y**2 + z**2 + a**2 + L**2 + 2*y*a - l**2

	E2 = -L*(root3*(x+b) + y + c)
	F2 = 2*z*L
	G2 = x**2 + y**2 + z**2 + b**2 + c**2 + L**2 + 2*(x*b + y*c) - l**2

	E3 = L*(root3*(x-b) - y - c)
	F3 = 2*z*L
	G3 = x**2 + y**2 + z**2 + b**2 + c**2 + L**2 + 2*(-x*b + y*c) - l**2;

	t1 = [None]*2
	t2 = [None]*2
	t3 = [None]*2

	t1[0] = (-F1 + math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
	t1[1] = (-F1 - math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
	t2[0] = (-F2 + math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
	t2[1] = (-F2 - math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
	t3[0] = (-F3 + math.sqrt(E3**2 + F2**2 - G3**2)) / (G3 - E3)
	t3[1] = (-F3 - math.sqrt(E3**2 + F2**2 - G3**2)) / (G3 - E3)

	theta1 = 2*math.atan(t1[1]);
	theta2 = 2*math.atan(t2[1]);
	theta3 = 2*math.atan(t3[1]);

	deg1 = theta1*rad2deg;
	deg2 = theta2*rad2deg;
	deg3 = theta3*rad2deg;

	ServoDeg1 = 90 + deg1
	ServoDeg2 = 90 + deg2
	ServoDeg3 = 90 + deg3

	DEG = [deg1, deg2, deg3]
	DEG = [deg1, deg2, deg3]

	if isinstance(theta1, complex) or isinstance(theta2, complex) or isinstance(theta3, complex):
		print("Error: Driving angle is complex number")
		break

	
################################# Servo Drives ########################################

	servo_ang1 = map(ServoDeg1, 0.0, 360.0, 0, 4095)
	servo_ang2 = map(ServoDeg2, 0.0, 360.0, 0, 4095)
	servo_ang3 = map(ServoDeg3, 0.0, 360.0, 0, 4095)

	dxl1_goal_position = servo_ang1
	dxl2_goal_position = servo_ang2
	dxl3_goal_position = servo_ang3

	if dxl1_goal_position and dxl2_goal_position and dxl3_goal_position > 568:
		dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
		dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
		dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))
	else:
		print("ERROR: servo angle is less than 40deg")

	endTime = time.time()
	period = endTime - startTime
	print("i:%d" %i )
	print(DEG)
	print('---------------------------')
	if i == 0:
		time.sleep(0.5)
		# make it delay a bit at first dot
	i = i + increment

	time.sleep(delaySpeed)

################### Finish the full circle #####################
x = X[359]
y = Y[359]
z = height   # home is -485 but actual length is -460 but the difference 

a = wb - up
b = sp/2 - (root3/2)*wb
c = wp - 0.5*wb

E1 = 2*L*(y + a)
F1 = 2*z*L
G1 = x**2 + y**2 + z**2 + a**2 + L**2 + 2*y*a - l**2

E2 = -L*(root3*(x+b) + y + c)
F2 = 2*z*L
G2 = x**2 + y**2 + z**2 + b**2 + c**2 + L**2 + 2*(x*b + y*c) - l**2

E3 = L*(root3*(x-b) - y - c)
F3 = 2*z*L
G3 = x**2 + y**2 + z**2 + b**2 + c**2 + L**2 + 2*(-x*b + y*c) - l**2;

t1 = [None]*2
t2 = [None]*2
t3 = [None]*2

t1[0] = (-F1 + math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
t1[1] = (-F1 - math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
t2[0] = (-F2 + math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
t2[1] = (-F2 - math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
t3[0] = (-F3 + math.sqrt(E3**2 + F2**2 - G3**2)) / (G3 - E3)
t3[1] = (-F3 - math.sqrt(E3**2 + F2**2 - G3**2)) / (G3 - E3)

theta1 = 2*math.atan(t1[1]);
theta2 = 2*math.atan(t2[1]);
theta3 = 2*math.atan(t3[1]);

deg1 = theta1*rad2deg;
deg2 = theta2*rad2deg;
deg3 = theta3*rad2deg;

ServoDeg1 = 90 + deg1
ServoDeg2 = 90 + deg2
ServoDeg3 = 90 + deg3

DEG = [deg1, deg2, deg3]

if isinstance(theta1, complex) or isinstance(theta2, complex) or isinstance(theta3, complex):
	print("Error: Driving angle is complex number")
	

################################# Servo Drives ########################################

servo_ang1 = map(ServoDeg1, 0.0, 360.0, 0, 4095)
servo_ang2 = map(ServoDeg2, 0.0, 360.0, 0, 4095)
servo_ang3 = map(ServoDeg3, 0.0, 360.0, 0, 4095)

dxl1_goal_position = servo_ang1
dxl2_goal_position = servo_ang2
dxl3_goal_position = servo_ang3

if dxl1_goal_position and dxl2_goal_position and dxl3_goal_position > 568:
	dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
	dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
	dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))
else:
	print("ERROR: servo angle is less than 40deg")

print("i:%d" %i )
print(DEG)
#############################################################################################################################################

time.sleep(1)

################## Go Back to stand by position ##################
pos1 = 90
pos2 = 90
pos3 = 90
servo_ang1 = map(pos1, 0.0, 360.0, 0, 4095)
servo_ang2 = map(pos2, 0.0, 360.0, 0, 4095)
servo_ang3 = map(pos3, 0.0, 360.0, 0, 4095)

dxl1_goal_position = servo_ang1
dxl2_goal_position = servo_ang2
dxl3_goal_position = servo_ang3

dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))
time.sleep(rest)

