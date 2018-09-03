# Test servo with Maestro 

import time
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

set_A_PRFL = 20      # between 0 ~ set_A_limit      [214.577 rev/min^2]
set_V_PRFL = 100       # between 0 ~ set_V_Limit      [0.229RPM]

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

# Give normal desired angle

inputAngle1 = 45
inputAngle2 = 45
inputAngle3 = 45
'''
inputAngle1 = 0
inputAngle2 = 0
inputAngle3 = 0
'''
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


time.sleep(0.1)



####################################################### Run Servo #############################################################
# Write goal position 1
startTime = time.time()

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

    print(dxl1_goal_position)
    print(dxl2_goal_position)
    print(dxl3_goal_position)
    des_pul1 = dxl1_goal_position
    des_pul2 = dxl2_goal_position
    des_pul3 = dxl3_goal_position



else:
    print("Servo Angle is less than 45degree")

endTime = time.time()
period = endTime -startTime

time.sleep(3)

################# Read Final Angles ###############################

dxl_present_position1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
dxl_present_position2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION)
dxl_present_position3, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PRESENT_POSITION)

read_pul1 = dxl_present_position1
read_pul2 = dxl_present_position2
read_pul3 = dxl_present_position3

error_pul1 = des_pul1 - read_pul1
error_pul2 = des_pul1 - read_pul2
error_pul3 = des_pul1 - read_pul3

print("RawPul1: %d" %dxl_present_position1)
print("RawPul2: %d" %dxl_present_position2)
print("RawPul3: %d" %dxl_present_position3)

last_pos1 = map(dxl_present_position1, 0, 4095, 0.0, 360.0)
last_pos2 = map(dxl_present_position2, 0, 4095, 0.0, 360.0)
last_pos3 = map(dxl_present_position3, 0, 4095, 0.0, 360.0)

last_pos1 = last_pos1 - 90
last_pos2 = last_pos2 - 90
last_pos3 = last_pos3 - 90

print("Joint1: %f" %last_pos1)
print("Joint2: %f" %last_pos2)
print("Joint3: %f" %last_pos3)

time.sleep(1)
################## Go to stand by position before starting circle ###########################
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
time.sleep(3)


'''
print("Input angle1:%f" %pos1)
print("Input angle2:%f" %pos2)
print("Input angle3:%f" %pos3)
print("Servo pulse1:%f" %servo_com1)
print("Servo pulse2:%f" %servo_com2)
print("Servo pulse3:%f" %servo_com3)
print("Period: %f" %period)
print("----------------------------")
'''

####################################################### Close Servo #############################################################
'''
# Disable Dynamixel Torque 1
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
'''


'''
t2 = time.time()
period = t2 - t1
print("time=%f" %period)
print("input_pos=%f" %pos)
print("servo_pos1=%f" %servo_pos1)
print("servo_pos2=%f" %servo_pos2)
print("servo_pos3=%f" %servo_pos3)
print("-----------------------------")
'''
