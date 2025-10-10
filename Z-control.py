from math import *
#!/usr/bin/env python
#
# *********     Sync Read Example      *********
#
#
# Available STServo model on this example : All models using Protocol STS
# This example is tested with a STServo and an URT
#

import sys
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from STservo_sdk import *                       # Uses STServo SDK library

# Default setting
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
DEVICENAME                  = '/dev/ttyACM0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = sts(portHandler)

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


l1 = 3.0   
l2 = 3.5   
l3 = 10.5  
l4 = 2.7   
l5 = 9.8   
l6 = 10.5  

#Constant Vars
MAX_SERVO_POSITION = 4095
MIN_SERVO_POSITION = 0

#Motor offsets
motor_offsets = [430, 279, 537, 300, 331, 192, 330, 264]
servo_pos = {}

def initialize():
    for i in range(1, 9):
        if i <= 4:
            servo_pos[str(i)] = MIN_SERVO_POSITION
        else:
            servo_pos[str(i)] = MAX_SERVO_POSITION

def IK_solver(coord_X, coord_Y):
    k1 = (coord_X**2 + l5**2 + l1**2 + coord_Y**2 - l6**2 + 2*coord_Y*l1)/(2*l5)
    theta5 = 2 * atan2(coord_X + sqrt(pow((coord_Y + l1), 2) + pow(coord_X, 2) - pow(k1, 2)), (k1 - coord_Y - l1))
    theta6 = atan2(-(coord_Y + l5 * cos(theta5) + l1), (-coord_X + l5 * sin(theta5)))
    x2 = coord_X + (l4 + l6) * cos(theta6)
    y2 = coord_Y + (l4 + l6) * sin(theta6)
    k2 = (pow(x2, 2) + pow(y2, 2) + pow(l2, 2) - pow(l3, 2)) / (2 * l2)
    theta1 = 2 * atan2((x2 + sqrt(pow(x2, 2) + pow(y2, 2) - pow(k2, 2))), (k2 - y2))
    angles_deg = [205 - theta1 * (180.0 / pi), 120 - theta5 * (180.0 / pi)]
    print(f"Joint angles for reaching {coord_X}, {coord_Y} = ", angles_deg)
    return angles_deg


def get_pos():
    position_matrix = [0 for i in range(8)]
    groupSyncRead = GroupSyncRead(packetHandler, STS_PRESENT_POSITION_L, 4)
    for sts_id in range(1, 9):
        # Add parameter storage for STServo#1~10 present position value
        sts_addparam_result = groupSyncRead.addParam(sts_id)
        if sts_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % sts_id)

    sts_comm_result = groupSyncRead.txRxPacket()
    if sts_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(sts_comm_result))

    for sts_id in range(1, 11):
        # Check if groupsyncread data of STServo#1~10 is available
        sts_data_result, sts_error = groupSyncRead.isAvailable(sts_id, STS_PRESENT_POSITION_L, 4)
        if sts_data_result == True:
            # Get STServo#scs_id present position value
            sts_present_position = groupSyncRead.getData(sts_id, STS_PRESENT_POSITION_L, 2)
            sts_present_speed = groupSyncRead.getData(sts_id, STS_PRESENT_SPEED_L, 2)
            position_matrix[sts_id - 1] = sts_present_position
            #print("[ID:%03d] PresPos:%d PresSpd:%d" % (sts_id, sts_present_position, packetHandler.sts_tohost(sts_present_speed, 15)))
        else:
            print("[ID:%03d] groupSyncRead getdata failed" % sts_id)
            continue
        if sts_error != 0:
            print("%s" % packetHandler.getRxPacketError(sts_error))
    groupSyncRead.clearParam()
    return position_matrix
    
def main(setpoint):
    print(f"Height = {setpoint}cm..")
    joint_angles = IK_solver(0, setpoint)
    for i in range(1, 9):
        if i <= 4:
            servo_pos[str(i)] = joint_angles[i%2] + motor_offsets[i-1]
        else:
            servo_pos[str(i)] = MAX_SERVO_POSITION - joint_angles[i%2] - motor_offsets[i-1]

    print("Joint Angles for Respective Legs and Motors")
    for i in range(1, 9):
        print("Motor", i, ' :' ,servo_pos[str(i)])
    samples = list(servo_pos.values())
    sample_per_cycle = [0 for i in range(9)]
    remainders_per_servo = [0 for i in range(9)]
    for i in range(len(samples)):
        sample_per_cycle[i] = samples[i] // 10
        remainders_per_servo[i] = samples[i] - sample_per_cycle[i]*10
    print(f"Samples : {samples}")
    print(f"Sample per servo : {sample_per_cycle}")
    print(f"Remainders per servo : {remainders_per_servo}")

    Kp = 7
    Kd = 0.5
    iterator = 0
    current_error = 0
    prev_error = 0
    sum_error = 0
    while 1:
        current_pos = get_pos()
        current_error = samples[iterator] - current_pos
        
main(-10)
    

    