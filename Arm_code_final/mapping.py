# Import EEZYbotARM library
from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk2
from easyEEZYbotARM.serial_communication import arduinoController
import time
# Insert your Arduino serial port here to initialise the arduino controller
myArduino = arduinoController(port="COM4")
myArduino.openSerialPort()
import numpy as np

# Initialise kinematic model with initial joint angles (home position)
robot = EEZYbotARM_Mk2(
    initial_q1=0, initial_q2=90, initial_q3=-90)
# Plot it
#myVirtualRobotArm.plot()
# Define end effector open and closed angle
servoAngle_EE_closed = 20
servoAngle_EE_open = 90

# Calculate the current servo angles
servoAngle_q1, servoAngle_q2, servoAngle_q3 = robot.map_kinematicsToServoAngles()
print("Initial = , ", servoAngle_q1, ", ", servoAngle_q2, ", ", servoAngle_q3)

# a1 = 0
# a2 = 45
# a3 = -45
# x, y, z = robot.forwardKinematics(q1=a1, q2=a2, q3=a3)
# print("xyz = , ", x, ", ", y, ", ", z)
Quit = False

def get_base_tv(angle):
    t_90 = 3600
    v_abs = 10
    angle_change = angle - robot.q1
    print("Robot is at ", robot.q1, " and is going to ", angle_change)
    if angle_change != 0:
        v_spin = round( angle/abs(angle)*v_abs )
        t_spin = round( abs(angle_change)/90*3500 )
        return t_spin, v_spin
    else:
        return 0, 0

mode = int(input("Input mode 1 (Test), 2 (Cal) , 3 (Run): "))

if mode == 1:
    while True:
        user_input = input("Enter angle or hit q: ")
        if user_input == 'q':
            print("Quitting")
            quit = True
            break
        elif user_input == 'b':
            break
        try:
            Q = int(user_input)
        except ValueError:
            continue
        t, v = get_base_tv(Q)
        print("Spinning for ", t)
        myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=v,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=servoAngle_EE_closed,
                                                            servo1_time=t))
elif mode == 2:
    v = -10
    while True:
        user_input = input("Enter time to run for or hit q: ")
        if user_input == 'q':
            print("Quitting")
            quit = True
            break
        elif user_input == 'b':
            break
        try:
            t = int(user_input)
        except ValueError:
            continue
        print("Spinning for ", t)
        myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=v,
                                                        servoAngle_q2=servoAngle_q2,
                                                        servoAngle_q3=servoAngle_q3,
                                                        servoAngle_EE=servoAngle_EE_closed, servo1_time=t)) #robot.get_base_time(servoAngle_q1)
        v *= -1
if mode == 3:
    # user_input = input("Input x y z : ")
    # user_input = user_input.split(" ")
    # try:
    #     x = int(user_input[0])
    #     y = int(user_input[1])
    #     z = (user_input[2])
    # except ValueError:
    #     print("Error with values")
    binGreen = [30, -280, 180] # then open.
    binRed = [-50, -280,180] # then open
    x,y,z = binRed
    # Compute inverse kinematics
    a1, a2, a3 = robot.inverseKinematics(x, y, z)
    t_q1, v_q1 = get_base_tv(a1)
    print("as = ", a1, ", ", a2, ", ", a3)
    robot.updateJointAngles(q1=a1, q2=a2, q3=a3)
    # robot.plot()

    # Calculate the current servo angles
    servoAngle_q1, servoAngle_q2, servoAngle_q3 = robot.map_kinematicsToServoAngles()
    print("servoAngles = ", servoAngle_q1, ", ", servoAngle_q2, ", ", servoAngle_q3)

    print("Spinning for ", t_q1)
    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=v_q1,
                                                        servoAngle_q2=servoAngle_q2,
                                                        servoAngle_q3=servoAngle_q3,
                                                        servoAngle_EE=servoAngle_EE_closed,
                                                        servo1_time=t_q1))
    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=0,
                                                        servoAngle_q2=servoAngle_q2,
                                                        servoAngle_q3=servoAngle_q3,
                                                        servoAngle_EE=servoAngle_EE_open))

# Close the serial port
myArduino.closeSerialPort()
