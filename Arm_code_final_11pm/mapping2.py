# Import EEZYbotARM library
from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk2
from easyEEZYbotARM.serial_communication import arduinoController
import time
import easyEEZYbotARM
import cv2 as cv
from imutils.video import VideoStream


# Insert your Arduino serial port here to initialise the arduino controller
myArduino = arduinoController(port="/dev/ttyUSB0")
myArduino.openSerialPort()
import numpy as np

# Initialise kinematic model with initial joint angles (home position)
robot = EEZYbotARM_Mk2(
    initial_q1=0, initial_q2=90, initial_q3=-90)
# Plot it
#myVirtualRobotArm.plot()
# Define end effector open and closed angle
servoAngle_EE_closed = 15
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

def go_to(location):
    x,y,z = location
    servoAngle_q1, servoAngle_q2, servoAngle_q3 = robot.map_kinematicsToServoAngles()
    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=0,
                                                        servoAngle_q2=servoAngle_q2,
                                                        servoAngle_q3=servoAngle_q3+45,
                                                        servoAngle_EE=servoAngle_EE_closed))
    # Compute inverse kinematics
    a1, a2, a3 = robot.inverseKinematics(x, y, z)
    t_q, v_q = get_base_tv(a1)
    print("as = ", a1, ", ", a2, ", ", a3)
    robot.updateJointAngles(q1=a1, q2=a2, q3=a3)

    # Calculate the current servo angles
    servoAngle_q1, servoAngle_q2, servoAngle_q3 = robot.map_kinematicsToServoAngles()
    print("servoAngles = ", servoAngle_q1, ", ", servoAngle_q2, ", ", servoAngle_q3)
    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=v_q,
                                                        servoAngle_q2=servoAngle_q2,
                                                        servoAngle_q3=servoAngle_q3,
                                                        servoAngle_EE=servoAngle_EE_closed,servo1_time=t_q))
def closeEE():
    servoAngle_q1, servoAngle_q2, servoAngle_q3 = robot.map_kinematicsToServoAngles()
    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=0,
                                                        servoAngle_q2=servoAngle_q2,
                                                        servoAngle_q3=servoAngle_q3,
                                                        servoAngle_EE=servoAngle_EE_closed))

def openEE():
    servoAngle_q1, servoAngle_q2, servoAngle_q3 = robot.map_kinematicsToServoAngles()
    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=0,
                                                        servoAngle_q2=servoAngle_q2,
                                                        servoAngle_q3=servoAngle_q3,
                                                        servoAngle_EE=servoAngle_EE_open))
    
    
def approach_target(target):
    x,y,z = target
    x1 = x*3/4
    y1 = y*3/4
    z1 = 100
    # Compute inverse kinematics
    a1_1, a2_1, a3_1 = robot.inverseKinematics(x1, y1, z1)
    t_q1, v_q1 = get_base_tv(a1_1)
    print("as = ", a1_1, ", ", a2_1, ", ", a3_1)
    robot.updateJointAngles(q1=a1_1, q2=a2_1, q3=a3_1)

    # Calculate the current servo angles
    servoAngle_q1, servoAngle_q2, servoAngle_q3 = robot.map_kinematicsToServoAngles()
    print("servoAngles = ", servoAngle_q1, ", ", servoAngle_q2, ", ", servoAngle_q3)

    print("Spinning for ", t_q1)
    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=v_q1,
                                                        servoAngle_q2=servoAngle_q2,
                                                        servoAngle_q3=servoAngle_q3,
                                                        servoAngle_EE=servoAngle_EE_open,
                                                        servo1_time=t_q1))
    # Compute inverse kinematics
    a1, a2, a3 = robot.inverseKinematics(x, y, z)
    t_q, v_q = get_base_tv(a1)
    print("as = ", a1, ", ", a2, ", ", a3)
    robot.updateJointAngles(q1=a1, q2=a2, q3=a3)

    # Calculate the current servo angles
    servoAngle_q1, servoAngle_q2, servoAngle_q3 = robot.map_kinematicsToServoAngles()
    print("servoAngles = ", servoAngle_q1, ", ", servoAngle_q2, ", ", servoAngle_q3)
    time.sleep(1)
    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=v_q,
                                                        servoAngle_q2=servoAngle_q2,
                                                        servoAngle_q3=servoAngle_q3,
                                                        servoAngle_EE=servoAngle_EE_open,servo1_time=t_q1))
    time.sleep(1)
    
def go_to(target):
    x,y,z = target
    
    # Compute inverse kinematics
    a1, a2, a3 = robot.inverseKinematics(x, y, z)
    t_q, v_q = get_base_tv(a1)
    print("as = ", a1, ", ", a2, ", ", a3)
    robot.updateJointAngles(q1=a1, q2=a2, q3=a3)

    # Calculate the current servo angles
    servoAngle_q1, servoAngle_q2, servoAngle_q3 = robot.map_kinematicsToServoAngles()
    print("servoAngles = ", servoAngle_q1, ", ", servoAngle_q2, ", ", servoAngle_q3)
    time.sleep(1)
    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=v_q,
                                                        servoAngle_q2=servoAngle_q2,
                                                        servoAngle_q3=servoAngle_q3,
                                                        servoAngle_EE=servoAngle_EE_closed,servo1_time=t_q))
    
def get_base_tv(angle):
    if angle >= 0:
        t_90 = 3700
    else:
        t_90 = 3400
    v_abs = 10
    angle_change = angle - robot.q1
    
    if angle_change != 0:
        v_spin = round( angle/abs(angle)*v_abs )
        t_spin = round( abs(angle_change)/90*t_90)
        print("Robot is at ", robot.q1, " and is going to ", angle_change, " for ", t_spin, "s")
        return t_spin, v_spin
    else:
        return 0, 0

mode = int(input("Input mode 1 (Test), 2 (Cal) , 3 (Run), 4 (goto): "))

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
elif mode == 3:
    target = [250,0,20]
    approach_target(target)
    closeEE()
    binGreen = [60, -220, 180] # then open.
    binRed = [-60, -220,180] # then open
    go_to(binGreen)
    openEE()
elif mode == 4:
    ## Using webcam
    print("Webcam Turning on...")
    vs = VideoStream(src=0).start()
    time.sleep(1.0)
    
    target = [300,0,150]
    go_to(target)
    frame = vs.read()
    cv.imshow("Stream", frame)
    cv.waitKey(0)

    
# Close the serial port
myArduino.closeSerialPort()
