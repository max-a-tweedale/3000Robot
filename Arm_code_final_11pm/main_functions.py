from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk2
from easyEEZYbotARM.serial_communication import arduinoController
import time
import numpy as np
import cv2 as cv




class easyRobot:
    
    servoAngle_EE_closed = 15
    servoAngle_EE_open = 90
    bin = {
          "red": [-20, -220,220],
          "green": [100, -220, 220],
          "blue": [-220,-220,220]
        }
    
    def __init__(self, arduino_port = "/dev/ttyUSB0"):
        self.arduino = arduinoController(port=arduino_port)
        self.arduino.openSerialPort()
        self.arm = EEZYbotARM_Mk2(initial_q1=0, initial_q2=90, initial_q3=-90)
        self.homePos = [0, 110, -140]
        self.return_home()
        
    def __del__(self):
        self.arduino.closeSerialPort()
    
    def get_base_tv(self, angle):
        if angle >= 0:
            t_90 = 3700
        else:
            t_90 = 3400
        v_abs = 10
        angle_change = angle - self.arm.q1
        
        if angle_change != 0:
            v_spin = round( angle_change/abs(angle_change)*v_abs )
            t_spin = round( abs(angle_change)/90*t_90)
            print("self.arm is at ", self.arm.q1, " and is going to ", angle_change, " for ", t_spin, "s")
            return t_spin, v_spin
        else:
            return 0, 0

    def go_to(self, location):
        x,y,z = location
        
        ## Move upward
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = self.arm.map_kinematicsToServoAngles()
        self.arduino.communicate(data=self.arduino.composeMessage(servoAngle_q1=0,
                                                            servoAngle_q2=servoAngle_q2-20,
                                                            servoAngle_q3=servoAngle_q3+45,
                                                            servoAngle_EE=self.servoAngle_EE_closed))
        # Compute inverse kinematics
        a1, a2, a3 = self.arm.inverseKinematics(x, y, z)
        t_q, v_q = self.get_base_tv(a1)
        self.arm.updateJointAngles(q1=a1, q2=a2, q3=a3)

        # Calculate the current servo angles
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = self.arm.map_kinematicsToServoAngles()
        self.arduino.communicate(data=self.arduino.composeMessage(servoAngle_q1=v_q,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=self.servoAngle_EE_closed,servo1_time=t_q))
    def return_home(self):
        a1, a2, a3 = self.homePos
        t_q1, v_q1 = self.get_base_tv(a1)
        self.arm.updateJointAngles(q1=a1, q2=a2, q3=a3)
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = self.arm.map_kinematicsToServoAngles()
        self.arduino.communicate(data=self.arduino.composeMessage(servoAngle_q1=v_q1,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=self.servoAngle_EE_closed,
                                                            servo1_time=t_q1))
        
        
    def closeEE(self):
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = self.arm.map_kinematicsToServoAngles()
        self.arduino.communicate(data=self.arduino.composeMessage(servoAngle_q1=0,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=self.servoAngle_EE_closed))

    def openEE(self):
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = self.arm.map_kinematicsToServoAngles()
        self.arduino.communicate(data=self.arduino.composeMessage(servoAngle_q1=0,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=self.servoAngle_EE_open))
        
        
    def approach_target(self, target):
        x,y,z = target
        x1 = x*3/4
        y1 = y*3/4
        z1 = 100
        a1_1, a2_1, a3_1 = self.arm.inverseKinematics(x1, y1, z1)
        t_q1, v_q1 = self.get_base_tv(a1_1)
        self.arm.updateJointAngles(q1=a1_1, q2=a2_1, q3=a3_1)
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = self.arm.map_kinematicsToServoAngles()
        self.arduino.communicate(data=self.arduino.composeMessage(servoAngle_q1=v_q1,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=self.servoAngle_EE_open,
                                                            servo1_time=t_q1))
        a1, a2, a3 = self.arm.inverseKinematics(x, y, z)
        t_q, v_q = self.get_base_tv(a1)
        self.arm.updateJointAngles(q1=a1, q2=a2, q3=a3)
        # Calculate the current servo angles
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = self.arm.map_kinematicsToServoAngles()
        self.arduino.communicate(data=self.arduino.composeMessage(servoAngle_q1=v_q,
                                                            servoAngle_q2=servoAngle_q2,
                                                            servoAngle_q3=servoAngle_q3,
                                                            servoAngle_EE=self.servoAngle_EE_open,servo1_time=t_q1))
