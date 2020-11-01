# Import EEZYbotARM library
from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk2
from easyEEZYbotARM.serial_communication import arduinoController

# Insert your Arduino serial port here to initialise the arduino controller
myArduino = arduinoController(port="COM4")
myArduino.openSerialPort()

# Initialise kinematic model with initial joint angles (home position)
myVirtualRobotArm = EEZYbotARM_Mk2(
    initial_q1=0, initial_q2=90, initial_q3=-90)
# Plot it
#myVirtualRobotArm.plot()
# Define end effector open and closed angle
servoAngle_EE_closed = 20
servoAngle_EE_open = 90

# Calculate the current servo angles
# servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()
# print("Initial = , ", servoAngle_q1, ", ", servoAngle_q2, ", ", servoAngle_q3)
#
# # Send the movement command to the arduino. The physical EEZYbotARM will move to this position
# myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=0,
#                                                     servoAngle_q2=servoAngle_q2,
#                                                     servoAngle_q3=servoAngle_q3,
#                                                     servoAngle_EE=servoAngle_EE_closed))
a1 = 0
a2 = 45
a3 = -45
x, y, z = myVirtualRobotArm.forwardKinematics(q1=a1, q2=a2, q3=a3)
print("xyz = , ", x, ", ", y, ", ", z)

x = 250
y = 0
z = 50

# Compute inverse kinematics
a1, a2, a3 = myVirtualRobotArm.inverseKinematics(x, y, z)
#print("as = , ", a1, ", ", a2, ", ", a3)
myVirtualRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
myVirtualRobotArm.plot()

# Calculate the current servo angles
servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()
print("servoAngles = , ", servoAngle_q1, ", ", servoAngle_q2, ", ", servoAngle_q3)

# Send the movement command to the arduino. The physical EEZYbotARM will move to this position
myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=0,
                                                    servoAngle_q2=servoAngle_q2,
                                                    servoAngle_q3=servoAngle_q3,
                                                    servoAngle_EE=servoAngle_EE_closed))

# Close the serial port
myArduino.closeSerialPort()
