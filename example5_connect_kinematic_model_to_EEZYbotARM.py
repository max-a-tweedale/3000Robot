# Import EEZYbotARM library
from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk2
from easyEEZYbotARM.serial_communication import arduinoController

# Insert your Arduino serial port here to initialise the arduino controller
myArduino = arduinoController(port='/dev/ttyUSB0')
myArduino.openSerialPort()

# Initialise kinematic model with initial joint angles (home position)
myVirtualRobotArm = EEZYbotARM_Mk2(
    initial_q1=0, initial_q2=90, initial_q3=-130)
# Plot it
myVirtualRobotArm.plot()

# Define end effector open and closed angle
servoAngle_EE_closed = 20
servoAngle_EE_open = 90

# Calculate the current servo angles
servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()

# Send the movement command to the arduino. The physical EEZYbotARM will move to this position
myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=servoAngle_q1,
                                                    servoAngle_q2=servoAngle_q2,
                                                    servoAngle_q3=servoAngle_q3,
                                                    servoAngle_EE=servoAngle_EE_closed))


# Assign new cartesian position where we want the robot arm end effector to move to
# (x,y,z in mm from centre of robot base)
x = 240  # mm
y = 85  # mm
z = 200  # mm

# Compute inverse kinematics
a1, a2, a3 = myVirtualRobotArm.inverseKinematics(x, y, z)

# Print the result
print('To move the end effector to the cartesian position (mm) x={}, y={}, z={}, the robot arm joint angles (degrees)  are q1 = {}, q2= {}, q3 = {}'.format(x, y, z, a1, a2, a3))

# Visualise the new joint angles
myVirtualRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
myVirtualRobotArm.plot()

# Calculate the current servo angles
servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()

# Send the movement command to the arduino. The physical EEZYbotARM will move to this position
myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=servoAngle_q1,
                                                    servoAngle_q2=servoAngle_q2,
                                                    servoAngle_q3=servoAngle_q3,
                                                    servoAngle_EE=servoAngle_EE_open))

# Close the serial port
myArduino.closeSerialPort()
