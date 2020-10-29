#!!!! Make sure to start the pigpio daemon in a terminal using the command: sudo pigpiod

#image libraries
import cv2 as cv
import time
import imutils
import argparse
from imutils.video import VideoStream
from imutils.video import FPS

#servo libraries

import time
from kinematic_model import EEZYbotARM_Mk2
from serial_communication import arduinoController

# Insert your Arduino serial port here to initialise the arduino controller
myArduino = arduinoController(port="/dev/ttyUSB0") #ACM0 for Max, USB0 for Josh
myArduino.openSerialPort()

# Initialise kinematic model with initial joint angles (home position)
myVirtualRobotArm = EEZYbotARM_Mk2(
    initial_q1=0, initial_q2=90, initial_q3=-130)
# Plot it
x_pos = 200  # mm
y_pos = 0  # mm
z_pos = 100  # mm
# Define end effector open and closed angle
servoAngle_EE_closed = 20
servoAngle_EE_open = 90

a1, a2, a3 = myVirtualRobotArm.inverseKinematics(x_pos, y_pos, z_pos)
myVirtualRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
# Calculate the current servo angles
servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()

# Send the movement command to the arduino. The physical EEZYbotARM will move to this position
myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=servoAngle_q1,
                                                    servoAngle_q2=servoAngle_q2,
                                                    servoAngle_q3=servoAngle_q3,
                                                    servoAngle_EE=servoAngle_EE_open))
#initialising variables that store values for PID
integral = 0
prev_errorx = 0
timer = 0
timerFlag = 0
startoftimer = 0
endoftimer = 0
a1 =90
#This moveCamera function takes in the width of the frame and the position of the head. It outputs the position of the servo 
def moveCameraPID(Head,frame_h,frame_w): 
    #global variables the function alters
    global integral
    global prev_errorx
    global servoPos
    global timer
    global timerFlag
    global startoftimer
    global endoftimer
    global x_pos
    global y_pos
    global z_pos
    global a1
    
    (x,y) = Head
    
    #finding centre point of frame
    mid_framex = int(frame_w/2)
    mid_framey = int(frame_h/2)
       
    #proportional, integral and derivative constants
    Kp = .1
    Ki = .0
    Kd = .0 
    
    
    errorx = mid_framex - x #difference between disired pos and actual pos
    integral+=errorx
    derivative = errorx-prev_errorx
    
    ut = -(Kp * errorx + Ki * integral + Kd * derivative) 
    
    x_pos = x_pos + ut
    
    errory = mid_framey - y
    yt = (.1 * errory)
    a1 = a1 + yt
    
    #ensuring servo output is within physical limitations
    #if servoPos > maxServoPos:
        #servoPos = maxServoPos
        
    #if servoPos < minServoPos:
        #servoPos = minServoPos
        
    #pwm.set_servo_pulsewidth(servo,servoPos)
    a, a2, a3 = myVirtualRobotArm.inverseKinematics(x_pos, y_pos, z_pos)
    myVirtualRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
# Calculate the current servo angles
    servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()

    # Send the movement command to the arduino. The physical EEZYbotARM will move to this position
    myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=a1,
                                                    servoAngle_q2=servoAngle_q2,
                                                    servoAngle_q3=servoAngle_q3,
                                                    servoAngle_EE=servoAngle_EE_open))
    
    prev_errorx =errorx
    
    #checking to see if head is out of the 10% boundary        
    successframe_w = frame_w * 0.05 # = 25, 10% of the frame width is 50 pixels
    
    #calculating response time
    if (errorx<-successframe_w or errorx>successframe_w) and timerFlag == 0:
        startoftimer = time.perf_counter()
        timerFlag = 1
        
    if (errorx>-successframe_w and errorx<successframe_w) and timerFlag == 1:

        endoftimer = time.perf_counter()
        timerFlag = 0
        timer = endoftimer - startoftimer
        
    print(errorx,' and ',errory)
    if errorx<=2 and errory<=2:
        x_pos = x_pos  # mm
        y_pos = y_pos  # mm
        z_pos = 20  # mm
        
        a, a2, a3 = myVirtualRobotArm.inverseKinematics(x_pos, y_pos, z_pos)
        myVirtualRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
# Calculate the current servo angles
        servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()

    # Send the movement command to the arduino. The physical EEZYbotARM will move to this position
        myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=a1,
                                                    servoAngle_q2=servoAngle_q2,
                                                    servoAngle_q3=servoAngle_q3,
                                                    servoAngle_EE=servoAngle_EE_closed))
        myArduino.closeSerialPort()

        

#def pickUpObject()
    
#Setting up trackers    
arg_p = argparse.ArgumentParser()
arg_p.add_argument('-t', '--tracker', type=str, default='mosse', help="OpenCV object tracker type")
args = vars(arg_p.parse_args())

## OpenCV version must be greater than 3.4
CV2_Trackers = {
    'kcf': cv.TrackerKCF_create,
    'csrt': cv.TrackerCSRT_create,
    'boosting': cv.TrackerBoosting_create,
    'mil': cv.TrackerMIL_create,
    'tld': cv.TrackerTLD_create,
    'medianflow': cv.TrackerMedianFlow_create,
    'mosse': cv.TrackerMOSSE_create
}        

tracker = CV2_Trackers[args['tracker']]()

## Using webcam
print("Webcam Turning on...")
vs = VideoStream(src=0).start()
time.sleep(1.0)

print("Press g to grab head\n Press q to exit\n Press exc to redo grab")

#Initialise tracking box
initBox = None
#Estimation of frames per second
fps = None
Head = (None,None)

#Main loop

while True:
    frame = vs.read()
    
    #It was more convenient to mount my camera upside down, therefore the image is rotated 180 degrees
    frame = cv.rotate(frame, cv.ROTATE_180)
    #Flipping image along vertical axis allows for more intuitive use
    frame = cv.flip(frame,1)
    frame = frame[1] if args.get("video", False) else frame
    

    #if end of stream has been reached
    if frame is None:
        break

    #resize frame so processing is faster and frab frame dimensions
    frame = imutils.resize(frame, width=500)
    (frame_h, frame_w) = frame.shape[:2]

    if initBox is not None:
        (success, box) = tracker.update(frame)

        if success:
            #draw rectangle
            (x,y,w,h) = [int(v) for v in box]

            cv.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2) #draws rectangle around tracked face
            cv.circle(frame, (x+int(w/2),y+int(h/2)), 2, (255,0,0), 2)
            
            Head = (x+int(w/2), y+int(h/2)) #this line calculates the centre point of the rectangle
            moveCameraPID(Head,frame_h,frame_w) #parse in centre of head and width of frame, this function will control the servo


        fps.update()
        fps.stop()

        #Information for screen
        info = [
            ("Tracker", args['tracker']),
            ("Success", 'Yes' if success else 'No'),
            ('FPS', "{:.2f}".format(fps.fps())),
            ("Response time: ", "{:.2f}".format(timer)),
        ]
                  

        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv.putText(frame, text, (10, frame_h - ((i * 20) + 20)), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            black = (179,179,255)
            right_line = int(frame_w/2 + frame_w * 0.05)
            left_line = int(frame_w/2 - frame_w * 0.05)
            
            #prints two lines on the screen that take up 10% of the width of the frame
            cv.line(img = frame, pt1 = (right_line,0), pt2 = (right_line,frame_h), color = black, thickness =2, lineType = 8, shift = 0) 
            cv.line(img = frame, pt1 = (left_line,0), pt2 = (left_line,frame_h), color = black, thickness =2, lineType = 8, shift = 0)

    #display frame
    cv.imshow("Stream", frame)
    key = cv.waitKey(1) & 0xFF

    if key == ord('q'): #quit
        break
    elif key == ord('g'): #grab box
        initBox = cv.selectROI("Stream", frame)
        tracker.init(frame, initBox)
        fps = FPS().start()


vs.stop()
cv.destroyAllWindows()




