#!!!! Make sure to start the pigpio daemon in a terminal using the command: sudo pigpiod

#image libraries
import cv2 as cv
import time
import imutils
import argparse
from imutils.video import VideoStream
from imutils.video import FPS

#servo libraries
import RPi.GPIO as GPIO
import pigpio
import time

servo = 18 #GPIO 18 corresponds to pin 11 in reality

#Use pigpio to set a PWM
pwm = pigpio.pi()
pwm.set_mode(servo,pigpio.OUTPUT)
pwm.set_PWM_frequency(servo,50) #setting the PWM to 50hz, this is what the servo was designed for


#initialising servo ranges
maxServoPos = 2500 #Pulse width of 2500 microseconds
minServoPos = 500 #Pulse width of 500 microseconds
midServoPos = minServoPos+(maxServoPos-minServoPos)/2 #Middle of servo = 1500 microseconds

servoPos = midServoPos #initialising variable which will store servo values
pwm.set_servo_pulsewidth(servo,servoPos) #setting servo to 90 degreees initially

#initialising variables that store values for PID
integral = 0
prev_error = 0
timer = 0
timerFlag = 0
startoftimer = 0
endoftimer = 0
#This moveCamera function takes in the width of the frame and the position of the head. It outputs the position of the servo 
def moveCameraPID(Head,frame_w): 
    #global variables the function alters
    global integral
    global prev_error
    global servoPos
    global timer
    global timerFlag
    global startoftimer
    global endoftimer
    
    (x,y) = Head
    
    #finding centre point of frame
    mid_frame = int(frame_w/2)
       
    #proportional, integral and derivative constants
    Kp = .15
    Ki = .4
    Kd = .3 
    
    
    error = mid_frame - x #difference between disired pos and actual pos
    integral+=error
    derivative = error-prev_error
    
    ut = -(Kp * error + Ki * integral + Kd * derivative) 
    
    servoPos = midServoPos + ut
    
    #ensuring servo output is within physical limitations
    if servoPos > maxServoPos:
        servoPos = maxServoPos
        
    if servoPos < minServoPos:
        servoPos = minServoPos
        
    pwm.set_servo_pulsewidth(servo,servoPos)     
    prev_error =error
    
    #checking to see if head is out of the 10% boundary        
    successframe_w = frame_w * 0.05 # = 25, 10% of the frame width is 50 pixels
    
    #calculating response time
    if (error<-successframe_w or error>successframe_w) and timerFlag == 0:
        startoftimer = time.perf_counter()
        timerFlag = 1
        
    if (error>-successframe_w and error<successframe_w) and timerFlag == 1:

        endoftimer = time.perf_counter()
        timerFlag = 0
        timer = endoftimer - startoftimer 
    
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
            moveCameraPID(Head,frame_w) #parse in centre of head and width of frame, this function will control the servo


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



