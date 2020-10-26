
# Written by Joshua Galland 5/10/2020
# For module 3 TRC3000 Sem 2 2020

import cv2 as cv
import time
import imutils
import argparse
from imutils.video import VideoStream
from imutils.video import FPS

#servo stuff
#import RPi.GPIO as GPIO
#import wiringpi
#import pigpio

import time
#### RPI GPIO software driving
# servoPin = 12
# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(servoPin,GPIO.OUT)
# pwm = GPIO.PWM(servoPin,100)
# pwm.start(0)

#### Using wiringPi but opencv not on root install
# servoPin = 1 #GPIO pin 12
# wiringpi.wiringPiSetup()
# wiringpi.pinMode(servoPin, 2)
# wiringpi.pwmWrite(servoPin,0)

### Using pigpio
import pigpio
Servo = pigpio.pi()
Servo.set_mode(12, pigpio.OUTPUT)
Servo.set_PWM_frequency(12, 8000) # set freq to 8kHz
Servo.set_PWM_dutycycle(12, 128)

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

servoPos = 128
def moveCamera(Head,frame_w,servoPos):
    (x,y) = Head
    mid_frame = int(frame_w/2)
    kp = 0.01
    error = mid_frame - x
    output = -kp*error
    
    servoPos = round(128 + output)
    print(servoPos)
    if servoPos >= 1 and servoPos<=256:
        #pwm.ChangeDutyCycle(servoPos)
        #wiringpi.pwmWrite(servoPos)
        Servo.set_PWM_dutycycle(12, servoPos)


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
    frame = cv.rotate(frame, cv.ROTATE_180)
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

            cv.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
            cv.circle(frame, (x+int(w/2),y+int(h/2)), 2, (255,0,0), 2)
            Head = (x+int(w/2), y+int(h/2))
            moveCamera(Head,frame_w,servoPos)

        fps.update()
        fps.stop()

        #Information for screen
        info = [
            ("Tracker", args['tracker']),
            ("Success", 'Yes' if success else 'No'),
            ('FPS', "{:.2f}".format(fps.fps())),
        ]

        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv.putText(frame, text, (10, frame_h - ((i * 20) + 20)), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            black = (179,179,255)
            cv.line(img = frame, pt1 = (int(frame_w/2),0), pt2 = (int(frame_w/2),frame_h), color = black, thickness =2, lineType = 8, shift = 0)

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

