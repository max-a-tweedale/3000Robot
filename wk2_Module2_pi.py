import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import matplotlib.pyplot as plt

camera = PiCamera()
camera.rotation =180
camera.resolution = (640,480)
camera.framerate = 32

x = []
y = []
#cap = cv2.VideoCapture(0)
rawCapture = PiRGBArray(camera, size=(640, 480))

class PenController():
    def __init__(self):
        self.mode = 0
        self.colourInt = 0
        self.colours = ['red', 'green', 'blue']
        self.coloursRGB = [(255,0,0), (0,255,0), (0,0,255)]
        self.colour = self.coloursRGB[self.colourInt]
        self.modes = ['off', 'draw', 'erase']
    def changeMode(self):
        #When called toggles the mode
        #returns string of mode name
        self.mode +=1
        if self.mode > len(self.modes):
            self.mode = 0
        return self.modes[self.mode]

    def changeColour(self):
        #When called toggles the pen colour
        #returns colour RGB touple
        self.colourInt +=1
        if self.colourInt > len(self.colours):
            self.colourInt = 0
        self.colour = self.coloursRGB[self.colourInt]
        return self.coloursRGB[self.colourInt]
    
    def currentColour(self):
        #when called it returns touple of
        #string of colour ie 'red' and RGB value for current colour
        return self.colours[self.colourInt], self.coloursRGB[self.colourInt]
    
def CalculateThreshold(grayFrame):
    #takes image array grayscaled and calculates the spread of black and white pixels.
    ly = len(grayFrame)
    lx = len(grayFrame[0])
    pixFreq= [0]*256
    for j in range(ly):
        for i in range(lx):
            pixFreq[grayFrame[j][i]] += 1
    #print(pixFreq)
    plt.figure(1)
    plt.plot(pixFreq)
    plt.show()
    
    
    
pen = PenController()
pen.changeColour()
for f in camera.capture_continuous(rawCapture,format="bgr",use_video_port=True):
    # Capture frame-by-frame
    frame = f.array

    #main
    
    #Mirror the frame for drawing easily
    frame = cv2.flip(frame,+1)
    
    #Convert the frame to grayscale and blur it slightly
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(5,5),0)
    
    #Threshold the image, apply erosions and dilations to remove small regions of noise
    #Threshold numbers may need to be changed to adapt to the ambient light
    #CalculateThreshold(gray)
    thresh1 = cv2.threshold(gray,60,150,cv2.THRESH_BINARY)[1]
    thresh = cv2.erode(thresh1,None,iterations=2)
    mask = cv2.dilate(thresh,None,iterations=2)
    
    #Apply the mask on the original image
    combine = cv2.bitwise_and(frame,frame,mask=mask)
    
    #Find the hand and its centroid
    _,contours,_ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    hand = []
    for contour in contours:
        if cv2.contourArea(contour)>1000:
            hand = contour
    if len(hand)>1:
        M = cv2.moments(hand)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        
        #Find the nib which is the toppest point of the finger
        finger = tuple(hand[hand[:,:,1].argmin()][0])
        #Two numbers should be changed to adapted to the finger move
        #print(cx-finger[0]) or print(cy-finger[1]) to determined the range
        if abs(cx - finger[0])<150 & abs(cy - finger[1])>50:
            x.append(finger[0])
            y.append(finger[1])
            
        #print(finger)
        #Drawing
        for i in range(len(x)):
            cv2.circle(frame,(x[i],y[i]),3,pen.colour,3)


    # Display the resulting frame
    cv2.imshow('frame', thresh1)

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    
    if cv2.waitKey(1) & 0xFF == ord('q'): # if q typed then exitq
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

