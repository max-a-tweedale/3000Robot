import numpy as np
import cv2

x = []
y = []
cap = cv2.VideoCapture(0)
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
    
pen = PenController()

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    #main
    

    #Mirror the frame for drawing easily
    frame = cv2.flip(frame,+1)
    
    #Convert the frame to grayscale and blur it slightly
    #
    # #Convert the frame to grayscale and blur it slightly
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #Threshold the image, apply erosions and dilations to remove small regions of noise
    #Threshold numbers may need to be changed to adapt to the ambient light
    thresh = cv2.threshold(gray,140,255,cv2.THRESH_BINARY)[1]
    # gray = cv2.GaussianBlur(gray,(5,5),0)
    #
    # #Threshold the image, apply erosions and dilations to remove small regions of noise
    # #Threshold numbers may need to be changed to adapt to the ambient light
    thresh = cv2.threshold(gray,250,255,cv2.THRESH_BINARY)[1]
    thresh = cv2.erode(thresh,None,iterations=2)
    mask = cv2.dilate(thresh,None,iterations=2)
    
    #Apply the mask on the original image
    #
    # #Apply the mask on the original image
    combine = cv2.bitwise_and(frame,frame,mask=mask)
    
    #Find the hand and its centroid
    #
    # #Find the hand and its centroid
    _,contours,_ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour)>1000:
            hand = contour
    M = cv2.moments(hand)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    
    #Find the nib which is the toppest point of the finger

    # #Find the nib which is the toppest point of the finger
    finger = tuple(hand[hand[:,:,1].argmin()][0])
    #Two numbers should be changed to adapted to the finger move
    #print(cx-finger[0]) or print(cy-finger[1]) to determined the range
    if abs(cx - finger[0])<150 & abs(cy - finger[1])>50:
        x.append(finger[0])
        y.append(finger[1])
        
    #
    #Drawing
    for i in range(len(x)):
        cv2.circle(frame,(x[i],y[i]),3,pen.colour,3)
    # Display the resulting frame
    cv2.imshow('frame', frame)


    if key == ord('q'):

        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
