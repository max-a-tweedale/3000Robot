import numpy as np
import cv2

cap = cv2.VideoCapture(0)
class pen():
    def __init__(self):
        self.mode = 0
        self.colour = 0
        self.colours = ['red', 'green', 'blue']
        self.modes = ['off', 'draw', 'erase']
    def changeMode(self):
        #When called toggles the mode
        self.mode +=1
        if self.mode > len(self.modes):
            self.mode = 0
        return self.modes[self.mode]

    def changeColour(self):
        #When called toggles the pen colour
        self.colour +=1
        if self.colour > len(self.colours):
            self.colour = 0
        return self.colours[self.colour]

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    #main




    # Display the resulting frame
    cv2.imshow('frame', frame)

    key = cv2.waitKey(0) & 0xFF
   

    if key == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
