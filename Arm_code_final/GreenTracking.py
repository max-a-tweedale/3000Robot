

import cv2 as cv
from imutils.video import VideoStream
import time
import numpy as np

def getArea(item):
    x,y,w,h = item
    return w*h

def findGreen(img):
    frame = img.copy()

    #colour bounds
    lowerBound = np.array([0, 0, 0])
    upperBound = np.array([360, 127, 51])
    kernelOpen = np.ones((5, 5))
    kernelClose = np.ones((20, 20))
    frame = cv.cvtColor(frame, cv.COLOR_RGB2HSV)
    frame = cv.inRange(frame, lowerBound, upperBound)
    frame = cv.morphologyEx(frame, cv.MORPH_OPEN, kernelOpen)
    frame = cv.morphologyEx(frame, cv.MORPH_CLOSE, kernelClose)
    conts, h = cv.findContours(frame.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    greenItems = []
    for i in range(len(conts)):
        x, y, w, h = cv.boundingRect(conts[i])
        corners = [x, y, w, h]
        greenItems.append(corners)
    greenItems.sort(key=getArea)
    if len(greenItems)>0:
        x,y,w,h = greenItems[-1] #largest item
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        return [x+w/2, y+w/2]
    else:
        return None
    


if __name__ == "__main__":
    ## Using webcam
    print("Webcam Turning on...")
    vs = VideoStream(src=0).start()
    time.sleep(1.0)
    while True:
        frame = vs.read()
        # if end of stream has been reached
        if frame is None:
            break
        green_item = findGreen(frame)
        # display frame
        cv.imshow("Stream", frame)
        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):  # quit
            break
