

import cv2 as cv
from imutils.video import VideoStream
import numpy as np

def maxsize(item):
    area = item[2]*item[3] # w*h
    return area

def findGreen(img):
    frame = img.copy()
    kernelOpen = np.ones((5, 5))
    kernelClose = np.ones((20, 20))
    lowerBound = np.array([32, 80, 30])
    upperBound = np.array([102, 255, 255])
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
    greenItems.sort(key=maxsize)
    if len(greenItems) >0:
        x,y,w,h = greenItems[0]
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        return [x+w/2, y+h/2] # main centre
    else:
        return None


if __name__ == "__main__":
    
    print("Webcam Turning on...")
    vs = VideoStream(src=0).start()
    while True:
        frame = vs.read()
        #frame = frame[1] if args.get("video", False) else frame
        #if end of stream has been reached
        if frame is None:
            break

        findGreen(frame)
        cv.imshow("Stream", frame)
        key = cv.waitKey(1) & 0xFF
        if key == ord('q'): #quit
            break


