

import cv2 as cv

import numpy as np

def findGreen(img):
    frame = img.copy()
    kernelOpen = np.ones((5, 5))
    kernelClose = np.ones((20, 20))
    #green
    #lowerBound = np.array([32, 80, 30])
    #upperBound = np.array([102, 255, 255])
    #black
    lowerBound = np.array([0, 0, 0])
    upperBound = np.array([360, 127, 51])
    frame = cv.cvtColor(frame, cv.COLOR_RGB2HSV)
    frame = cv.inRange(frame, lowerBound, upperBound)
    frame = cv.morphologyEx(frame, cv.MORPH_OPEN, kernelOpen)
    frame = cv.morphologyEx(frame, cv.MORPH_CLOSE, kernelClose)
    conts, h = cv.findContours(frame.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    greenItems = []
    for i in range(len(conts)):
        x, y, w, h = cv.boundingRect(conts[i])
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        corners = [[x, y], [x + w, y + h]]
        greenItems.append(corners)
    return greenItems
def findGreenTest(img):
    frame = img.copy()
    y, x, z = frame.shape
    print(frame.shape)
    print(frame[0,900,0])
    kernelOpen = np.ones((5, 5))
    kernelClose = np.ones((20, 20))

    lowerBound = np.array([32, 80, 30])
    upperBound = np.array([102, 255, 255])

    # for px in range(x):
    #     for py in range(y):
    #         condR = frame[py,px,0] < 5
    #         condG = (frame[py,px,1] > 123-20) & (frame[py][px][1] < 123+20)
    #         condB = frame[py,px,2] < 5
    #         if condR and condG and condB:
    #             #frame[py,px] = 255
    #             print(px,",", py)
    #             print(frame[py,px])
    #         else:
    #             frame[py, px] = 0

    frame= cv.cvtColor(img, cv.COLOR_RGB2HSV)
    frame=cv.inRange(frame,lowerBound,upperBound)

    maskOpen = cv.morphologyEx(frame, cv.MORPH_OPEN, kernelOpen)
    frame = cv.morphologyEx(maskOpen, cv.MORPH_CLOSE, kernelClose)
    conts, h = cv.findContours(frame.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    greenItems = []
    for i in range(len(conts)):
        x, y, w, h = cv.boundingRect(conts[i])
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        corners = [[x,y],[x+w,y+h]]
        greenItems.append(corners)
    cv.imshow("img", img)
    cv.waitKey(0)
    return greenItems


if __name__ == "__main__":
    file_name = 'GreenMarkerScan.png'
    img = cv.imread(file_name)

    findGreen(img)


