import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)
cap.set(10, 70)

classNames = []
classFile = 'coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
    classNames = np.array(classNames)

configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# two arrays to store objects position
center_X = []
center_Y = []

while True:
    success, frame = cap.read()
    frame = cv2.flip(frame, +1)

    # Convert the frame in BGR(RGB color space) to HSV(hue-saturation-value)
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # The threshold values can be changed
    # Set range for orange color and define mask
    orange_lower = np.array([10, 100, 20], np.uint8)
    orange_upper = np.array([25, 255, 255], np.uint8)
    orange_mask = cv2.inRange(hsvFrame, orange_lower, orange_upper)

    # Morphological Transform, Dilation for each color and bitwise_and operator between frame and mask determines
    # to detect only particular colors
    kernal = np.ones((5, 5), "uint8")

    # For orange color
    orange_mask = cv2.dilate(orange_mask, kernal)
    res_orange = cv2.bitwise_and(frame, frame, mask=orange_mask)

    # Creating contour to track orange color
    contours, hierarchy = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)

        # This value can be changed as well
        if (area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 165, 255), 2)
            cv2.putText(frame, "Claw", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255))

            # find the centroid point and label it
            x2 = x + int(w / 2)
            y2 = y + int(h / 2)
            cv2.circle(frame, (x2, y2), 2, (0, 0, 0), -1)
            text = "x: " + str(x2) + ", y: " + str(y2)
            cv2.putText(frame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
            print(x2, y2)

    # Find the coordinates of rectangle which labels our object
    classIds, confs, bbox = net.detect(frame, confThreshold=0.45)
    # print(classIds, bbox)

    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            cv2.rectangle(frame, box, color=(0, 255, 0), thickness=2)
            cv2.putText(frame, classNames[classId - 1].upper(), (box[0] + 10, box[1] + 30),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

            # Shows the probability, which can be deleted
            cv2.putText(frame, str(round(confidence * 100, 2)), (box[0] + 200, box[1] + 30),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

            # center_X = round(box[0] + box[2] / 2)
            # center_Y = round(box[1] + box[3] / 2)
            # cv2.circle(img, (center_X, center_Y), 2, (0, 0, 255), -1)

    if cv2.waitKey(1) & 0xFF == ord('s'):
        # when the arm gets back to its original position, detect for once
        cv2.imwrite("capture.jpg", frame)
        capture = cv2.imread("capture.jpg")
        objectIds, objectConfs, objectBbox = net.detect(capture, confThreshold=0.5)
        print(objectIds, objectBbox)

        if len(classIds) != 0:
            # Select the first object for the arm to grab
            objectId = objectIds[0]
            objectBox = objectBbox[0]
            center_X.append(round(objectBox[0] + objectBox[2] / 2))
            center_Y.append(round(objectBox[1] + objectBox[3] / 2))
            print(classNames[objectId - 1])
            print(center_X[-1], center_Y[-1])
        else:
            print("Nothing is detected in the area.")

    if len(center_X) > 0:
        cv2.circle(frame, (center_X[-1], center_Y[-1]), 2, (0, 0, 255), -1)

    cv2.imshow("Output", frame)
    cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()