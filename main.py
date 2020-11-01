import cv2

thres = 0.45 # Threshold to detect object

cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)
cap.set(10, 70)

classNames = []
classFile = 'coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

while True:
    success, img = cap.read()
    img = cv2.flip(img, +1)

    # Find the coordinates of rectangle which labels our object
    classIds, confs, bbox = net.detect(img, confThreshold=thres)
    # print(classIds, bbox)

    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            # The specific categoris can be selected by classID numbers in 'coco.names'
            # if classId == 1:
            cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
            cv2.putText(img, classNames[classId-1].upper(), (box[0]+10, box[1]+30),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

            # Shows the probability, which can be deleted
            cv2.putText(img, str(round(confidence*100, 2)), (box[0]+200, box[1]+30),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

            center_X = round(box[0] + box[2] / 2)
            center_Y = round(box[1] + box[3] / 2)
            print(center_X, center_Y)
            cv2.circle(img, (center_X, center_Y), 2, (0, 0, 255), -1)

    cv2.imshow("Output", img)
    cv2.waitKey(1)
