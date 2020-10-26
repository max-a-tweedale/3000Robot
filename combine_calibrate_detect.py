import numpy as np
import cv2

def undistortion(img, mtx, dist):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    # print('roi ', roi)
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # crop the image
    x, y, w, h = roi
    if roi != (0, 0, 0, 0):
        dst = dst[y:y + h, x:x + w]

    return dst


def calibrate():
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    Nx_cor = 9
    Ny_cor = 6

    objp = np.zeros((Nx_cor * Ny_cor, 3), np.float32)
    objp[:, :2] = np.mgrid[0:Nx_cor, 0:Ny_cor].T.reshape(-1, 2)
    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane.

    count = 0  #
    while (True):

        ret, frame = cap.read()
        frame = cv2.flip(frame, +1)

        if cv2.waitKey(1) & 0xFF == ord(' '):

            # Our operations on the frame come here
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(gray, (Nx_cor, Ny_cor), None)  # Find the corners
            # If found, add object points, image points
            if ret == True:
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # Refined corners information
                objpoints.append(objp)
                imgpoints.append(corners)
                cv2.drawChessboardCorners(frame, (Nx_cor, Ny_cor), corners, ret)  # Draw the found corners
                count += 1
                print('Calibration: ' + str(count) + '/20')

                if count > 20:
                    break

        # Display the resulting frame
        cv2.imshow('Now is calibration', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    global mtx, dist

    # mtx, camera internal parameters; dist, distortion; rvecs, tvecs, rotation matrix, translation matrix
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print(mtx, dist)

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    print("total error: ", mean_error / len(objpoints))
    np.savez('calibrate.npz', mtx=mtx, dist=dist[0:4])


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


if __name__ == '__main__':

    mtx = []
    dist = []

    try:
        npzfile = np.load('calibrate.npz')
        mtx = npzfile['mtx']
        dist = npzfile['dist']
    except IOError:
        calibrate()

    print('dist', dist[0:4])

    while (True):

        ret, frame = cap.read()
        frame = cv2.flip(frame, +1)
        frame = undistortion(frame, mtx, dist[0:4])
        # Display the resulting frame
        cv2.imshow('Now is detection', frame)

        # Detecting objects and give out centroid point
        # The key should be replaced by an input signal (when the arm at its original position)
        if cv2.waitKey(1) & 0xFF == ord('s'):
            # when the arm gets back to its original position, detect for once
            cv2.imwrite("capture.jpg", frame)
            capture = cv2.imread("capture.jpg")
            classIds, confs, bbox = net.detect(capture, confThreshold=0.5)
            print(classIds, bbox)

            # There are some objects are detected
            if len(classIds) != 0:
                # Select the first object for the arm to grab
                classId = classIds[0]
                box = bbox[0]
                center_X = round(box[0] + box[2] / 2)
                center_Y = round(box[1] + box[3] / 2)
                print(classNames[classId-1])
                print(center_X, center_Y)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    cap.release()
    cv2.destroyAllWindows()