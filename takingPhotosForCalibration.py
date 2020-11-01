
import cv2 

key = cv2. waitKey(1)
webcam = cv2.VideoCapture(0)
shot = 0 
while True:

    check, frame = webcam.read()
    print(check) #prints true as long as the webcam is running
    print(frame) #prints matrix values of each framecd 
    cv2.imshow("Capturing", frame)
    key = cv2.waitKey(1)
    if key == ord('s'):
        shotname = str(shot)
        cv2.imwrite(filename='saved_img' + shotname + '.jpg', img=frame)
        shot += 1
        
            
        cv2.waitKey(1650)
        cv2.destroyAllWindows()
        print("Processing image...")
        img_ = cv2.imread('saved_img.jpg', cv2.IMREAD_ANYCOLOR)
            
        print("Image saved!")
        
        
    elif key == ord('q'):
        print("Turning off camera.")
        webcam.release()
        print("Camera off.")
        print("Program ended.")
        cv2.destroyAllWindows()
        break
q        
