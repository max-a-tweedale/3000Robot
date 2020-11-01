import numpy as np 
import cv2 


def findObjects(frame, min_area=700):
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # convert to hsv
    big_cont = None
    # Set range for color and define mask 
    red_lower = np.array([136, 87, 111], np.uint8) 
    red_upper = np.array([180, 255, 255], np.uint8) 
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    
    green_lower = np.array([25, 52, 72], np.uint8) 
    green_upper = np.array([102, 255, 255], np.uint8) 
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
    
    blue_lower = np.array([110, 50, 50], np.uint8) 
    blue_upper = np.array([130, 255, 255], np.uint8) 
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 
    
    kernal = np.ones((5, 5), "uint8") 
    
    red_mask = cv2.dilate(red_mask, kernal) 
    res_red = cv2.bitwise_and(frame, frame, mask = red_mask) 

    green_mask = cv2.dilate(green_mask, kernal) 
    res_green = cv2.bitwise_and(frame, frame, mask = green_mask) 
    
    blue_mask = cv2.dilate(blue_mask, kernal) 
    res_blue = cv2.bitwise_and(frame, frame, mask = blue_mask) 

    # Creating contour to track red color 
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    red_cont = None
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        
        if red_cont is None or red_cont[4] < area:
            x, y, w, h = cv2.boundingRect(contour)
            red_cont = [x, y, w, h, area, 'red']
            
                
            
    if red_cont is not None:
        x, y, w, h, area, _ = red_cont
        if area > min_area:
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            big_cont = red_cont
        else:
            red_cont = None

    # Creating contour to track green color 
    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    green_cont = None
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour)
        
        if green_cont is None or green_cont[4] < area:
            x, y, w, h = cv2.boundingRect(contour)
            green_cont = [x, y, w, h, area, 'green']
            
    if green_cont is not None:
        x, y, w, h, area, _ = green_cont
        if area > min_area :
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            if big_cont is None:
                big_cont = green_cont
            elif big_cont[4] < area:
                big_cont = green_cont
        else:
            green_cont = None

    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    blue_cont = None
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour)
        if blue_cont is None or blue_cont[4] < area:
            x, y, w, h = cv2.boundingRect(contour)
            blue_cont = [x, y, w, h, area, 'blue']
            
    if blue_cont is not None:
        x, y, w, h, area, _ = blue_cont
        if area > min_area:
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            if big_cont is None:
                big_cont = blue_cont
            elif big_cont[4] < area:
                big_cont = blue_cont
        else:
            blue_cont = None
    
    
    return big_cont
if __name__=="__main__":
    cap = cv2.VideoCapture(0) 

    while(True): 
        _, frame = cap.read() 
        frame = cv2.flip(frame, +1)
        
        findObjects(frame)
                
        # Program Termination 
        cv2.imshow("Multiple Colors Detection", frame) 
        if cv2.waitKey(10) & 0xFF == ord('q'): 
            cap.release() 
            cv2.destroyAllWindows() 
            break
