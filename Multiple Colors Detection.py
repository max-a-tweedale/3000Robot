import numpy as np 
import cv2 

cap = cv2.VideoCapture(0) 

while(True): 
    _, frame = cap.read() 
    frame = cv2.flip(frame, +1)

    # Convert the frame in BGR(RGB color space) to HSV(hue-saturation-value) 
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

    # The threhold values can be changed 
    # Set range for red color and define mask 
    red_lower = np.array([136, 87, 111], np.uint8) 
    red_upper = np.array([180, 255, 255], np.uint8) 
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 

    # Set range for green color and define mask 
    green_lower = np.array([25, 52, 72], np.uint8) 
    green_upper = np.array([102, 255, 255], np.uint8) 
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 

    # Set range for blue color and define mask 
    blue_lower = np.array([94, 80, 2], np.uint8) 
    blue_upper = np.array([120, 255, 255], np.uint8) 
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 
    
    # Morphological Transform, Dilation for each color and bitwise_and operator between frame and mask determines to detect only particular colors 
    kernal = np.ones((5, 5), "uint8") 
    
    # For red color 
    red_mask = cv2.dilate(red_mask, kernal) 
    res_red = cv2.bitwise_and(frame, frame, mask = red_mask) 
    
    # For green color 
    green_mask = cv2.dilate(green_mask, kernal) 
    res_green = cv2.bitwise_and(frame, frame, mask = green_mask) 
    
    # For blue color 
    blue_mask = cv2.dilate(blue_mask, kernal) 
    res_blue = cv2.bitwise_and(frame, frame, mask = blue_mask) 

    # Creating contour to track red color 
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        
        # This value can be changed as well
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour) 
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2) 
            cv2.putText(frame, "Red Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
            
            # find the centroid point and label it
            x2 = x + int(w / 2)
            y2 = y + int(h / 2)
            cv2.circle(frame, (x2, y2), 2, (0, 0, 255), -1)
            text = "x: " + str(x2) + ", y: " + str(y2)
            cv2.putText(frame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))


    # Creating contour to track green color 
    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour)
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour) 
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2) 
            cv2.putText(frame, "Green Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
            
            # find the centroid point and label it
            x2 = x + int(w / 2)
            y2 = y + int(h / 2)
            cv2.circle(frame, (x2, y2), 2, (0, 255, 0), -1)
            text = "x: " + str(x2) + ", y: " + str(y2)
            cv2.putText(frame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))


    # Creating contour to track blue color 
    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour)
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour) 
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2) 
            cv2.putText(frame, "Blue Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0))
            
            # find the centroid point and label it
            x2 = x + int(w / 2)
            y2 = y + int(h / 2)
            cv2.circle(frame, (x2, y2), 2, (255, 0, 0), -1)
            text = "x: " + str(x2) + ", y: " + str(y2)
            cv2.putText(frame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0))

            
    # Program Termination 
    cv2.imshow("Multiple Colors Detection", frame) 
    if cv2.waitKey(10) & 0xFF == ord('q'): 
        cap.release() 
        cv2.destroyAllWindows() 
        break
