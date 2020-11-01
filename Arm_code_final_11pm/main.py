# Imports
import time
import numpy as np
from imutils.video import VideoStream
from main_functions import easyRobot
import cv2 as cv
from Colour_classifier import findObjects

## definitions
arduino_port = "/dev/ttyUSB0"

def sort_Plastic(bot, target, colour):
    bot.approach_target(target)
    bot.closeEE()
    bot.go_to(bot.bin[colour])
    bot.openEE()
    time.sleep(2)
    bot.return_home()

if __name__ == "__main__":
    
    #define arm from our code
    bot = easyRobot(arduino_port)
    
    # Using webcam
    print("Webcam Turning on...")
    vs = VideoStream(src=0).start()
    time.sleep(1.0)
    print("Press q to quit")
    frame = vs.read()
    cv.imshow("Stream", frame)
    key = cv.waitKey(1) & 0xFF
    time
    col = [None, 0]
    while True:
        frame = vs.read()
        # if end of stream has been reached
        if frame is None:
            break
        largest_object = findObjects(frame)
        
        if largest_object is not None:
            colour = largest_object[5]
            print("Plastic found (", colour, ")")
            if col[0] is None or col[0] != colour:
                col = [colour,1]
            elif col[0] == colour:
                col[1] +=1
                if col[1] > 20:
                    col = [None,0]
                    print("\n Moving to target")
                    cv.imshow("Stream", frame)
                    cv.waitKey(1)
                    time.sleep(2)
                    target = [250,0,20]
                    sort_Plastic(bot, target, colour)
        
        # display frame
        cv.imshow("Stream", frame)
        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):  # quit
            break
    
    
    
    
# Close the serial port

