import RPi.GPIO as GPIO
import time
servoPin = 12
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servoPin,GPIO.OUT)
pwm = GPIO.PWM(servoPin,100)
pwm.start(0)


# Loop for duty values from 2 to 12 (0 to 180 degrees)
while 1:
  
    pwm.ChangeDutyCycle(12)
    time.sleep(1)
    pwm.ChangeDutyCycle(24)
    time.sleep(1)
    pwm.ChangeDutyCycle(1)
    time.sleep(1)
