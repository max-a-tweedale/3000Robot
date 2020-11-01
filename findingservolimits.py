import RPi.GPIO as GPIO
import pigpio
import time

servo = 18

pwm = pigpio.pi()
pwm.set_mode(servo,pigpio.OUTPUT)
pwm.set_PWM_frequency(servo,50)

ma
while True:
    pwm.set_servo_pulsewidth(servo,500)
    time.sleep(3)

    pwm.set_servo_pulsewidth(servo,2500)
    time.sleep(3)

