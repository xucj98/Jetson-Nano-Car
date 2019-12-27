import Jetson.GPIO as GPIO
from adafruit_servokit import ServoKit

import time
import threading
from enum import Enum

class SoftPwm(threading.Thread):
	
    def __init__(self, pin):
        threading.Thread.__init__(self)
        self.pin = pin
        self.duty_ratio = 0
        self.run_flag = False

    def set_duty_ratio(self, duty_ratio):
        assert duty_ratio >= 0 and duty_ratio <= 100, \
            'duty_ratio should be a number between 0 and 100.'
        self.duty_ratio = duty_ratio

    def run(self):
        self.run_flag = True
        while self.run_flag:
            GPIO.output(self.pin, GPIO.HIGH)
            time.sleep(0.0001 * self.duty_ratio)	
            GPIO.output(self.pin, GPIO.LOW)
            time.sleep(0.0001 * (100 - self.duty_ratio))

    def stop(self):
	    self.run_flag = False
	    self.join()
	    GPIO.output(self.pin, GPIO.LOW)

class Mode(Enum):

    FORWARD = 0
    BACKWARD = 1
    STOP = 2 

class Controller:

    def __init__(self): 

        # GPIO.BOARD mode
        # self.A_PWM_PIN = 12
        # self.A_IN1_PIN = 15
        # self.A_IN2_PIN = 13

        # self.B_PWM_PIN = 16
        # self.B_IN1_PIN = 22
        # self.B_IN2_PIN = 18

        # GPIO.TEGRA_SOC mode
        self.A_PWM_PIN = 'DAP4_SCLK'
        self.A_IN1_PIN = 'LCD_TE'
        self.A_IN2_PIN = 'SPI2_SCK'

        self.B_PWM_PIN = 'SPI2_CS1'
        self.B_IN1_PIN = 'SPI2_MISO'
        self.B_IN2_PIN = 'SPI2_CS0'

        # GPIO.setmode(GPIO.BOARD) 

        GPIO.setup([self.A_PWM_PIN, self.A_IN1_PIN, self.A_IN2_PIN], GPIO.OUT)
        GPIO.setup([self.B_PWM_PIN, self.B_IN1_PIN, self.B_IN2_PIN], GPIO.OUT)

        self.servo = ServoKit(channels=16).servo[1]
        
        self.speed = 0
        self.angle = 90
        self.mode = Mode.STOP        

        # protect the motor, prevent the motor from jamming due to the low duty cycle    
        self.MIN_SPEED = 20
        # protect the servo
        self.MIN_SERVO_ANGLE = 45
        self.MAX_SERVO_ANGLE = 160

        self.softpwm_a = SoftPwm(self.A_PWM_PIN)
        self.softpwm_b = SoftPwm(self.B_PWM_PIN)
    
    def __del__(self):
        self.set_motion(0, 90, Mode.STOP)
        if self.softpwm_a.run_flag:
            self.softpwm_a.stop()
        if self.softpwm_b.run_flag:
            self.softpwm_b.stop()
        GPIO.cleanup()    

    def set_motion(self, speed: int=None, angle: int=None, mode: Mode=None):
        '''
        speed: an integer between MIN_SPEED and 100.
        angle: an integer between MIN_SERVO_ANGLE and MAX_SERVO_ANGLE
        mode: FORWARD, BACKWARD OR STOP
        '''
        assert speed is None or speed == 0 or speed >= self.MIN_SPEED and speed <= 100, \
            'speed should be an integer between {} and 100 or 0.'.format(self.MIN_SPEED)
        assert angle is None or angle >= self.MIN_SERVO_ANGLE and angle <= self.MAX_SERVO_ANGLE, \
            'angle should be an integer between {} and {}.'.format(self.MIN_SERVO_ANGLE, self.MAX_SERVO_ANGLE)
        
        if speed is not None:
            self.speed = speed
        if angle is not None:        
            self.angle = angle
        if mode is not None:
        	self.mode = mode

        if self.mode == Mode.FORWARD:
            GPIO.output([self.A_IN1_PIN, self.B_IN1_PIN], GPIO.HIGH)
            GPIO.output([self.A_IN2_PIN, self.B_IN2_PIN], GPIO.LOW)
            if not self.softpwm_a.run_flag:
                self.softpwm_a.start()
            if not self.softpwm_b.run_flag:
                self.softpwm_b.start()
        elif self.mode == Mode.BACKWARD:
            GPIO.output([self.A_IN1_PIN, self.B_IN1_PIN], GPIO.LOW)
            GPIO.output([self.A_IN2_PIN, self.B_IN2_PIN], GPIO.HIGH)
            if not self.softpwm_a.run_flag:
                self.softpwm_a.start()
            if not self.softpwm_b.run_flag:
                self.softpwm_b.start()
        else:
            GPIO.output([self.A_IN1_PIN, self.B_IN1_PIN], GPIO.LOW)
            GPIO.output([self.A_IN2_PIN, self.B_IN2_PIN], GPIO.LOW)
            '''
            if self.softpwm_a.run_flag:
                self.softpwm_a.stop()
            if self.softpwm_b.run_flag:
                self.softpwm_b.stop()
            '''
        self.softpwm_a.set_duty_ratio(self.speed)
        self.softpwm_b.set_duty_ratio(self.speed)
        self.servo.angle = self.angle

