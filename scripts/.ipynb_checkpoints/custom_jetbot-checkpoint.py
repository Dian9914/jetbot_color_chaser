#!/usr/bin/env python3
import time
from Adafruit_MotorHAT import Adafruit_MotorHAT

class Motor():
    def __init__(self, driver, channel, alpha, beta, gamma):
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        
        self._driver = driver
        self._motor = self._driver.getMotor(channel)
        if(channel == 1):
            self._ina = 1
            self._inb = 0
        else:
            self._ina = 2
            self._inb = 3
        
    def calibrate(self, alpha, beta, gamma):
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma

    def write_speed(self, value):
        """Sets motor value between [-1, 1]"""
        """ primero comprobamos que el valor sea mayor que el coef de seguridad gamma.
        Si es menor, la velocidad sera 0.
        Si es mayor, la velocidad se calculara segun los valores de la calibraci0n"""
        if abs(value) < self.gamma:
            mapped_value = 0
        elif value > 0:
            mapped_value = int(255.0 * (self.alpha * value + self.beta))
        else:
            mapped_value = int(255.0 * (self.alpha * value - self.beta))
        
        speed = min(max(mapped_value, 0), 255)
        self._motor.setSpeed(speed)
        if mapped_value < 0:
            self._motor.run(Adafruit_MotorHAT.FORWARD)
            self._driver._pwm.setPWM(self._ina,0,0)
            self._driver._pwm.setPWM(self._inb,0,speed*16)
        else:
            self._motor.run(Adafruit_MotorHAT.BACKWARD)
            self._driver._pwm.setPWM(self._ina,0,speed*16)
            self._driver._pwm.setPWM(self._inb,0,0)

    def release(self):
        """Stops motor by releasing control"""
        self._motor.run(Adafruit_MotorHAT.RELEASE)
        self._driver._pwm.setPWM(self._ina,0,0)
        self._driver._pwm.setPWM(self._inb,0,0)


class Robot():   
    def __init__(self):
        self.motor_driver = Adafruit_MotorHAT(i2c_bus=1)
        self.left_motor = Motor(self.motor_driver, channel=1, alpha=0.76, beta=0.25, gamma=0.02)
        self.right_motor = Motor(self.motor_driver, channel=1, alpha=0.75, beta=0.26, gamma=0.02)
        
    def set_motors(self, left_speed, right_speed):
        self.left_motor.write_speed(left_speed)
        self.right_motor.write_speed(right_speed)
        
    def forward(self, speed=1.0, duration=None):
        self.left_motor.write_speed(speed)
        self.right_motor.write_speed(speed)

    def backward(self, speed=1.0):
        self.left_motor.write_speed(-speed)
        self.right_motor.write_speed(-speed)

    def left(self, speed=1.0):
        self.left_motor.write_speed(-speed)
        self.right_motor.write_speed(speed)

    def right(self, speed=1.0):
        self.left_motor.write_speed(speed)
        self.right_motor.write_speed(-speed)

    def stop(self):
        self.left_motor.write_speed(0)
        self.right_motor.write_speed(0)
        
    def calibrate_left(self, alpha, beta, gamma):
        self.left_motor.calibrate(alpha, beta, gamma)
        
    def calibrate_right(self, alpha, beta, gamma):
        self.right_motor.calibrate(alpha, beta, gamma)