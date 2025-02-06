from machine import Pin, time_pulse_us, PWM
import time
import utime


RIGHT_MOTOR_1_PIN = 10
RIGHT_MOTOR_2_PIN = 11
LEFT_MOTOR_1_PIN = 12
LEFT_MOTOR_2_PIN = 15
ENA_PIN = 7
ENB_PIN = 6 


# Motor pins
right_motor_1 = Pin(RIGHT_MOTOR_1_PIN, Pin.OUT)
right_motor_2 = Pin(RIGHT_MOTOR_2_PIN, Pin.OUT)
left_motor_1 = Pin(LEFT_MOTOR_1_PIN, Pin.OUT)
left_motor_2 = Pin(LEFT_MOTOR_2_PIN, Pin.OUT)

#PWM
enable1 = PWM(Pin(ENA_PIN))
enable2 = PWM(Pin(ENB_PIN))
enable1.freq(500) 
enable2.freq(500) 

def setspeed(left_speed, right_speed):
    enable1.duty_u16(left_speed)
    enable2.duty_u16(right_speed)

def forward():
    left_motor_1.off()
    left_motor_2.on()
    right_motor_1.off()
    right_motor_2.on()


while True:
    forward()