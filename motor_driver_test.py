from machine import Pin, time_pulse_us, PWM
import time
import utime


IN1 = 17
IN2 = 12
IN3 = 13
IN4 = 14
ENA_PIN = 16
ENB_PIN = 15


# Motor pins
in1 = Pin(IN1, Pin.OUT)
in2 = Pin(IN2, Pin.OUT)
in3 = Pin(IN3, Pin.OUT)
in4 = Pin(IN4, Pin.OUT)

#PWM
enable1 = PWM(Pin(ENA_PIN))
enable2 = PWM(Pin(ENB_PIN))
enable1.freq(500) 
enable2.freq(500) 

def setspeed(left_speed, right_speed):
    enable1.duty_u16(left_speed)
    enable2.duty_u16(right_speed)

def forward():
    in1.high()  
    in2.low()   
    in3.high()  
    in4.low()   
    setspeed(30000, 30000)  
    print("in1", in1.value()) 
    print("in2", in2.value()) 
    print("ena", enable1.duty_u16()) #Prints the current duty cycle
    utime.sleep(1) # sleep for one second.

while True:
    forward()