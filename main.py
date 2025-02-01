from machine import Pin, time_pulse_us, PWM
import time
import utime
# Pin configuration
trig_pin = Pin(14, Pin.OUT)
echo_pin = Pin(17, Pin.IN)


# ir sensor pins
s1 = Pin(1, Pin.IN)
s2 = Pin(2, Pin.IN)
s3 = Pin(4, Pin.IN)
s4 = Pin(5, Pin.IN)
s5 = Pin(6, Pin.IN)



# Motor control pins
right_motor_1 = Pin(10, Pin.OUT)
right_motor_2 = Pin(11, Pin.OUT)
left_motor_1 = Pin(12, Pin.OUT)
left_motor_2 = Pin(15, Pin.OUT)

# PWM motor speed control
speed = 4500
enable1 = PWM(Pin(6))
enable2 = PWM(Pin(7))

def sensor_readings():
    time.sleep_us(0.05)
    return ([s1.value(), s2.value(), s3.value(), s4.value(), s5.value()])

def white_line():
    sensorValues = sensor_readings()
    if sensorValues == [1,1, 1,1,1]:
        print("f{sensorValues} line data")
    elif sensorValues == [0,0,0,0,0]:
        print("f{sensorValues} line data")
    elif sensorValues == [0, 0, 1, 1, 1]:  
        print("f{sensorValues} line data")

def measure_distance():
    trig_pin.low()
    time.sleep_us(2)
    trig_pin.high()
    time.sleep_us(5)
    trig_pin.low()
   
    pulse_duration = time_pulse_us(echo_pin, 1) # Wait for HIGH, returns time in microseconds
    distance = (pulse_duration * 0.0343) / 2 # Speed of sound in cm/us, then convert us to cm

    return distance

def setspeed():
    enable1.duty_u16(speed)
    enable2.duty_u16(speed)

def move_forward():
    right_motor_1.on()
    left_motor_1.off()
    right_motor_2.on()
    left_motor_2.off()


def stop():
    right_motor_1.off()
    left_motor_1.off()
    right_motor_2.off()
    left_motor_2.off()

def right_turn():


def left_turn():

def u_turn():

while True:
    white_line()
    distance = measure_distance()
    print("The distance from object is ", distance, "cm")
    utime.sleep(0.2) 