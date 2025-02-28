from machine import Pin, time_pulse_us, PWM
import time
import utime

# Motor Pins
IN1, IN2, IN3, IN4 = 17, 12, 13, 14
ENA_PIN, ENB_PIN = 16, 15

# IR Sensor Pins
IR1_PIN, IR2_PIN, IR3_PIN, IR4_PIN, IR5_PIN = 11, 10, 9, 8, 7

# Ultrasonic Sensor Pins
TRIG_PIN, ECHO_PIN = 1, 2

# PID Tuning
KP = 25    # Increased KP for more aggressive corrections
KI = 0.0   # Still zero, but added anti-windup protection
KD = 0.5   # Small value to avoid rapid oscillations
BASE_SPEED = 500  # Adjusted base speed (scaled for PWM)
MAX_SPEED = 1500  # Motor maximum speed

# Global Variables
previous_error, integral = 0, 0
previous_time = time.time()

# Pin Initializations
ir_pins = [Pin(pin, Pin.IN) for pin in [IR1_PIN, IR2_PIN, IR3_PIN, IR4_PIN, IR5_PIN]]
in1, in2, in3, in4 = Pin(IN1, Pin.OUT), Pin(IN2, Pin.OUT), Pin(IN3, Pin.OUT), Pin(IN4, Pin.OUT)

enable1, enable2 = PWM(Pin(ENA_PIN)), PWM(Pin(ENB_PIN))
enable1.freq(1000)
enable2.freq(1000)

TRIG, ECHO = Pin(TRIG_PIN, Pin.OUT), Pin(ECHO_PIN, Pin.IN)

# Function to Read IR Sensors
def sensor_readings():
    return [pin.value() for pin in ir_pins]

# Function to Calculate Error Based on Sensor Readings
def calculate_error(sensor_values):
    WEIGHTS = [-2, -1, 0, 1, 2]
    return sum(weight * value for weight, value in zip(WEIGHTS, sensor_values))

# Function to Set Motor Speed
def set_speed(left_speed, right_speed):
    left_speed = max(0, min(left_speed, MAX_SPEED))
    right_speed = max(0, min(right_speed, MAX_SPEED))
    
    enable1.duty_u16(int(left_speed))
    enable2.duty_u16(int(right_speed))

# Function to Move Forward with PID Control
def forward():
    global previous_error, integral, previous_time
    
    sensor_values = sensor_readings()
    error = calculate_error(sensor_values)

    current_time = time.time()
    dt = max(current_time - previous_time, 0.01)  # Prevent divide by zero
    
    proportional = error
    integral += error * dt
    integral = max(-50, min(integral, 50))  # Anti-windup

    derivative = (error - previous_error) / dt

    correction = (KP * proportional) + (KI * integral) + (KD * derivative)

    left_speed = BASE_SPEED - correction
    right_speed = BASE_SPEED + correction

    in1.low()
    in2.high()
    in3.high()
    in4.low()

    set_speed(left_speed, right_speed)

    previous_error = error
    previous_time = current_time

# Function to Stop the Robot
def stop():
    in1.low()
    in2.low()
    in3.low()
    in4.low()
    set_speed(0, 0)

# Main Loop
try:
    while True:
        forward()
        utime.sleep(0.05)
except KeyboardInterrupt:
    stop()
    print("Program terminated.")
 