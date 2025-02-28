from machine import Pin, time_pulse_us, PWM
import time
import utime
import machine

# Motor Pins
IN1 = 17
IN2 = 12
IN3 = 13
IN4 = 14
ENA_PIN = 16
ENB_PIN = 15

# IR sensor Pins
IR1_PIN = 11
IR2_PIN = 10
IR3_PIN = 9
IR4_PIN = 8
IR5_PIN = 7

# Ultrasonic Sensor Pins
TRIG_PIN = 1
ECHO_PIN = 2


# PID Tuning
KP = 20
KI = 0.0
KD = 0.0
BASE_SPEED = 100  # Tune this value (0-100)
MAX_SPEED = 1500 # Reduced MAX_SPEED dramatically

# Global Variables
previous_error = 0
integral = 0
previous_time = time.time()
t_junction_count = 0
previous_error = 0
integral = 0
previous_time = time.time()

# Pin Definitions and Initializations
ir_pins = [Pin(IR1_PIN, Pin.IN), Pin(IR2_PIN, Pin.IN), Pin(IR3_PIN, Pin.IN), Pin(IR4_PIN, Pin.IN), Pin(IR5_PIN, Pin.IN)]
in1 = Pin(IN1, Pin.OUT)
in2 = Pin(IN2, Pin.OUT)
in3 = Pin(IN3, Pin.OUT)
in4 = Pin(IN4, Pin.OUT)

enable1 = PWM(Pin(ENA_PIN))
enable2 = PWM(Pin(ENB_PIN))
enable1.freq(500)
enable2.freq(500)

TRIG = Pin(TRIG_PIN, Pin.OUT)  # TRIG pin set as output
ECHO = Pin(ECHO_PIN, Pin.IN)   # ECHO pin set as input

LOG_FILE = "robot_log.txt"  # Define the log file name

# Ultrasonic Distance Function
def distance():
    """
    Measures the distance using the ultrasonic sensor.

    Returns:
        float: Distance in centimeters.  Returns -1 if a timeout occurs.
    """
    TRIG.low()
    utime.sleep_us(2)
    TRIG.high()
    utime.sleep_us(10)
    TRIG.low()

    # Wait for ECHO pin to go high with a timeout
    timeout_start = utime.ticks_us()
    while not ECHO.value():
        if utime.ticks_diff(utime.ticks_us(), timeout_start) > 100000:  # 100ms timeout
            log_message = "Ultrasonic Sensor Timeout (ECHO High)"
            print(log_message)
            log_to_file(log_message)
            return -1  # Indicate timeout

    echo_start = utime.ticks_us()

    # Wait for ECHO pin to go low with a timeout
    timeout_start = utime.ticks_us()
    while ECHO.value():
        if utime.ticks_diff(utime.ticks_us(), timeout_start) > 100000:  # 100ms timeout
            log_message = "Ultrasonic Sensor Timeout (ECHO Low)"
            print(log_message)
            log_to_file(log_message)
            return -1  # Indicate timeout

    echo_end = utime.ticks_us()

    pulse_duration = utime.ticks_diff(echo_end, echo_start)

    distance_cm = (pulse_duration * 0.0343) / 2  # Distance in cm/us

    return distance_cm

# IR Sensor Reading Function
def sensor_readings():
    """
    Reads the values from the IR sensors.

    Returns:
        list: A list of sensor values (0 or 1) from IR1 to IR5.
    """
    return [pin.value() for pin in ir_pins] 


# Motor Speed Control Function
def setspeed(left_speed, right_speed):
    """
    Sets the speed of the left and right motors using PWM.

    Args:
        left_speed (int): PWM duty cycle for the left motor (0-65535).
        right_speed (int): PWM duty cycle for the right motor (0-65535).
    """
    enable1.duty_u16(int(left_speed*(65535/100))) #Scaling speeds that goes from 0 to 100
    enable2.duty_u16(int(right_speed*(65535/100))) #Scaling speeds that goes from 0 to 100


# Forward Movement (Crossings Counter) - UNUSED in current implementation
def forward_cross(c):
    """
    Moves forward, counting the number of times all IR sensors detect a line (crossing).

    Args:
        c (int): The number of crossings to count.
    """
    cross = 0
    while cross < c:
        forward()
        if all(reading == 1 for reading in sensor_readings()):
            cross += 1
            while all(reading == 1 for reading in sensor_readings()):
                forward()
def calculate_error(sensor_values):
    # Assign weights to sensors: [leftmost, left, center, right, rightmost]
    WEIGHTS = [-2, -1, 0, 1, 2]
    error = sum(weight * value for weight, value in zip(WEIGHTS, sensor_values))
    return error

# Forward Movement with PID Control
def forward():
    """
    Moves the robot forward while attempting to stay on the line using PID control.
    """
    global previous_error, integral, previous_time

    sensor_values = sensor_readings()
    error = calculate_error(sensor_values)
    log_message = f"sensor:{sensor_values},error: {error}"
    print(log_message)
    log_to_file(log_message)

    current_time = time.time()
    dt = current_time - previous_time

    proportional = error
    integral += error * dt
    derivative = (error - previous_error) / dt if dt > 0 else 0

    correction = (KP * proportional) + (KI * integral) + (KD * derivative)

    left_speed = BASE_SPEED - correction
    right_speed = BASE_SPEED + correction

    # Clamp the speeds between 0 and 100
    left_speed = max(min(left_speed, 100), 0)
    right_speed = max(min(right_speed, 100), 0)

    log_message = f"Correction: {correction}, Left speed: {left_speed}, Right speed: {right_speed}"
    print(log_message)
    log_to_file(log_message)

    # Motor direction control
    in1.low()
    in2.high()
    in3.high()
    in4.low()

    # Set motor speeds
    setspeed(left_speed, right_speed)

    previous_error = error
    previous_time = current_time


# Stop Function
def stop():
    """
    Stops the robot.
    """
    in1.low()
    in2.low()
    in3.low()
    in4.low()
    setspeed(0, 0)

def log_to_file(message):
    """
    Appends a message to the log file with a timestamp.
    """
    timestamp = utime.time()
    log_message = f"{timestamp}: {message}\n"
    try:
        with open(LOG_FILE, "a") as f:
            f.write(log_message)
    except OSError as e:
        print(f"Error writing to log file: {e}")

# Main Loop
try:
    with open(LOG_FILE, "w") as f: #Clear previous data
        f.write("")  # Clear the log file at the start of each run
except OSError as e:
    print(f"Error opening/creating log file: {e}")

try:
    while True:
        dist = distance()
        log_message = "Distance: %.2f cm" % dist
        print(log_message)
        log_to_file(log_message)

        if dist == -1:
            log_message = "Error reading distance, stopping"
            print(log_message)
            log_to_file(log_message)
            stop()
            utime.sleep(0.1)
            continue

        if dist > 10:
            forward()
        else:
            stop()
            log_message = "Object detected, stopping."
            print(log_message)
            log_to_file(log_message)
        utime.sleep(0.05)
except KeyboardInterrupt:
    print("Exiting program")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
