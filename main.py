import uasyncio
import utime
from machine import Pin, PWM, time_pulse_us
import _thread



# Motor Pins
IN1, IN2, IN3, IN4 = 20, 19, 18, 17
ENA_PIN, ENB_PIN = 22, 16

# Ultrasonic Sensor Pins
TRIG_PIN = 9
ECHO_PIN = 8

# IR sensor pins (5-sensor array for line tracking)
IR1_PIN, IR2_PIN, IR3_PIN, IR4_PIN, IR5_PIN = 14, 13, 12, 11, 10

# Side sensors for junction detection
IR6_PIN = 7
IR7_PIN = 15

# Setup motor control pins
in1, in2, in3, in4 = [Pin(p, Pin.OUT) for p in [IN1, IN2, IN3, IN4]]
enable1, enable2 = PWM(Pin(ENA_PIN)), PWM(Pin(ENB_PIN))
enable1.freq(1000)
enable2.freq(1000)

# Setup IR sensors
ir_pins = [Pin(pin, Pin.IN) for pin in [IR1_PIN, IR2_PIN, IR3_PIN, IR4_PIN, IR5_PIN]]
side_ir_pins = [Pin(pin, Pin.IN) for pin in [IR6_PIN, IR7_PIN]]

# Setup ultrasonic sensor pins
TRIG = Pin(TRIG_PIN, Pin.OUT)
ECHO = Pin(ECHO_PIN, Pin.IN)


BASE_SPEED = 50000
MAX_SPEED = 64555
MIN_SPEED = 20000
SPEED = 50000

# PID constants for line tracking
Kp = 270.0
Kd = 100
last_error = 0
set_position = 0

# Global flags for events
junction_detected = False
cross_count = 0
action_in_progress = False


def set_speed(left_speed, right_speed):
    left_speed = max(MIN_SPEED, min(int(left_speed), MAX_SPEED))
    right_speed = max(MIN_SPEED, min(int(right_speed), MAX_SPEED))
    enable1.duty_u16(left_speed)
    enable2.duty_u16(right_speed)

def stop():
    in1.low()
    in2.low()
    in3.low()
    in4.low()
    set_speed(0, 0)

def distance():
    TRIG.low()
    utime.sleep_us(2)
    TRIG.high()
    utime.sleep_us(10)
    TRIG.low()
    try:
        pulse_time = time_pulse_us(ECHO, 1, 30000)
    except Exception as e:
        return 999
    if pulse_time <= 0:
        return 999
    return (pulse_time * 0.0343) / 2

def ultrasonic():
    global dist
    while True:
        dist = distance()
        utime.sleep(0.05)
    return dist

# Run ultrasonic in its own thread
_thread.start_new_thread(ultrasonic, ())

--------------------------

async def line_follow():
   
    global last_error, action_in_progress
    while True:
        if not action_in_progress:
            sensor_values = [p.value() for p in ir_pins]
            sum_w = 0
            sum_i = 0
            for i, val in enumerate(sensor_values):
                weight = (i - 2) * 100  # Weights: -200, -100, 0, 100, 200
                sum_w += val * weight
                sum_i += val
            position = sum_w / sum_i if sum_i > 0 else set_position
            error = set_position - position
            p_term = error * Kp
            d_term = (error - last_error) * Kd
            last_error = error
            correction = p_term + d_term
            left_speed = SPEED - correction
            right_speed = SPEED + correction

            # Drive forward
            in1.low()
            in2.high()
            in3.high()
            in4.low()
            set_speed(left_speed, right_speed)
        await uasyncio.sleep_ms(10)

async def detect_junctions():
   
    global junction_detected, action_in_progress
    prev_junction = False
    while True:
        j0 = side_ir_pins[0].value()
        j1 = side_ir_pins[1].value()
        current_junction = (j0 == 1 and j1 == 0)
        print("Junction sensors:", j0, j1, "Current:", current_junction, "Prev:", prev_junction)
        # Trigger only on rising edge
        if current_junction and not prev_junction:
            print("Junction detected! Initiating right turn.")
            action_in_progress = True
            stop()
            
            await right_turn()
            junction_detected = True
            action_in_progress = False
        prev_junction = current_junction
        await uasyncio.sleep_ms(50)

async def forward_cross(target_cross):
   
    global cross_count, action_in_progress
    while cross_count < target_cross:
        sensor_vals = [p.value() == 1 for p in ir_pins]
        print("Cross sensors:", sensor_vals)
        if all(sensor_vals):  # All sensors HIGH: a cross is detected.
            action_in_progress = True
            stop()
            print("Cross detected, stopping for 2 seconds.")
            await uasyncio.sleep(2)  # Stop for 2 seconds at a cross
            cross_count += 1
            print("Forward Cross count:", cross_count)
            action_in_progress = False
            # Debounce: wait until cross condition is cleared
            while all(p.value() for p in ir_pins):
                await uasyncio.sleep_ms(50)
        await uasyncio.sleep_ms(20)

async def right_turn():
  
    print("Starting sharp right turn pivot...")

    # Ensure the middle sensor is initially on white.
    # (Adjust the condition if your sensor logic is reversed.)
    while ir_pins[2].value() == 1:
        await uasyncio.sleep_ms(10)

    # Initiate the pivot:
    # - Reverse the left motor (in1 HIGH, in2 LOW)
    # - Drive the right motor forward (in3 HIGH, in4 LOW)
    in1.low()
    in2.high()
    in3.low()
    in4.low()
    # Set both speeds for a balanced pivot (adjust if needed).
    enable1.duty_u16(70000)
    enable2.duty_u16(0)

    # Wait until the middle sensor (IR3) leaves the white line.
    while ir_pins[2].value() == 0:
        await uasyncio.sleep_ms(10)
    # Then wait until the middle sensor detects the white line again.
    while ir_pins[2].value() == 1:
        await uasyncio.sleep_ms(10)

    print("Sharp right turn pivot completed, white line detected by center sensor.")
    stop()
    await uasyncio.sleep_ms(200)

async def obstacle_detection():
   
    global action_in_progress
    while True:
        if distance() < 30:
            print("Obstacle detected. Stopping and turning right.")
            action_in_progress = True
            stop()
            await right_turn()
            action_in_progress = False
        await uasyncio.sleep_ms(100)

async def main():
    
    uasyncio.create_task(line_follow())
    uasyncio.create_task(detect_junctions())
    uasyncio.create_task(forward_cross(2))  # Adjust target cross count as needed
    uasyncio.create_task(obstacle_detection())

    while True:
        await uasyncio.sleep_ms(100)

uasyncio.run(main())

