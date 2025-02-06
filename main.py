from machine import Pin, time_pulse_us, PWM
import time
import utime

# --- Constants & Configurations ---
# Motor Pin Definitions
RIGHT_MOTOR_1_PIN = 10
RIGHT_MOTOR_2_PIN = 11
LEFT_MOTOR_1_PIN = 12
LEFT_MOTOR_2_PIN = 15
ENA_PIN = 7
ENB_PIN = 6 

# Servos
#SERVO_PIN = 16 

#ultrasonic pins
TRIG_PIN = 14
ECHO_PIN = 17

# IR sensor Pins
IR1_PIN = 1
IR2_PIN = 2
IR3_PIN = 4
IR4_PIN = 5
IR5_PIN = 6

# PID Tuning 
KP = 0.2 
KD = 0.1
MAX_SPEED = 4500  

#global variables

pos1 = 115 #servo
#set_position = 200 
d = 0
last_p =0
sum_w = 0
sum_i = 0
error = 0
i=0
p = 0
sensor = [0, 0, 0, 0, 0]
current_position=0

# --- Pin Initializations ---

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
#ultrasonic
trig_pin = Pin(TRIG_PIN, Pin.OUT)
echo_pin = Pin(ECHO_PIN, Pin.IN)

# IR sensors
ir_pins = [Pin(IR1_PIN, Pin.IN), Pin(IR2_PIN, Pin.IN), Pin(IR3_PIN, Pin.IN), Pin(IR4_PIN, Pin.IN), Pin(IR5_PIN, Pin.IN)]
'''
# Servo 
servo = PWM(Pin(SERVO_PIN))
servo.freq(50)  # Set servo PWM frequency to 50Hz (standard)
initial servo position
def set_servo_angle(angle):
    pulse_width = int(2000 / 180 * angle + 500)
    servo.duty_u16(pulse_width * 3.268292682926829) #convert pulse width into correct duty cycle
'''
def setspeed(left_speed, right_speed):
    enable1.duty_u16(left_speed)
    enable2.duty_u16(right_speed)


# --- Function Definitions ---

def measure_distance():
    trig_pin.low()
    time.sleep_us(2)
    trig_pin.high()
    time.sleep_us(10)
    trig_pin.low()
    while not echo_pin.value():
        pass

    time1 = time.ticks_us()  

    
    while echo_pin.value():
        pass

    time2 = time.ticks_us()  

    
    distance = time.ticks_diff(time2, time1)

    return distance * 340 / 2 / 10000
    

def sensor_readings():
    return [pin.value() for pin in ir_pins]

def forward_cross(c):
    cross = 0
    while cross < c:
        forward()
        if all(reading == 1 for reading in sensor_readings()):
            cross+=1
            while all(reading == 1 for reading in sensor_readings()):
                forward()


def left_u_turn():
    while ir_pins[2].value() == 1:
        left_motor_1.off()
        left_motor_2.off()
        right_motor_1.off()
        right_motor_2.on()
        setspeed(150, 150)

    while ir_pins[2].value() == 0:
        left_motor_1.off()
        left_motor_2.off()
        right_motor_1.off()
        right_motor_2.on()
        setspeed(150, 150)
    stop()
    time.sleep_ms(500)


def right_u_turn():
    while ir_pins[2].value() == 1:
        left_motor_1.off()
        left_motor_2.on()
        right_motor_1.off()
        right_motor_2.off()
        setspeed(150, 150)
    while ir_pins[2].value() == 0:
        left_motor_1.off()
        left_motor_2.on()
        right_motor_1.off()
        right_motor_2.off()
        setspeed(150, 150)
    stop()
    time.sleep_ms(500)

def u_turn():
    while ir_pins[2].value() == 1:
        left_motor_1.off()
        left_motor_2.on()
        right_motor_1.on()
        right_motor_2.off()
        setspeed(150, 150)
    while ir_pins[2].value() == 0:
        left_motor_1.off()
        left_motor_2.on()
        right_motor_1.on()
        right_motor_2.off()
        setspeed(150, 150)

    stop()
    time.sleep_ms(500)

def turn_left():
    stop()
    time.sleep_ms(500)
    while ir_pins[2].value() == 0:
        left_motor_1.off()
        left_motor_2.off()
        right_motor_1.off()
        right_motor_2.on()
        setspeed(150, 150)

    stop()
    time.sleep_ms(500)


def forward():
    global sum_w, sum_i, p, d, last_p,current_position

    sum_w = 0
    sum_i = 0
    sensor=sensor_readings()

    for i in range(5):
       sum_w += (sensor[i] * i * 100)
       sum_i += sensor[i]
    if(sum_i != 0):
        current_position = sum_w / sum_i
    else :
      current_position =0

    p= 100 - current_position 
    d = p - last_p
    last_p=p
    error = (p*KP + d*KD)
    if error < 0:
        leftWheel = MAX_SPEED
        rightWheel = int(MAX_SPEED - error)
    else:
        rightWheel = MAX_SPEED
        leftWheel = int(MAX_SPEED+error)
    left_motor_1.off()
    left_motor_2.on()
    right_motor_1.off()
    right_motor_2.on()
    setspeed(leftWheel, rightWheel)
    


def stop():
    right_motor_1.off()
    right_motor_2.off()
    left_motor_1.off()
    left_motor_2.off()
    setspeed(0, 0)


# --- Main Loop ---

while True:
    distance = measure_distance()
   # set_servo_angle(pos1)
    T=True
    while not T:
        forward()
        T=all(reading == 1 for reading in sensor_readings())
    right_u_turn()
    forward_cross(3)
    right_u_turn()

    while distance > 13:
       forward()
       distance = measure_distance()

    stop()
    '''
    for pos1 in range(115, 19, -1):
        set_servo_angle(pos1)
        time.sleep_ms(25)
    time.sleep_ms(1000)
    for pos1 in range(20,116):
         set_servo_angle(pos1)
         time.sleep_ms(25)
         '''
    time.sleep_ms(1000)
    u_turn()
    forward_cross(2)
    left_u_turn()
    forward_cross(1)

    while distance > 13:
       forward()
       distance = measure_distance()
    stop()
    '''
    for pos1 in range(115, 19, -1):
        set_servo_angle(pos1)
        time.sleep_ms(25)
    time.sleep_ms(1000)
    for pos1 in range(20,116):
         set_servo_angle(pos1)
         time.sleep_ms(25)
         '''
    time.sleep_ms(1000)
    forward_cross(2)

    while distance > 13:
       forward()
       distance = measure_distance()
    stop()
    '''
    for pos1 in range(115, 19, -1):
        set_servo_angle(pos1)
        time.sleep_ms(25)
    time.sleep_ms(1000)
    for pos1 in range(20,116):
         set_servo_angle(pos1)
         time.sleep_ms(25)
    time.sleep_ms(1000)
    '''
    time.sleep_ms(10000)