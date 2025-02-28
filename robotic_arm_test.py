from machine import Pin,PWM
import time
import utime

SERVO_PINS= {
    "base":18,
    "shoulder": 19,
    "elbow":20,
    "gripper":21,
}

servo_pwms = {}

for servo_name, servo_pin in SERVO_PINS.items():
    pwm = PWM(Pin(servo_pin))
    pwm.freq(50)
    servo_pwms[servo_name] = pwm


def set_servo_angle(servo_name, angle):
    if servo_name not in servo_pwms:
        print(f"Error: Servo '{servo_name}' not found")
        return
    pwm = servo_pwms[servo_name]
    #calculate duty cycle for servo

    pulse_width_us = int((angle / 180) * 2000 + 1000)
    duty_cycle = int(pulse_width_us * (65535 / 20000))
    pwm.duty_u16(duty_cycle)
    
while True:
      
    for angle in range(19, 120):
        set_servo_angle("gripper", angle)
        print(f"{angle}")
        time.sleep_ms(25)  
        
    for angle in range(120, 19):
        set_servo_angle("gripper", angle)
        print(f"{angle}")
        time.sleep_ms(25)