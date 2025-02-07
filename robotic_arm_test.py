from machine import Pin,PWM
import time
import utime

SERVO_PINS= {
    "base":4,
    "shoulder": 16,
    "elbow":21,
    "wrist":8,
    "gripper":9
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
    for angle in range(115, 19, -1):
        set_servo_angle("base", angle)
        time.sleep_ms(25)
    for angle in range(19, 115):
        set_servo_angle("shoulder", angle)
        time.sleep_ms(25)    