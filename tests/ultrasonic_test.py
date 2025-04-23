import machine  
import time  
#########test

TRIG = machine.Pin(1, machine.Pin.OUT)  # TRIG pin set as output
ECHO = machine.Pin(2, machine.Pin.IN)  # ECHO pin set as input


def distance():
    # Function to calculate distance in centimeters
    TRIG.low()  # Set TRIG low
    time.sleep_us(2)  # Wait for 2 microseconds
    TRIG.high()  # Set TRIG high
    time.sleep_us(10)  # Wait for 10 microseconds
    TRIG.low()  # Set TRIG low again

    # Wait for ECHO pin to go high
    while not ECHO.value():
        pass

    time1 = time.ticks_us()  # Record time when ECHO goes high

    # Wait for ECHO pin to go low
    while ECHO.value():
        pass

    time2 = time.ticks_us()  # Record time when ECHO goes low

    # Calculate the duration of the ECHO pin being high
    during = time.ticks_diff(time2, time1)

    # Return the calculated distance (using speed of sound)
    return during * 340 / 2 / 10000  # Distance in centimeters


# Main loop
while True:
    during = distance()  # Get distance from sensor
    print("Distance: %.2f cm" % during)  # Print distance
    time.sleep_ms(300)  # Wait for 300 milliseconds before next measurement
