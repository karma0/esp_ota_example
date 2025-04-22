from machine import Pin, PWM
import time

# Initialize digital output pins
d13 = Pin(13, Pin.OUT)
d14 = Pin(14, Pin.OUT)

# Initialize PWM on GPIO27 with a frequency of 1000 Hz
pwm_pin = PWM(Pin(27), freq=1000)

# Define duty cycle range for 10-bit resolution (0-1023)
duty_min = 0
duty_max = 1023
step = 10  # Adjust step for smoother or faster transitions

debug_interval = 1000

try:
    interval_counter = 0

    while True:
        # Set D13 HIGH and D14 LOW
        d13.value(1)
        d14.value(0)

        # Fade in
        for duty in range(duty_min, duty_max + 1, step):
            pwm_pin.duty(duty)
            time.sleep(0.01)  # Adjust delay for desired speed

        # Fade out
        for duty in range(duty_max, duty_min - 1, -step):
            pwm_pin.duty(duty)
            time.sleep(0.01)

        # Swap D13 and D14 states: D13 LOW, D14 HIGH
        d13.value(0)
        d14.value(1)

        # Fade in
        for duty in range(duty_min, duty_max + 1, step):
            pwm_pin.duty(duty)
            time.sleep(0.01)

        # Fade out
        for duty in range(duty_max, duty_min - 1, -step):
            pwm_pin.duty(duty)
            time.sleep(0.01)
        
        # DEBUG OUTPUT
        if interval_counter % debug_interval == 0:
            print(f"Interval: {interval_counter}")
        interval_counter += 1

except KeyboardInterrupt:
    # Clean up on interrupt
    pwm_pin.deinit()
    d13.value(0)
    d14.value(0)
