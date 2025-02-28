# main.py
import time
import machine
from secrets import DEVICE_ROLE

# Set up LED on GPIO2 with PWM for smooth glow effect
led_pin = machine.Pin(2, machine.Pin.OUT)
led_pwm = machine.PWM(led_pin)
led_pwm.freq(1000)  # Set PWM frequency to 1kHz

def glow_led(duration=2, steps=50):
    # Gradually increase brightness
    for duty in range(0, 1024, int(1024 / steps)):
        led_pwm.duty(duty)
        time.sleep(duration / (2 * steps))
    # Gradually decrease brightness
    for duty in range(1023, -1, -int(1024 / steps)):
        led_pwm.duty(duty)
        time.sleep(duration / (2 * steps))

# Indicate startup with LED glow
glow_led()

# Delegate to the appropriate script based on DEVICE_ROLE
if DEVICE_ROLE == 'RC':
    import rc
    rc.main()
elif DEVICE_ROLE == 'ROV':
    import rov
    rov.main()
else:
    print("Invalid DEVICE_ROLE specified in secrets.py. Please set to 'RC' or 'ROV'.")

