# rc.py
import machine
import time

def run():
    led_pin = 2  # GPIO2 for onboard LED
    pwm = machine.PWM(machine.Pin(led_pin), freq=1000)

    try:
        while True:
            # Fade in
            for duty in range(0, 1024, 10):
                pwm.duty(duty)
                time.sleep(0.01)
            # Fade out
            for duty in range(1023, -1, -10):
                pwm.duty(duty)
                time.sleep(0.01)
    except KeyboardInterrupt:
        pwm.deinit()