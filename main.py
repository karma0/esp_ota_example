# main.py
import time, machine

# Use the same LED (GPIO2)
led = machine.Pin(2, machine.Pin.OUT)

while True:
    led.on()
    time.sleep(0.25)
    led.off()
    time.sleep(0.25)
