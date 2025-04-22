# ===========================================
# main.py for RC (Transmitter)
# ===========================================

import network
import espnow
import time
from machine import Pin, ADC, PWM

# ------------- CONFIGURATION -------------
RC_ADC_X_PIN = 26        # D26: X-axis potentiometer
RC_ADC_Y_PIN = 27        # D27: Y-axis potentiometer
LED_PIN       = 2        # GPIO2: Blue LED
ROV_MAC       = b"\xAA\xBB\xCC\xDD\xEE\xFF"  # Replace with ROV MAC
SEND_INTERVAL = 1.0      # seconds between sends

# ------------- SETUP HARDWARE -------------
# LED as PWM for fade, digital for flash
led = PWM(Pin(LED_PIN), freq=1000)
led_digital = Pin(LED_PIN, Pin.OUT)

# ADC setup
adc_x = ADC(Pin(RC_ADC_X_PIN))
adc_x.atten(ADC.ATTN_11DB)
adc_y = ADC(Pin(RC_ADC_Y_PIN))
adc_y.atten(ADC.ATTN_11DB)

# ------------- UTILITY FUNCTIONS -------------
# Map raw ADC (0-4095) to -100..100

def read_percent(adc):
    raw = adc.read()                  # 0..4095
    return int((raw / 4095 * 200) - 100)

# Flash LED digital at given interval for count

def flash_led(interval, count=1):
    for _ in range(count):
        led_digital.high()
        time.sleep(interval)
        led_digital.low()
        time.sleep(interval)

# Fade LED in/out once over duration

def fade_cycle(duration=2.0):
    steps = 50
    delay = duration / (2 * steps)
    # fade up
    for duty in range(0, 1024, int(1024/steps)):
        led.duty(duty)
        time.sleep(delay)
    # fade down
    for duty in range(1023, -1, -int(1024/steps)):
        led.duty(duty)
        time.sleep(delay)

# ------------- WIFI & ESP-NOW INIT -------------
sta = network.WLAN(network.WLAN.STA_IF)
sta.active(True)
sta.disconnect()

# Print own MAC
print("RC MAC:", ':'.join('{:02X}'.format(b) for b in sta.config('mac')))

# Initialize ESP-NOW
esp = espnow.ESPNow()
esp.active(True)
esp.add_peer(ROV_MAC)

# ------------- MAIN LOOP -------------
last_print = time.ticks_ms()

# Flash while "connecting" (peer added instantly)
# We assume connection established immediately after add_peer()

# Single fade before main loop
fade_cycle()

while True:
    # Read joystick
    steer = read_percent(adc_x)
    throttle = read_percent(adc_y)
    msg = "{}:{}".format(steer, throttle)

    # Fast flash to indicate transmit
    flash_led(0.10, count=1)

    # Send message
    esp.send(ROV_MAC, msg)

    # Print every 1s
    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) >= SEND_INTERVAL * 1000:
        print("Sent ->", msg)
        last_print = now

    # Idle glow (fade) until next send
    fade_cycle(duration=SEND_INTERVAL)
