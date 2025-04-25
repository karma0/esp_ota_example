# main.py — Unified RC/ROV Controller (ESP32, MicroPython)
# Set ROLE to either 'RC' or 'ROV'

import network
import espnow
import time
import math
from machine import Pin, ADC, PWM

# ----------------------------------------
# CONFIGURATION
# ----------------------------------------
ROLE = 'RC'  # 'RC' or 'ROV'

# Common
LED_PIN         = 2         # GPIO2: Blue LED (active-low PWM)
FAST_FLASH_MS   = 100       # ms flash on action
CONN_FLASH_MS   = 500       # ms initial connect flash
FADE_PERIOD_MS  = 2000      # ms fade cycle period

# RC-specific
RC_ADC_X_PIN    = 34        # ADC1_CH6: X-axis pot
RC_ADC_Y_PIN    = 35        # ADC1_CH7: Y-axis pot
RC_MAC          = b"\x2C\xBC\xBB\x4B\xEB\xA4"  # RC MAC address
ROV_MAC         = b"\x2C\xBC\xBB\x4B\xF2\x34"  # ROV MAC address
SEND_INTERVAL   = 1000      # ms between sends/prints
AXIS_DEADZONE   = 5         # percent deadzone threshold
OFFSET_X        = -13       # calibration offset for X
OFFSET_Y        = -11       # calibration offset for Y

# ROV-specific
IN1_PIN         = 13        # H-bridge IN1
IN2_PIN         = 14        # H-bridge IN2
ENA_PIN         = 27        # PWM enable for motor
SERVO_PIN       = 26        # Steering servo signal
RECV_TIMEOUT    = 200       # ms timeout for recv

# ----------------------------------------
# NON-BLOCKING LED CONTROLLER (PWM)
def millis(): return time.ticks_ms()
class LEDController:
    def __init__(self, pwm_obj):
        self.pwm = pwm_obj
        self.mode = 'off'
        self.interval = 0
        self.last = millis()
        self.state = False
    def set(self, mode, interval):
        self.mode, self.interval = mode, interval
        self.last = millis()
        self.state = False
        if mode == 'flash':
            self.pwm.duty(1023)
        elif mode == 'fade':
            pass
        else:
            self.pwm.duty(1023)
    def update(self):
        now = millis()
        if self.mode == 'flash' and time.ticks_diff(now, self.last) >= self.interval:
            duty = 0 if self.state else 1023
            self.pwm.duty(duty)
            self.state = not self.state
            self.last = now
        elif self.mode == 'fade':
            t = time.ticks_diff(now, self.last) % self.interval
            duty = int((math.sin(2 * math.pi * t / self.interval) + 1) * 511.5)
            self.pwm.duty(duty)

# ----------------------------------------
# SETUP
led_pwm = PWM(Pin(LED_PIN), freq=1000)
led = LEDController(led_pwm)

# Wi-Fi & ESP-NOW init
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.disconnect()
esp = espnow.ESPNow()
esp.active(True)

# Print MAC
mac = sta.config('mac')
print(f"{ROLE} MAC:", ":".join(f"{b:02X}" for b in mac))

if ROLE == 'RC':
    # ADC setup
    adc_x = ADC(Pin(RC_ADC_X_PIN))
    adc_x.atten(ADC.ATTN_11DB)
    adc_y = ADC(Pin(RC_ADC_Y_PIN))
    adc_y.atten(ADC.ATTN_11DB)
    esp.add_peer(ROV_MAC)
    # Initial LED behavior
    led.set('flash', CONN_FLASH_MS)
    time.sleep_ms(500)
    led.set('fade', FADE_PERIOD_MS)
    # Main RC loop
    timer = millis()
    action_ts = 0
    while True:
        now = millis()
        if time.ticks_diff(now, timer) >= SEND_INTERVAL:
            # Read and calibrate
            raw_x = adc_x.read()
            raw_y = adc_y.read()
            raw_pct_x = int((raw_x / 4095 * 200) - 100)
            raw_pct_y = int((raw_y / 4095 * 200) - 100)
            # Apply offsets
            pct_x = raw_pct_x - OFFSET_X
            pct_y = raw_pct_y - OFFSET_Y
            # Clamp to -100..100
            pct_x = max(min(pct_x, 100), -100)
            pct_y = max(min(pct_y, 100), -100)
            # Deadzone
            x = 0 if abs(pct_x) < AXIS_DEADZONE else pct_x
            y = 0 if abs(pct_y) < AXIS_DEADZONE else pct_y
            # Send every interval
            msg = f"{x}:{y}"
            esp.send(ROV_MAC, msg)
            print("Sent ->", msg)
            # LED flash on transmit
            led.set('flash', FAST_FLASH_MS)
            action_ts = now
            timer = now
        elif led.mode == 'flash' and time.ticks_diff(now, action_ts) >= FAST_FLASH_MS:
            led.set('fade', FADE_PERIOD_MS)
        led.update()

elif ROLE == 'ROV':
    # Motor & servo setup
    in1 = Pin(IN1_PIN, Pin.OUT)
    in2 = Pin(IN2_PIN, Pin.OUT)
    # ENA controls speed via PWM; use a higher frequency for DC motor driver
    ena = PWM(Pin(ENA_PIN), freq=1000)
    servo = PWM(Pin(SERVO_PIN), freq=50)
    # Center servo and stop motor at boot
    center = int((0 + 100) * (115 - 40) / 200 + 40)
    servo.duty(center)
    in1.off()
    in2.off()
    ena.duty(0)
    # Initial LED behavior
    led.set('flash', CONN_FLASH_MS)
    time.sleep_ms(500)
    led.set('fade', FADE_PERIOD_MS)
    # Main ROV loop
    action_ts = 0
    while True:
        host, msg = esp.recv(RECV_TIMEOUT)
        now = millis()
        if msg:
            x_str, y_str = msg.decode().split(':')
            x, y = int(x_str), int(y_str)
            # Steering servo
            duty_s = int((x + 100) * (115 - 40) / 200 + 40)
            servo.duty(duty_s)
            # Motor control
            if y > 0:
                in1.on()
                in2.off()
                duty = int(abs(y) * 1023 / 100)
                ena.duty(duty)
            elif y < 0:
                in1.off()
                in2.on()
                duty = int(abs(y) * 1023 / 100)
                ena.duty(duty)
            else:
                # brake mode per L298: both inputs low; ENA low
                in1.off()
                in2.off()
                ena.duty(0)
            print("Rx <-", msg.decode())
            led.set('flash', FAST_FLASH_MS)
            action_ts = now
        elif led.mode == 'flash' and time.ticks_diff(now, action_ts) >= FAST_FLASH_MS:
            led.set('fade', FADE_PERIOD_MS)
        led.update()

else:
    raise ValueError("Invalid ROLE: must be 'RC' or 'ROV'")