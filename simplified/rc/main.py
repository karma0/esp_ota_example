# main.py — Unified RC/ROV Controller with Diagnostics (ESP32, MicroPython)
# Single file combining RC, ROV, and Diagnostics modes

import network
import espnow
import time
import math
from machine import Pin, ADC, PWM, I2C

# ----------------------------------------
# CONFIGURATION
# ----------------------------------------
MODE = 'RC'         # 'RC', 'ROV', or 'DIAG'

# Common LED settings
LED_PIN         = 2  # GPIO2: Blue LED (active-low PWM)
FAST_FLASH_MS   = 100
CONN_FLASH_MS   = 500
FADE_PERIOD_MS  = 2000

# RC parameters
RC_ADC_X_PIN    = 34
RC_ADC_Y_PIN    = 35
RC_MAC_STR      = "2C:BC:BB:4B:EB:A4"
ROV_MAC_STR     = "38:18:2B:2F:11:58"
RC_MAC          = bytes(int(x,16) for x in RC_MAC_STR.split(':'))
ROV_MAC         = bytes(int(x,16) for x in ROV_MAC_STR.split(':'))
SEND_INTERVAL   = 1000
AXIS_DEADZONE   = 5
OFFSET_X        = -13
OFFSET_Y        = -11

# ROV parameters
I2C_SCL_PIN     = 22
I2C_SDA_PIN     = 21
PCA_I2C_FREQ    = 400000
PCA_ADDR        = 0x40
SERVO_CH        = 0
ENA_CH          = 1
IN1_CH          = 2
IN2_CH          = 3
RECV_TIMEOUT    = 200

# Diagnostics channels (first 4 PCA9685 channels)
DIAG_CHANNELS   = range(4)

# ----------------------------------------
# PCA9685 DRIVER
# ----------------------------------------
class PCA9685:
    _MODE1      = 0x00
    _PRESCALE   = 0xFE
    _LED0_ON_L  = 0x06
    def __init__(self, i2c, address=PCA_ADDR):
        self.i2c = i2c; self.address = address
        # enable AI, clear SLEEP
        self.i2c.writeto_mem(address, self._MODE1, b'\x20')
    def freq(self, hz):
        prescale = int((25_000_000/4096/hz)-1 + 0.5)
        old = self.i2c.readfrom_mem(self.address,self._MODE1,1)[0]
        self.i2c.writeto_mem(self.address,self._MODE1,bytes([(old&0x7F)|0x10]))
        self.i2c.writeto_mem(self.address,self._PRESCALE,bytes([prescale]))
        self.i2c.writeto_mem(self.address,self._MODE1,bytes([old])); time.sleep_ms(5)
        self.i2c.writeto_mem(self.address,self._MODE1,bytes([old|0x80]))
    def duty(self, ch, val):
        val = max(0,min(4095,val))
        base = self._LED0_ON_L+4*ch
        self.i2c.writeto_mem(self.address,base,bytes([0,0,val&0xFF,(val>>8)&0xFF]))

# ----------------------------------------
# LED CONTROLLER
# ----------------------------------------
def millis(): return time.ticks_ms()
class LEDCtrl:
    def __init__(self,pwm): self.pwm=pwm; self.mode='off'; self.intv=0; self.last=millis(); self.state=False
    def set(self,m,i): self.mode=m; self.intv=i; self.last=millis(); self.state=False; \
        (self.pwm.duty(1023) if m=='flash' else None)
    def update(self):
        now=millis()
        if self.mode=='flash' and time.ticks_diff(now,self.last)>=self.intv:
            self.pwm.duty(0 if self.state else 1023); self.state=not self.state; self.last=now
        elif self.mode=='fade':
            t=time.ticks_diff(now,self.last)%self.intv
            d=int((math.sin(2*math.pi*t/self.intv)+1)*511.5)
            self.pwm.duty(d)

# ----------------------------------------
# SETUP COMMON
# ----------------------------------------
led = LEDCtrl(PWM(Pin(LED_PIN),freq=1000))
sta=network.WLAN(network.STA_IF); sta.active(True); sta.disconnect()
esp=espnow.ESPNow(); esp.active(True)
mac=sta.config('mac'); print(f"{MODE} MAC:",":".join(f"{b:02X}" for b in mac))

# ----------------------------------------
# RC MODE
# ----------------------------------------
if MODE=='RC':
    adc_x=ADC(Pin(RC_ADC_X_PIN)); adc_x.atten(ADC.ATTN_11DB)
    adc_y=ADC(Pin(RC_ADC_Y_PIN)); adc_y.atten(ADC.ATTN_11DB)
    esp.add_peer(ROV_MAC)
    led.set('flash',CONN_FLASH_MS); time.sleep_ms(500); led.set('fade',FADE_PERIOD_MS)
    last_x=last_y=None; t0=millis(); a0=0
    while True:
        now=millis()
        if time.ticks_diff(now,t0)>=SEND_INTERVAL:
            rx=adc_x.read(); ry=adc_y.read()
            px=int((rx/4095*200)-100)-OFFSET_X; py=int((ry/4095*200)-100)-OFFSET_Y
            x=0 if abs(px)<AXIS_DEADZONE else max(-100,min(100,px))
            y=0 if abs(py)<AXIS_DEADZONE else max(-100,min(100,py))
            if (x,y)!=(last_x,last_y): msg=f"{x}:{y}"; esp.send(ROV_MAC,msg); print("Sent->",msg); last_x,last_y=x,y; \
                led.set('flash',FAST_FLASH_MS); a0=now
            elif led.mode=='flash' and time.ticks_diff(now,a0)>=FAST_FLASH_MS: led.set('fade',FADE_PERIOD_MS)
            t0=now
        led.update()

# ----------------------------------------
# ROV MODE
# ----------------------------------------
elif MODE=='ROV':
    i2c=I2C(0,scl=Pin(I2C_SCL_PIN),sda=Pin(I2C_SDA_PIN),freq=PCA_I2C_FREQ)
    pca=PCA9685(i2c); pca.freq(50)
    # center/stop
    cent=int(1500/20000*4096)
    pca.duty(SERVO_CH,cent); pca.duty(ENA_CH,0); pca.duty(IN1_CH,0); pca.duty(IN2_CH,0)
    led.set('flash',CONN_FLASH_MS); time.sleep_ms(500); led.set('fade',FADE_PERIOD_MS)
    a0=0
    while True:
        host,msg=esp.recv(RECV_TIMEOUT); now=millis()
        if msg:
            x,y=map(int,msg.decode().split(':'))
            pul=(x+100)/200*2000+500; pca.duty(SERVO_CH,int(pul/20000*4096))
            if y>0:   pca.duty(IN1_CH,4095); pca.duty(IN2_CH,0);  pca.duty(ENA_CH,int(y*4095/100))
            elif y<0: pca.duty(IN1_CH,0);    pca.duty(IN2_CH,4095); pca.duty(ENA_CH,int(-y*4095/100))
            else:     pca.duty(IN1_CH,0);    pca.duty(IN2_CH,0);    pca.duty(ENA_CH,0)
            print("Rx<-",msg.decode()); led.set('flash',FAST_FLASH_MS); a0=now
        elif led.mode=='flash' and time.ticks_diff(now,a0)>=FAST_FLASH_MS: led.set('fade',FADE_PERIOD_MS)
        led.update()

# ----------------------------------------
# DIAGNOSTICS MODE
# ----------------------------------------
elif MODE=='DIAG':
    i2c=I2C(0,scl=Pin(I2C_SCL_PIN),sda=Pin(I2C_SDA_PIN),freq=PCA_I2C_FREQ)
    print(f"Diag I2C SCL={I2C_SCL_PIN}, SDA={I2C_SDA_PIN}")
    devs=i2c.scan(); print("I2C devs:",[hex(d) for d in devs])
    if PCA_ADDR in devs:
        m=i2c.readfrom_mem(PCA_ADDR,0,1)[0]; p=i2c.readfrom_mem(PCA_ADDR,0xFE,1)[0]
        print(f"PCA@0x{PCA_ADDR:02X} MODE1=0x{m:02X},PRE={p}")
        pca=PCA9685(i2c); pca.freq(50)
        print("PWM50Hz. Sweep ch0:")
        for ch in DIAG_CHANNELS:
            print(f"Ch{ch}:")
            for d in (0,2048,4095): print(f" {d/4095*100:.0f}%",end="");pca.duty(ch,d);time.sleep(0.5)
            # read regs
            base=0x06+4*ch; regs=i2c.readfrom_mem(PCA_ADDR,base,4)
            print(f" regs={regs}")
        # reset
        for ch in DIAG_CHANNELS: pca.duty(ch,0)
    else:
        print("PCA not found.")

else:
    raise ValueError("MODE must be 'RC','ROV', or 'DIAG'")
