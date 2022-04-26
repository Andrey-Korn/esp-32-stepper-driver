from machine import Pin
from machine import Timer
import time

led = Pin(2, Pin.OUT)
tim0 = Timer(0)

def LED_toggle(t):
    led.value(not(led.value()))

tim0.init(period=500, callback=LED_toggle)

while(True):
    pass
