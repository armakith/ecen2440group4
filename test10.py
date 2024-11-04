
import math, time
import machine
from machine import Pin

#======================================================================#
# HARDWARE NEEDED:

# Raspberry Pi Pico, 4 LEDs w/ 1kOhm resistors RF Transmitter, 
# & Receiver (use 4.7kohm resistors for the voltage dividers), Benchtop Power Supply

#======================================================================#
# BENCHTOP EQUIPMENT:

# Oscilloscope to measure the voltage at the
# output of the voltage dividers, benchtop
# power supplies.

#======================================================================#
# SOFTWARE NEEDS:

# Code to receive input from the RF
# Receiver using the Pico.
# Code to map the buttons on the RF
# transmitter to the LEDs connected
# as outputs from the Pico.

#======================================================================#


#========================================================================================================#
# Define LED pins 

BLUE_LED_PIN = 9
GREEN_LED_PIN = 8
YELLOW_LED_PIN = 7   
RED_LED_PIN = 6

blue_led = Pin(BLUE_LED_PIN, Pin.OUT)
green_led = Pin(GREEN_LED_PIN, Pin.OUT)
yellow_led = Pin(YELLOW_LED_PIN, Pin.OUT)
red_led = Pin(RED_LED_PIN, Pin.OUT)

blue_led.value(0)
green_led.value(0)
yellow_led.value(0)
red_led.value(0)

def flash_led():
    blue_led.value(1)
    green_led.value(1)
    yellow_led.value(1)
    red_led.value(1)
    time.sleep(0.1)
    blue_led.value(0)
    green_led.value(0)
    yellow_led.value(0)
    red_led.value(0)

#========================================================================================================#
# RF RECEIVER PINS 

D0_PIN = 18     # MAPS TO [D] BUTTON ON CONTROLLER      && RED_LED
D1_PIN = 19     # MAPS TO [C] BUTTON ON CONTROLLER      && YELLOW_LED
D2_PIN = 20     # MAPS TO [B] BUTTON ON CONTROLLER      && GREEN_LED
D3_PIN = 21     # MAPS TO [A] BUTTON ON CONTROLLER      && BLUE_LED

D0_RF_INPUT = Pin(D0_PIN, Pin.IN, Pin.PULL_UP)
D1_RF_INPUT = Pin(D1_PIN, Pin.IN, Pin.PULL_UP)
D2_RF_INPUT = Pin(D2_PIN, Pin.IN, Pin.PULL_UP)
D3_RF_INPUT = Pin(D3_PIN, Pin.IN, Pin.PULL_UP)

def RF_LED_ON(led):
    led.value(1)

def RF_LED_OFF(led):
    led.value(0)

# RF BUTTONS AS INTERUPTS:
"""
# button debounce is important inside callback function!
RF_debounce_time = 0
# Callback function to execute when an IR code is received
def RF_ON_callback(pin):
    global RF_debounce_time
    if (time.ticks_ms()-RF_debounce_time) > 500:

        if D0_RF_INPUT.value() == 1:
            RF_LED_ON(red_led)
        else:
            RF_LED_OFF(red_led)

        if D1_RF_INPUT.value() == 1:
            RF_LED_ON(yellow_led)
        else:
            RF_LED_OFF(yellow_led)

        if D2_RF_INPUT.value() == 1:
            RF_LED_ON(green_led)
        else:
            RF_LED_OFF(green_led)

        if D3_RF_INPUT.value() == 1:
            RF_LED_ON(blue_led)
        else:
            RF_LED_OFF(blue_led)
        
        RF_debounce_time=time.ticks_ms()


D0_RF_INPUT.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = RF_ON_callback)
D1_RF_INPUT.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = RF_ON_callback)
D2_RF_INPUT.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = RF_ON_callback)
D3_RF_INPUT.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = RF_ON_callback)
"""

time.sleep(1) # 1 second delay before main loop

def main():

    while True:
       
        #=====================================================================///
        # RF BUTTON POLLING:
        if (D0_RF_INPUT.value() == 1):
            RF_LED_ON(red_led)
        else:
            RF_LED_OFF(red_led)
        
        if (D1_RF_INPUT.value() == 1):
            RF_LED_ON(yellow_led)
        else:
            RF_LED_OFF(yellow_led)

        if (D2_RF_INPUT.value() == 1):
            RF_LED_ON(green_led)
        else:
            RF_LED_OFF(green_led)

        if (D3_RF_INPUT.value() == 1):
            RF_LED_ON(blue_led)
        else:
            RF_LED_OFF(blue_led)
        #=====================================================================///

        time.sleep(0.1)  # Delay to prevent overwhelming the output

if __name__ == "__main__":
    main()
