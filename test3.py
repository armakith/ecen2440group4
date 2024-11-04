from machine import Pin
import time
import random

#======================================================================#
# HARDWARE NEEDED:

# Raspberry Pi Pico w/ Benchtop Power Supply & 3 User LEDs

#======================================================================#
# BENCHTOP EQUIPMENT:

# Benchtop Power Supply to power Pico, visually check the LEDs Use multimeter to measure the current
# draw of the system and document these measurements in the Lab Report.

#======================================================================#
# SOFTWARE NEEDS:

# Blink the 3 user LEDs

#======================================================================#

led_one = Pin(2, Pin.OUT)
led_two = Pin(3, Pin.OUT)
led_three = Pin(4, Pin.OUT)

while True:

    #Flash LEDs
    led_one.value(1)
    time.sleep(0.25)
    led_one.value(0)
    time.sleep(0.25)

    led_two.value(1)
    time.sleep(0.25)
    led_two.value(0)
    time.sleep(0.25)

    led_three.value(1)
    time.sleep(0.25)
    led_three.value(0)
    time.sleep(0.25)
