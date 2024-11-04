import math, time
import machine
# load the MicroPython pulse-width-modulation module for driving hardware
from machine import PWM
from machine import Pin

from ir_rx.nec import NEC_8 # Use the NEC 8-bit class
from ir_rx.print_error import print_error # for debugging

#======================================================================#
# HARDWARE NEEDED:

# Raspberry Pi Pico w/ TSOP328, 3 User LEDs, Transmitter Circuit and Benchtop Power Supply

#======================================================================#
# BENCHTOP EQUIPMENT:

# Benchtop Power Supply to power Pico &
# use oscilloscope at that Receive pin on the
# Pico to verify reception of IR Signal.
# Conduct several measurements to
# experimentally determine the range of the
# IR Receiver, document these
# measurements and summarize the results in
# the report.

#======================================================================#
# SOFTWARE NEEDS:

# IR Receiver Code: Serial Monitor, IR Transmitter Code: Serial Monitor

#======================================================================#

green_led = Pin(16, Pin.OUT)
yellow_led = Pin(17, Pin.OUT)
red_led = Pin(18, Pin.OUT)

def flash_led():
    green_led.value(1)
    yellow_led.value(1)
    red_led.value(1)
    time.sleep(0.25)
    green_led.value(0)
    yellow_led.value(0)
    red_led.value(0)

# Callback function to execute when an IR code is received
def ir_callback(data, addr, _):
    print(f"Received NEC command! Data: 0x{data:02X}, Addr: 0x{addr:02X}")
    flash_led()

# Setup the IR receiver
ir_pin = Pin(0, Pin.IN, Pin.PULL_UP) 

ir_receiver = NEC_8(ir_pin, callback=ir_callback)

# Optional: Use the print_error function for debugging
ir_receiver.error_function(print_error)

time.sleep(1) # Wait for USB to become ready

print("Listening...")

# Main loop to keep the script running
while True:
    pass
