import math, time
import machine
# load the MicroPython pulse-width-modulation module for driving hardware
from machine import PWM
from machine import Pin

#======================================================================#
# HARDWARE NEEDED:

# Raspberry Pi Pico w/Benchtop Supply & DRV8835 no motors

#======================================================================#
# BENCHTOP EQUIPMENT:

# Benchtop Power Supply to power Pico &
# DRV8835, Oscilloscope to measure the
# voltage @ aen1, aph1, ben2, bph2, AO1,
# AO2, BO1, BO2, be sure to document any
# voltage spikes exceeding expectations

#======================================================================#
# SOFTWARE NEEDS:

# controlling the motors

#======================================================================#

time.sleep(1) # Wait for USB to become ready

pwm_rate = 2000   #ORIG:    2000

ain1_ph = Pin(12, Pin.OUT)                                  # MOTOR CHANNEL A 
ain2_en = PWM(13, freq = pwm_rate, duty_u16 = 0)

bin1_ph = Pin(14, Pin.OUT)                                  # MOTOR CHANNEL B
bin2_en = PWM(15, freq = pwm_rate, duty_u16 = 0)

pwm = min(max(int(2**16 * abs(1)), 0), 65535)   ##Orig: 65535

while True:
    print("Motor FWD") # Print to REPL
    ain1_ph.low()
    bin1_ph.low()
    ain2_en.duty_u16(pwm)
    bin2_en.duty_u16(pwm)
    time.sleep(2)
    print("Motor OFF") # Print to REPL
    ain1_ph.low()
    bin1_ph.low()
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    time.sleep(2)
    print("Motor BACK") # Print to REPL
    ain1_ph.high()
    bin1_ph.high()
    ain2_en.duty_u16(pwm)
    bin2_en.duty_u16(pwm)
    time.sleep(2)
    print("Motor OFF") # Print to REPL
    ain1_ph.high()
    bin1_ph.high()
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    time.sleep(2)
