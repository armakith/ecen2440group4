import math, time
import machine
# load the MicroPython pulse-width-modulation module for driving hardware
from machine import PWM
from machine import Pin

#======================================================================#
# HARDWARE NEEDED:

# Raspberry Pi Pico, DRV8835, motors and Voltage Regulator w/ Benchtop Power Supply

#======================================================================#
# BENCHTOP EQUIPMENT:

# Oscilloscope to measure the Voltage across
# a motor. Include a screenshot of the
# measurement and summary in the Lab
# Report. Run the motors for ~1minute to
# measure the batteries output voltage.
# Monitor the voltage from the battery and
# record both the initial voltage across the
# battery and the final voltage across the
# battery.
# Verify that the elapsed time from sending
# the signal to reception of the IR signal
# using the Oscilloscope to measure both the
# IR Transmitter output and the IR Receiver
# input, be sure to have common ground is
# approximately the same as previously
# measured.

#======================================================================#
# SOFTWARE NEEDS:

# Controlling the motors: Serial Monitor

#======================================================================#

####### MOTOR CONTROL WITH PUSHBUTTON !!!
# START WITH MOTOR OFF
# PRESS BUTTON TO TURN MOTOR ON
# PRINT THE ELAPSED TIME IN TERMINAL
# PRESS BUTTON TO TURN THE MOTOR OFF

motor_button = Pin(26, Pin.IN, Pin.PULL_UP)    # BUTTON IS CONFIGURED IN PULL-UP TO 3.3V 

# use this timer to roughly know how long the motor has been running for
global countdown_timer
countdown_timer = 0

# motor settings
pwm_rate = 2000   #ORIG:    2000
ain1_ph = Pin(12, Pin.OUT)                                  # MOTOR CHANNEL A 
ain2_en = PWM(13, freq = pwm_rate, duty_u16 = 0)
bin1_ph = Pin(14, Pin.OUT)                                  # MOTOR CHANNEL B
bin2_en = PWM(15, freq = pwm_rate, duty_u16 = 0)
pwm = min(max(int(2**16 * abs(1)), 0), 65535)   ##Orig: 65535

global motor_state 
motor_state = False     # Motor on or off!    True means the motor will operate

# INITIAL DIRECTION OF MOTORS DRIVE DIRECTION       # .low() is      # .high() is 
ain1_ph.low()
bin1_ph.low()

def motor_control():
    global motor_state, direction, countdown_timer
    if (motor_state == False):
        print("*** Motors ON ***")
        print("    Counter (seconds):")
        motor_state = True
        ain2_en.duty_u16(pwm)
        bin2_en.duty_u16(pwm)
    elif (motor_state == True):
        print("*** Motors OFF ***")
        print("    Counter reset")
        countdown_timer = 0
        motor_state = False
        ain2_en.duty_u16(0)
        bin2_en.duty_u16(0)

# button debounce is important inside callback function!
debounce_time = 0

# Callback function to execute when an IR code is received
def motor_callback(motor_button):
    global interrupt_flag, debounce_time
    if (time.ticks_ms()-debounce_time) > 500:
        motor_control()
        debounce_time=time.ticks_ms()
    
motor_button.irq(trigger=Pin.IRQ_FALLING, handler=motor_callback)

time.sleep(1) #wait for usb to become ready
print("[[[Motor controlled by button]]]\n")

# Main loop to keep the script running
while True:
    if (motor_state == True):
        countdown_timer += 1
        print(countdown_timer)
        time.sleep(1) # 1 second
