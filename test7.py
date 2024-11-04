import math, time
import machine
# load the MicroPython pulse-width-modulation module for driving hardware
from machine import PWM
from machine import Pin

from ir_rx.nec import NEC_8 # Use the NEC 8-bit class
from ir_rx.print_error import print_error # for debugging

#======================================================================#
# HARDWARE NEEDED:

# Raspberry Pi Pico, DRV8835, motors, TSOP328, 3 User LEDs, 
# Transmitter Circuit and Voltage Regulator w/ Benchtop Power Supply

#======================================================================#
# BENCHTOP EQUIPMENT:

# Measure elapsed time from sending the
# signal to reception of the IR signal using
# the Oscilloscope to measure both the IR
# Transmitter output and the IR Receiver
# input, be sure to have common ground.
# Use Benchtop Power Supply to provide
# power to the system through the voltage
# regulator.
# Use Multimeter to measure the loaded
# output voltage of the regulator. Use the
# multimeter to measure the current draw of
# the SumoBot. Document these
# measurements and summarize the results in
# the report.

#======================================================================#
# SOFTWARE NEEDS:

# Integrated Motor Control w/ IR Receiver: Serial Monitor IR Transmitter Code: Serial Monitor

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

####### MOTOR CONTROL WITH IR RECEIVER!!!
# MOTOR TURNS ON FROM OFF STATE, GOES FORWARD AT IR_RX
# MOTOR SHUTS OFF AT IR_RX
# MOTOR TURNS ON FROM OFF STATE, GOES BACKWARD AT IR_RX
# MOTOR SHUTS OFF AT IR_RX

pwm_rate = 2000   #ORIG:    2000
ain1_ph = Pin(12, Pin.OUT)                                  # MOTOR CHANNEL A 
ain2_en = PWM(13, freq = pwm_rate, duty_u16 = 0)
bin1_ph = Pin(14, Pin.OUT)                                  # MOTOR CHANNEL B
bin2_en = PWM(15, freq = pwm_rate, duty_u16 = 0)
pwm = min(max(int(2**16 * abs(1)), 0), 65535)   ##Orig: 65535

global motor_state 
motor_state = False     # Motor on or off!    True means the motor will operate

global motor_forward
motor_forward = True    # Direction!          False means the motor will go backwards

global direction 
direction = "Driving Forward"

# INITIAL DIRECTION OF MOTORS DRIVE DIRECTION       # .low() is      # .high() is 
ain1_ph.low()
bin1_ph.low()

def switch_direction():
    global motor_forward, direction
    if (motor_forward == True):
        motor_forward = False
        direction = "Driving backward"
        ain1_ph.high()
        bin1_ph.high()
    elif (motor_forward == False):
        motor_forward = True
        direction = "Driving forward"
        ain1_ph.low()
        bin1_ph.low()

def motor_control():
    global motor_state, direction
    if (motor_state == False):
        print(direction) # Print to REPL
        motor_state = True
        ain2_en.duty_u16(pwm)
        bin2_en.duty_u16(pwm)
    elif (motor_state == True):
        print(direction) # Print to REPL
        motor_state = False
        ain2_en.duty_u16(0)
        bin2_en.duty_u16(0)
        switch_direction()

# Callback function to execute when an IR code is received
def ir_callback(data, addr, _):
    print(f"Received NEC command! Data: 0x{data:02X}, Addr: 0x{addr:02X}")
    flash_led()
    motor_control()

# Setup the IR receiver
ir_pin = Pin(0, Pin.IN, Pin.PULL_UP) # Adjust the pin number based on your wiring

ir_receiver = NEC_8(ir_pin, callback=ir_callback)

# Optional: Use the print_error function for debugging
ir_receiver.error_function(print_error)

time.sleep(1) # Wait for USB to become ready

# Main loop to keep the script running
while True:
    pass
