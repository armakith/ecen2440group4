import math, time
import machine

# load the MicroPython pulse-width-modulation module for driving hardware
from machine import PWM
from machine import Pin

# for IR receiver
from ir_rx.nec import NEC_8 # Use the NEC 8-bit class
from ir_rx.print_error import print_error # for debugging

#======================================================================#
# HARDWARE NEEDED:

# Raspberry Pi Pico,
# DRV8835, motors,
# TSOP328, IR Transmitter
# Circuit w/ Joystick, 4 LEDs
# w/ 1kOhm resistors RF
# Transmitter, & Receiver
# (use 4.7kohm resistors for
# the voltage dividers), and
# Voltage Regulator w/
# Benchtop Power Supply
# Additional circuitry as
# needed.

#======================================================================#
# BENCHTOP EQUIPMENT:

#                       STUDENT'S DESIGN!!!

#======================================================================#
# SOFTWARE NEEDS:

# Integration of the RF Receiver w/
# the rest of the code base.
# Ability to switch between the RF
# and IR receivers.
# The Joystick controls should be
# mapped to the following
# functionality of the SumoBot:
# Drive Forward, Backward, Turn
# Left, Turn Right.
# The RF Transmitter code is also
# mapped to the following:
# functionality of the SumoBot:
# Drive Forward, Backward, Turn
# Left, Turn Right.


#========================================================================================================#
# Define LED pins 

board_led = Pin(25, Pin.OUT)

def flash_board_led(x_times):
    for i in range(x_times):
        board_led.value(1)
        time.sleep(0.1)
        board_led.value(0)
        time.sleep(0.1)

BLUE_LED_PIN = 9
GREEN_LED_PIN = 8
YELLOW_LED_PIN = 7   
RED_LED_PIN = 6

blue_led = Pin(BLUE_LED_PIN, Pin.OUT)
green_led = Pin(GREEN_LED_PIN, Pin.OUT)
yellow_led = Pin(YELLOW_LED_PIN, Pin.OUT)
red_led = Pin(RED_LED_PIN, Pin.OUT)

# LEDS ARE OFF ON STARTUP
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

D0_RF_INPUT = Pin(D0_PIN, Pin.IN, Pin.PULL_DOWN)             
D1_RF_INPUT = Pin(D1_PIN, Pin.IN, Pin.PULL_DOWN)               
D2_RF_INPUT = Pin(D2_PIN, Pin.IN, Pin.PULL_DOWN)
D3_RF_INPUT = Pin(D3_PIN, Pin.IN, Pin.PULL_DOWN)

def RF_LED_ON(led):
    led.value(1)

def RF_LED_OFF(led):
    led.value(0)

# RF BUTTONS AS INTERUPTS:
# CURRENT ISSUES: 
# BUTTON DEBOUNCE TIME WILL INTERFERE WITH DETECTING THE FALLING EDGE WHEN RELEASING THE BUTTON, 
# CAUSING THE LED OFF AND MOTOR STOP FUNCTIONS TO NOT EXECUTE
# WHEN THE BUTTON IS PRESSED AND RELEASED QUICKLY
# 
# SOLUTIONS: DEBUG THE INTERRUPT FUNCTION
#            USE POLLING INSTEAD (ALREADY IMPLEMENTED, SEE MAIN FUNCTION LOOP)

# RF CONTROLLER DRIVE MAPPING:
#
#   # HOLD THE CONTROLLER DIAGONAL WITH THE B BUTTON FACING FORWARD !!!
#
#       # TRY TO POINT THE RF CONTROLLER AT THE BOT !!!
#
#       [A]     BUTTON:     DRIVE FORWARD                   D3_PIN
#       [D]     BUTTON:     DRIVE BACKWARD                  D0_PIN
#       [C]     BUTTON:     SPIN LEFT                       D1_PIN
#       [B]     BUTTON:     SPIN RIGHT                      D2_PIN

#"""
# button debounce is important inside callback function!
RF_debounce_time = 0
# Callback function to execute when an IR code is received
def RF_ON_callback(pin):
    global RF_debounce_time
    if (time.ticks_ms()-RF_debounce_time) > 300:

        if D2_RF_INPUT.value() == 1:
            RF_LED_ON(red_led)
            print("     SPIN RIGHT")
            spin_right()
        elif D0_RF_INPUT.value() == 1:
            RF_LED_ON(yellow_led)
            print("     DRIVE BACKWARD")
            drive_backward()
        elif D3_RF_INPUT.value() == 1:
            RF_LED_ON(green_led)
            print("     DRIVE FORWARD")
            drive_forward()
        elif D1_RF_INPUT.value() == 1:
            RF_LED_ON(blue_led)
            print("     SPIN LEFT")
            spin_left()
        else:
            RF_LED_OFF(blue_led)
            RF_LED_OFF(green_led)
            RF_LED_OFF(yellow_led)
            RF_LED_OFF(red_led)
            print("     STOP MOTORS")
            motors_stop()
        
        RF_debounce_time=time.ticks_ms()

D0_RF_INPUT.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = RF_ON_callback)
D1_RF_INPUT.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = RF_ON_callback)
D2_RF_INPUT.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = RF_ON_callback)
D3_RF_INPUT.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = RF_ON_callback)

#"""
#========================================================================================================#
# MOTOR SETTINGS

pwm_rate = 2000   # THIS VALUE DOES NOT AFFECT SPEED OF MOTORS

# MOTOR CHANNEL A 
ain1_ph = Pin(12, Pin.OUT)                                  
ain2_en = PWM(13, freq = pwm_rate, duty_u16 = 0)
# MOTOR CHANNEL B
bin1_ph = Pin(14, Pin.OUT)                                  
bin2_en = PWM(15, freq = pwm_rate, duty_u16 = 0)

#                                      XXXXX    65535 IS FULL SPEED 
pwm = min(max(int(2**16 * abs(1)), 0), 4000)   #   FOR TESTING PURPOSES USE SLOWER SPEED

# INITIAL DIRECTION OF MOTORS DRIVE DIRECTION       # .low() is FORWARD     # .high() is BACKWARD
ain1_ph.low()
bin1_ph.low()

def motors_stop():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)

def drive_forward():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()
    bin1_ph.low()
    ain2_en.duty_u16(pwm)
    bin2_en.duty_u16(pwm)
    #forward

def drive_backward():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()
    bin1_ph.high()
    ain2_en.duty_u16(pwm)
    bin2_en.duty_u16(pwm)
    #backward

def spin_left():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()
    bin1_ph.low()
    ain2_en.duty_u16(pwm)
    bin2_en.duty_u16(pwm)
    #left back right forward

def spin_right():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()
    bin1_ph.high()
    ain2_en.duty_u16(pwm)
    bin2_en.duty_u16(pwm)
    #left fwd right back


#========================================================================================================#
# IR receiver

# CONFIGURE THIS PIN TO POWER THE IR RECEIVER
# WE CAN TOGGLE THIS PIN ON/OFF TO CONTROL THE IR RECEIVER
# default/ on startup we want the IR receiver on
ir_power_pin = Pin(22, Pin.OUT)
ir_power_pin.value(1)

#data input from IR
ir_pin = Pin(16, Pin.IN, Pin.PULL_UP)

# Callback function to execute when an IR code is received
def ir_callback(data, addr, _):
    print(f"Received NEC command! Data: 0x{data:02X}, Addr: 0x{addr:02X}")
    flash_board_led(1)
    if (data == 0x01):
        #drive forward
        print("     Drive FORWARD")
        drive_forward()
    if (data == 0x02):
        #drive backward
        print("     Drive BACKWARD")
        drive_backward()
    if (data == 0x03):
        #drive left
        print("     SPIN LEFT")
        spin_left()
    if (data == 0x04):
        #drive right
        print("     SPIN RIGHT")
        spin_right()
    if (data == 0x05):
        #stop motors
        print("     MOTORS STOP")
        motors_stop()
    time.sleep(0.1)

ir_receiver = NEC_8(ir_pin, callback=ir_callback)
      
#========================================================================================================#
# IR/RF RECEIVER TOGGLE PUSHBUTTON

push_button = Pin(17, Pin.IN, Pin.PULL_UP)    # BUTTON IS CONFIGURED IN PULL-UP TO 3.3V 

# button debounce is important inside callback function!
debounce_time = 0

# Callback function to execute when an IR code is received
def push_button_callback(push_button):
    global debounce_time
    if (time.ticks_ms()-debounce_time) > 500:
        #motor_control()
        print("Switching Receiver Mode!")
        switch_receiver()
        debounce_time=time.ticks_ms()
    
push_button.irq(trigger=Pin.IRQ_FALLING, handler=push_button_callback)

# WE ARE HAVING THE IR RECEIVER ON BY DEFAULT, SO TURN THE RF INPUT INTERRUPT HANDLERS OFF
# so disable the RF interrupt handlers
D0_RF_INPUT.irq(handler = None)
D1_RF_INPUT.irq(handler = None)
D2_RF_INPUT.irq(handler = None)
D3_RF_INPUT.irq(handler = None)

global ir_mode 
ir_mode = True   # if false, then bot is using RF receiver instead

# Optional: Use the print_error function for debugging
# THIS WILL PRINT "INVALID START PULSE" WHEN TOGGLING BETWEEN THE IR/RF RECEIVER MODES  (DEBUG THIS)
ir_receiver.error_function(print_error)

def switch_receiver():
    global ir_mode, ir_receiver
    if (ir_mode == True):
        print("~RF RECEIVER ACTIVE~")
        ir_mode = False
        # DISABLE POWER PIN TO IR RECEIVER TO SHUT IT OFF
        ir_power_pin.value(0)
        #TURN ON THE RF INTERRUPTS
        D0_RF_INPUT.irq(handler = RF_ON_callback)
        D1_RF_INPUT.irq(handler = RF_ON_callback)
        D2_RF_INPUT.irq(handler = RF_ON_callback)
        D3_RF_INPUT.irq(handler = RF_ON_callback)
        #flash onboard LED to signal execution
        flash_board_led(4)

    elif (ir_mode == False):
        print("~IR RECEIVER ACTIVE~")
        ir_mode = True
        # ENABLE POWER PIN TO IR RECEIVER TO TURN IT ON
        ir_power_pin.value(1)
        #TURN OFF THE RF INTERRUPTS
        D0_RF_INPUT.irq(handler = None)
        D1_RF_INPUT.irq(handler = None)
        D2_RF_INPUT.irq(handler = None)
        D3_RF_INPUT.irq(handler = None)
        #flash onboard LED to signal execution
        flash_board_led(4)
        
#========================================================================================================#

time.sleep(1) # 1 second delay before main loop
print("Goliath online!")
#flash lights to signal the main loop is starting
blue_led.value(1)
green_led.value(1)
yellow_led.value(1)
red_led.value(1)
time.sleep(0.1)
blue_led.value(0)
green_led.value(0)
yellow_led.value(0)
red_led.value(0)
time.sleep(0.1)


def main():

    while True:
        
        pass

        # HERE ARE THE POLLING VERSIONS OF RF AND IR RECEIVERS FOR FUTURE USE IF NEEDED
        #=====================================================================///
        """
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
        """
        #=====================================================================///

        #=====================================================================///
        """
        #JOYSTICK POLLING:
        # Read joystick values
        current_x = seesaw_device.analog_read(JOYSTICK_X_PIN)
        current_y = seesaw_device.analog_read(JOYSTICK_Y_PIN)

        # Check if joystick position has changed significantly
        if abs(current_x - last_x) > joystick_threshold or abs(current_y - last_y) > joystick_threshold:
           
            print("Joystick moved - X:", current_x, ", Y:", current_y)
            last_x, last_y = current_x, current_y
           
            # JOYSTICK
            if current_y < joystick_center_y - joystick_threshold:  # Joystick moved up
                print("j_up")
                joystick_up()
            elif current_y > joystick_center_y + joystick_threshold:  # Joystick moved down
                print("j_down")
                joystick_down()
            elif current_x < joystick_center_x - joystick_threshold:  # Joystick moved left
                print("j_left")
                joystick_left()
            elif current_x > joystick_center_x + joystick_threshold:  # Joystick moved right
                print("j_right")
                joystick_right()
            else:
                print("j_STOPPED")
                motors_stop()
        """
        #=====================================================================///

        # DISABLED THIS TO TEST SYSTEM RESPONSE TIMES
        #time.sleep(0.1)  # Delay to prevent overwhelming the output

if __name__ == "__main__":
    main()
