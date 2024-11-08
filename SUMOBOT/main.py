import math, time
import machine

# load the MicroPython pulse-width-modulation module for driving hardware
from machine import PWM, Pin
#from machine import Pin

# for IR receiver
from ir_rx.nec import NEC_8 # Use the NEC 8-bit class
from ir_rx.print_error import print_error # for debugging

#========================================================================================================#
# LED SETUP AND FUNCTIONS
#========================================================================================================#

BOARD_LED_PIN = 25
board_led = Pin(BOARD_LED_PIN, Pin.OUT)

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

# SET LEDS TO OFF AT STARTUP
blue_led.value(0)
green_led.value(0)
yellow_led.value(0)
red_led.value(0)

def flash_4leds(x_times):
    for i in range(x_times):
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

#========================================================================================================#
# RF RECEIVER SETUP AND FUNCTIONS
#========================================================================================================#

def disable_rf_irq():
    D0_RF_INPUT.irq(handler = None)
    D1_RF_INPUT.irq(handler = None)
    D2_RF_INPUT.irq(handler = None)
    D3_RF_INPUT.irq(handler = None)

def enable_rf_irq():
    D0_RF_INPUT.irq(handler = RF_ON_callback)
    D1_RF_INPUT.irq(handler = RF_ON_callback)
    D2_RF_INPUT.irq(handler = RF_ON_callback)
    D3_RF_INPUT.irq(handler = RF_ON_callback)

D0_PIN = 18     # MAPS TO [D] BUTTON ON CONTROLLER
D1_PIN = 19     # MAPS TO [C] BUTTON ON CONTROLLER
D2_PIN = 20     # MAPS TO [B] BUTTON ON CONTROLLER
D3_PIN = 21     # MAPS TO [A] BUTTON ON CONTROLLER

D0_RF_INPUT = Pin(D0_PIN, Pin.IN, Pin.PULL_DOWN)             
D1_RF_INPUT = Pin(D1_PIN, Pin.IN, Pin.PULL_DOWN)               
D2_RF_INPUT = Pin(D2_PIN, Pin.IN, Pin.PULL_DOWN)
D3_RF_INPUT = Pin(D3_PIN, Pin.IN, Pin.PULL_DOWN)

# RF CONTROLLER DRIVE MAPPING:
# 
#       [A]     BUTTON:     DRIVE FORWARD                   D3_PIN
#       [B]     BUTTON:     SPIN RIGHT                      D2_PIN
#       [C]     BUTTON:     SPIN LEFT                       D1_PIN
#       [D]     BUTTON:     DRIVE BACKWARD                  D0_PIN

# Callback function to execute when an RF code is received
def RF_ON_callback(pin):
     # when an callback is activated, disable all RF Handlers so we cant 
     # request an interrupt while one is active
    disable_rf_irq()
    if D3_RF_INPUT.value() == 1:                # [A]     BUTTON:     DRIVE FORWARD
        drive_forward()
        while(1):
            if D3_RF_INPUT.value() == 0:
                motors_stop()
                break
    elif D2_RF_INPUT.value() == 1:              # [B]     BUTTON:     SPIN RIGHT
        spin_right()
        while(1):
            if D2_RF_INPUT.value() == 0:
                motors_stop()
                break
    elif D1_RF_INPUT.value() == 1:              # [C]     BUTTON:     SPIN LEFT
        spin_left()
        while(1):
            if D1_RF_INPUT.value() == 0:
                motors_stop()
                break        
    elif D0_RF_INPUT.value() == 1:              # [D]     BUTTON:     DRIVE BACKWARD
        drive_backward()
        while(1):
            if D0_RF_INPUT.value() == 0:
                motors_stop()
                break
    enable_rf_irq()

D0_RF_INPUT.irq(trigger=Pin.IRQ_RISING, handler = RF_ON_callback)
D1_RF_INPUT.irq(trigger=Pin.IRQ_RISING, handler = RF_ON_callback)
D2_RF_INPUT.irq(trigger=Pin.IRQ_RISING, handler = RF_ON_callback)
D3_RF_INPUT.irq(trigger=Pin.IRQ_RISING, handler = RF_ON_callback)

#========================================================================================================#
# MOTOR SETUP AND FUNCTIONS
#========================================================================================================#

pwm_rate = 2000   # THIS VALUE DOES NOT AFFECT SPEED OF MOTORS

# MOTOR CHANNEL A 
ain1_ph = Pin(12, Pin.OUT)                                  
ain2_en = PWM(13, freq = pwm_rate, duty_u16 = 0)
# MOTOR CHANNEL B
bin1_ph = Pin(14, Pin.OUT)                                  
bin2_en = PWM(15, freq = pwm_rate, duty_u16 = 0)

#                                      XXXXX    65535 IS FULL SPEED (100% DUTY)
pwm = min(max(int(2**16 * abs(1)), 0), 4000)   #   FOR TESTING PURPOSES USE SLOWER SPEED

# SET INITIAL DIRECTION OF MOTORS TO BE FORWARD       # .low() is FORWARD     # .high() is BACKWARD
ain1_ph.low()
bin1_ph.low()

def motors_stop():          
    ain2_en.duty_u16(0)     # STOPS THE MOTORS
    bin2_en.duty_u16(0)
    print("      MOTORS STOPPED")

def drive_forward():        
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()           # SET MOTOR DIRECTION FORWARD
    bin1_ph.low()
    ain2_en.duty_u16(pwm)   # DRIVES THE MOTORS
    bin2_en.duty_u16(pwm)
    print("     DRIVE FORWARD")

def drive_backward():       
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()          # SET MOTOR DIRECTION BACKWARD
    bin1_ph.high()
    ain2_en.duty_u16(pwm)   # DRIVES THE MOTORS
    bin2_en.duty_u16(pwm)
    print("     DRIVE BACKWARD")

def spin_left():            
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()          # LEFT WHEEL BACKWARD
    bin1_ph.low()           # RIGHT WHEEL FORWARD
    ain2_en.duty_u16(pwm)
    bin2_en.duty_u16(pwm)
    print("     SPIN LEFT")

def spin_right():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()           # LEFT WHEEL FORWARD
    bin1_ph.high()          # RIGHT WHEEL BACKWARD
    ain2_en.duty_u16(pwm)
    bin2_en.duty_u16(pwm)
    print("     SPIN RIGHT")

#========================================================================================================#
# IR RECEIVER SETUP AND FUNCTIONS
#========================================================================================================#

# CONFIGURE THIS PIN TO POWER THE IR RECEIVER
# WE CAN TOGGLE THIS PIN ON/OFF TO CONTROL THE IR RECEIVER
# IR RECEIVER IS SELECTED AS DEFAULT ON STARTUP
ir_power_pin = Pin(22, Pin.OUT)
ir_power_pin.value(1)

#data input from IR
ir_pin = Pin(16, Pin.IN, Pin.PULL_UP)

# Callback function to execute when an IR code is received
def ir_callback(data, addr, _):
    #print(f"Received NEC command! Data: 0x{data:02X}, Addr: 0x{addr:02X}")     # USE FOR TROUBLESHOOTING
    if (data == 0x01):      # drive forward
        drive_forward()
    elif (data == 0x02):    # drive backward
        drive_backward()
    elif (data == 0x03):    # drive left
        spin_left()
    elif (data == 0x04):    # drive right
        spin_right()
    elif (data == 0x05):    # stop motors
        motors_stop()

ir_receiver = NEC_8(ir_pin, callback=ir_callback)
      
#========================================================================================================#
# TOGGLE COMMS PUSHBUTTON SETUP AND FUNCTIONS
#========================================================================================================#

push_button = Pin(17, Pin.IN, Pin.PULL_UP)    # BUTTON IS CONFIGURED IN PULL-UP TO 3.3V 

# button debounce to deal with button bounce signals
debounce_time = 0
# Callback function to execute when an IR code is received
def push_button_callback(push_button):
    global debounce_time
    if (time.ticks_ms()-debounce_time) > 500:
        switch_receiver()
        debounce_time=time.ticks_ms()
    
push_button.irq(trigger=Pin.IRQ_FALLING, handler=push_button_callback)

# WE ARE HAVING THE IR RECEIVER ON BY DEFAULT, SO TURN THE RF INPUT INTERRUPT HANDLERS OFF
# so disable the RF interrupt handlers
disable_rf_irq()

global ir_mode 
ir_mode = True   # False means comms are set to RF Receiver instead

# Optional: Use the print_error function for debugging
# Note: Causes "INVALID START PULSE" WHEN TOGGLING BETWEEN THE IR/RF RECEIVER MODES
ir_receiver.error_function(print_error)        # USE FOR TROUBLESHOOTING IF NEEDED

def switch_receiver():
    global ir_mode
    print("Switching Receiver Mode!")
    motors_stop()
    if (ir_mode == True):
        print("~RF RECEIVER ACTIVE~")
        ir_mode = False
        ir_power_pin.value(0)       # DISABLE POWER PIN TO IR RECEIVER TO SHUT IT OFF
        enable_rf_irq()             # TURN ON THE RF INTERRUPTS
        flash_board_led(4)          # Flash onboard LED to signal execution
    elif (ir_mode == False):
        print("~IR RECEIVER ACTIVE~")
        ir_mode = True
        ir_power_pin.value(1)       # ENABLE POWER PIN TO IR RECEIVER TO TURN IT ON
        disable_rf_irq()            # TURN OFF THE RF INTERRUPTS
        flash_board_led(4)          # Flash onboard LED to signal execution
        
#========================================================================================================#
# MAIN SETUP AND FUNCTIONS
#========================================================================================================#

time.sleep(1) # 1 second delay before main loop
print("Goliath online!")
#flash lights to signal the main loop is starting
flash_4leds(4)

def main():

    while True:
        
        pass

if __name__ == "__main__":
    main()
