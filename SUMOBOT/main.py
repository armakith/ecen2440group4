import time


# load the MicroPython pulse-width-modulation module for driving hardware
from machine import PWM, Pin, ADC, WDT


# for IR receiver
from ir_rx.nec import NEC_8 # Use the NEC 8-bit class
from ir_rx.print_error import print_error # for debugging

# scheduler
from micropython import schedule


#========================================================================================================#
# SETUP WATCHDOG
#========================================================================================================#

wdt = WDT(timeout=2000) # 2 seconds

# works with IR because we constantly get IR signals, so feed the dog when we get an ir signal

# implementing this with rf will be different, how would we do this since we dont constantly send
# rf signals?

#========================================================================================================#
# EXTERNAL ANALOG VARIABLE KNOB
#========================================================================================================#
# FOR TESTING I AM USING THIS TO CHANGE THE "LOWER BOUNDS" OF THE MOTOR DRIVING PWM VALUE
# WE NEED TO KNOW THIS IF WE WANT TO HAVE VARIABLE STEERING THAT SWEEPS FROM LOWER BOUND TO FULL SPEED

ADC_0_PIN = 26
external_knob = ADC(ADC_0_PIN)


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
        full_forward(0x33)
        while(1):
            if D3_RF_INPUT.value() == 0:
                motors_stop(0x00)
                break
    elif D2_RF_INPUT.value() == 1:              # [B]     BUTTON:     SPIN RIGHT
        spin_right(9)
        while(1):
            if D2_RF_INPUT.value() == 0:
                motors_stop(0x00)
                break
    elif D1_RF_INPUT.value() == 1:              # [C]     BUTTON:     SPIN LEFT
        spin_left(9)
        while(1):
            if D1_RF_INPUT.value() == 0:
                motors_stop(0x00)
                break        
    elif D0_RF_INPUT.value() == 1:              # [D]     BUTTON:     DRIVE BACKWARD
        full_backward(0x88)
        while(1):
            if D0_RF_INPUT.value() == 0:
                motors_stop(0x00)
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


FULL_PWR = 65535  #65535 IS FULL SPEED (100% DUTY)

BACK_LEFT = 35000

BACK_RIGHT = 35000

low_pwr = 6553
#low_pwr = external_knob.read_u16()

### VIA TESTING FOUND ~6553 TO BE THE LOWER BOUND FOR PWM ON THE BULKY TEST SET UP
### ACTUAL BOT THIS VALUE WILL BE DIFFERENT

#                                           XXXXX    
pwm_full = min(max(int(2**16 * abs(1)), 0), FULL_PWR)   

pwm_back_left = min(max(int(2**16 * abs(1)), 0), BACK_LEFT)

pwm_back_right = min(max(int(2**16 * abs(1)), 0), BACK_RIGHT)

pwm_slow = min(max(int(2**16 * abs(1)), 0), low_pwr) # <---- toggle the slower drive setting

pwm_adjust = min(max(int(2**16 * abs(1)), 0), low_pwr) # <---- toggle the slower drive setting


#adjusting the external knob to find what the lower bound is according to the weight of bot
def adjust_pwm_slow():
    global low_pwr, pwm_slow
    low_pwr = external_knob.read_u16()
    print(f"knob val: {low_pwr}")
    pwm_slow = min(max(int(2**16 * abs(1)), 0), low_pwr)   


def turn_adjust(turning_code):
    global low_pwr, pwm_adjust
    PARTITIONS = 9 # since we use 0-9 in the hex number for turning instructions
    interval_size = (FULL_PWR - low_pwr) / 9
    turning_pwm = int(FULL_PWR - (turning_code * interval_size))
    pwm_adjust = min(max(int(2**16 * abs(1)), 0), turning_pwm) # <---- toggle the slower drive setting
    print(f"pwm adjust: {turning_pwm}")

def spin_adjust(turning_code):
    global low_pwr, pwm_adjust
    PARTITIONS = 9 # since we use 0-9 in the hex number for turning instructions
    interval_size = (FULL_PWR - low_pwr) / 9
    turning_pwm = int(low_pwr + (turning_code * interval_size))
    pwm_adjust = min(max(int(2**16 * abs(1)), 0), turning_pwm) # <---- toggle the slower drive setting
    print(f"pwm adjust: {turning_pwm}")


# SET INITIAL DIRECTION OF MOTORS TO BE FORWARD       # .low() is FORWARD     # .high() is BACKWARD
ain1_ph.low()           # RIGHT WHEEL
bin1_ph.low()           # 


def full_forward(data):
    #print(f"    FULL FORWARD {data}")
    #adjust_pwm_slow()       #update the amount of slow per the external knob's adc value
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()           # SET MOTOR DIRECTION FORWARD
    bin1_ph.high()
    ain2_en.duty_u16(pwm_full)   # DRIVES THE MOTORS
    bin2_en.duty_u16(pwm_full)
    wdt.feed()
    

def forward_left(turning_code):
    #print("    FWD/LEFT")
    turn_adjust(turning_code)      
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()           # SET MOTOR DIRECTION FORWARD
    bin1_ph.high()
    ain2_en.duty_u16(pwm_full)   # DRIVES THE MOTORS
    bin2_en.duty_u16(pwm_adjust)
    wdt.feed()
    

def forward_right(turning_code):
    #print("    FWD/RIGHT")
    turn_adjust(turning_code)        
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()           # SET MOTOR DIRECTION FORWARD
    bin1_ph.high()
    bin2_en.duty_u16(pwm_full)
    ain2_en.duty_u16(pwm_adjust)   # DRIVES THE MOTORS
    wdt.feed()
    

def full_backward(data):
    #print(f"    FULL BACKWARD{data}")
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()          # SET MOTOR DIRECTION BACKWARD
    bin1_ph.low()
    ain2_en.duty_u16(pwm_back_right)   # DRIVES THE MOTORS
    bin2_en.duty_u16(pwm_back_left)
    wdt.feed()
    

def backward_left(turning_code):
    #print("    BACK/LEFT")
    turn_adjust(turning_code)            
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()          # LEFT WHEEL BACKWARD
    bin1_ph.low()           # RIGHT WHEEL FORWARD
    ain2_en.duty_u16(pwm_full)   # DRIVES THE MOTORS
    bin2_en.duty_u16(pwm_adjust)
    wdt.feed()


def backward_right(turning_code):
    #print("    BACK/RIGHT")
    turn_adjust(turning_code)           
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()          # LEFT WHEEL BACKWARD
    bin1_ph.low()           # RIGHT WHEEL FORWARD
    bin2_en.duty_u16(pwm_full)
    ain2_en.duty_u16(pwm_adjust)   # DRIVES THE MOTORS
    
    wdt.feed()


def spin_left(turning_code): #turning code will determine how fast of a spin to do
    #print("    SPIN LEFT")
    spin_adjust(turning_code)            
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()         # RIGHT WHEEL FORWARD 
    bin1_ph.low()           # LEFT WHEEL BACKWARD
    bin2_en.duty_u16(pwm_adjust)
    ain2_en.duty_u16(pwm_adjust)
    wdt.feed()
    
    
def spin_right(turning_code): #turning code will determine how fast of a spin to do
    #print("    SPIN/RIGHT")
    spin_adjust(turning_code)
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()           # RIGHT WHEEL BACKWARD
    bin1_ph.high()          # LEFT WHEEL FORWARD
    ain2_en.duty_u16(pwm_adjust)
    bin2_en.duty_u16(pwm_adjust)
    wdt.feed()
    

def motors_stop(data):
    #print(f"    motors stop{data}")         
    ain2_en.duty_u16(0)     # STOPS THE MOTORS
    bin2_en.duty_u16(0)
    wdt.feed()
   
motors_stop(0x00) # incase of restart or brownout, issue a motor_stop command on restart

#========================================================================================================#
# IR RECEIVER SETUP AND FUNCTIONS
#========================================================================================================#

#data input from IR
ir_pin = Pin(16, Pin.IN, Pin.PULL_UP)

def turning_code(hex_num):
    last_digit = hex_num % 16
    print(f"        {last_digit}")
    return last_digit


# Callback function to execute when an IR code is received
def ir_callback(data, addr, _):
    
    #print(f"Received NEC command! Data: 0x{data:02X}, Addr: 0x{addr:02X}")     # USE FOR TROUBLESHOOTING

    if (addr == 0x61):      # drive forward
        #print(f"addr: 0x{addr:02X}, command: 0x{data:02X}")

        if (data == 0x00):
            #motors_stop()
            schedule(motors_stop, data)
        elif(data == 0x33):
            #full_forward()
            schedule(full_forward, data)
        elif(data == 0x88):
            #full_backward()
            schedule(full_backward, data)
        elif(0x3f < data < 0x50):
            #forward_left(turning_code(data))
             schedule(forward_left, turning_code(data))
        elif(0x4f < data < 0x60):
            #forward_right(turning_code(data))
             schedule(forward_right, turning_code(data))
        elif(0x5f < data < 0x70):
            #backward_left(turning_code(data))
             schedule(backward_left, turning_code(data))
        elif(0x6f < data < 0x80):
            #backward_right(turning_code(data))
             schedule(backward_right, turning_code(data))
        elif(0x0f < data < 0x20):
            #spin_left(turning_code(data))
             schedule(spin_left, turning_code(data))
        elif(0x1f < data < 0x30):
            #spin_right(turning_code(data))
             schedule(spin_right, turning_code(data))
       
       

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

def turn_off_ir():
    ir_receiver.close()
    time.sleep(0.5)


global ir_mode 
ir_mode = True   # False means comms are set to RF Receiver instead

# Optional: Use the print_error function for debugging
# Note: Causes "INVALID START PULSE" WHEN TOGGLING BETWEEN THE IR/RF RECEIVER MODES
ir_receiver.error_function(print_error)        # USE FOR TROUBLESHOOTING IF NEEDED

def switch_receiver():
    global ir_mode, ir_receiver
    print("Switching Receiver Mode!")
    motors_stop(0x00)
    if (ir_mode == True):
        print("~RF RECEIVER ACTIVE~")
        ir_mode = False
        turn_off_ir()                                                              
        enable_rf_irq()             # TURN ON THE RF INTERRUPTS
        flash_board_led(4)          # Flash onboard LED to signal execution
    elif (ir_mode == False):
        print("~IR RECEIVER ACTIVE~")
        ir_mode = True
        ir_receiver = NEC_8(ir_pin, callback=ir_callback)
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
