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

wdt = WDT(timeout=1000) # 1 seconds

# works with IR because we constantly get IR signals, so feed the dog when we get an ir signal

# implementing this with rf will be different, how would we do this since we dont constantly send
# rf signals?

wdt.feed()

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

wdt.feed()

#========================================================================================================#
# RF RECEIVER SETUP AND FUNCTIONS
#========================================================================================================#

def disable_rf_irq():
    D0_RF_INPUT.irq(handler = None)
    D1_RF_INPUT.irq(handler = None)
    D2_RF_INPUT.irq(handler = None)
    D3_RF_INPUT.irq(handler = None)

def enable_rf_irq():
    D0_RF_INPUT.irq(handler = RF_D0_callback)
    D1_RF_INPUT.irq(handler = RF_D1_callback)
    D2_RF_INPUT.irq(handler = RF_D2_callback)
    D3_RF_INPUT.irq(handler = RF_D3_callback)

D0_PIN = 7     # MAPS TO [D] BUTTON ON CONTROLLER    #<---------- THIS NEEDS TO BE MOVED FOR PCB !!!!
D1_PIN = 6     # MAPS TO [C] BUTTON ON CONTROLLER
D2_PIN = 5     # MAPS TO [B] BUTTON ON CONTROLLER
D3_PIN = 4     # MAPS TO [A] BUTTON ON CONTROLLER

D0_RF_INPUT = Pin(D0_PIN, Pin.IN, Pin.PULL_DOWN)             
D1_RF_INPUT = Pin(D1_PIN, Pin.IN, Pin.PULL_DOWN)               
D2_RF_INPUT = Pin(D2_PIN, Pin.IN, Pin.PULL_DOWN)
D3_RF_INPUT = Pin(D3_PIN, Pin.IN, Pin.PULL_DOWN)

wdt.feed()

# RF CONTROLLER DRIVE MAPPING:
# 
#       [A]     BUTTON:     DRIVE FORWARD                   D3_PIN
#       [B]     BUTTON:     SPIN RIGHT                      D2_PIN
#       [C]     BUTTON:     SPIN LEFT                       D1_PIN
#       [D]     BUTTON:     DRIVE BACKWARD                  D0_PIN



def RF_D0_callback(pin):
    disable_rf_irq()  # when callback is activated, disable all RF Handlers
    if D0_RF_INPUT.value() == 1:
        print("BACKWARD")
        full_backward(0x88)
    elif D0_RF_INPUT.value() == 0:
        print("STOP")
        motors_stop(0x00)
    enable_rf_irq()
        
 

def RF_D1_callback(pin):
    disable_rf_irq()  # when callback is activated, disable all RF Handlers
    if D1_RF_INPUT.value() == 1:
        print("SPIN_LEFT")
        spin_left(5)
    elif D1_RF_INPUT.value() == 0:
        print("STOP")
        motors_stop(0x00)
    enable_rf_irq()
    

def RF_D2_callback(pin):
    disable_rf_irq()  # when callback is activated, disable all RF Handlers
    if D2_RF_INPUT.value() == 1:
        print("SPIN_RIGHT")
        spin_right(5)
    elif D2_RF_INPUT.value() == 0:
        print("STOP")
        motors_stop(0x00)
    enable_rf_irq()

    
def RF_D3_callback(pin):
    disable_rf_irq()  # when callback is activated, disable all RF Handlers
    if D3_RF_INPUT.value() == 1:
        print("FORWARD")
        full_forward(0x33)
    elif D3_RF_INPUT.value() == 0:
        print("STOP")
        motors_stop(0x00)
    enable_rf_irq()

wdt.feed()

#D0_RF_INPUT.irq(trigger=Pin.IRQ_RISING, handler = RF_D0_callback)
#D1_RF_INPUT.irq(trigger=Pin.IRQ_RISING, handler = RF_D1_callback)
#D2_RF_INPUT.irq(trigger=Pin.IRQ_RISING, handler = RF_D2_callback)
#D3_RF_INPUT.irq(trigger=Pin.IRQ_RISING, handler = RF_D3_callback)
#"""
D0_RF_INPUT.irq(trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING, handler = RF_D0_callback)
D1_RF_INPUT.irq(trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING, handler = RF_D1_callback)
D2_RF_INPUT.irq(trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING, handler = RF_D2_callback)
D3_RF_INPUT.irq(trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING, handler = RF_D3_callback)
#"""


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

#want to add one for half pwr and then ramming speed

FULL_PWR = 65535  #65535 IS FULL SPEED (100% DUTY)

FORWARD_PWR = 30000

BACK_LEFT = 25000

BACK_RIGHT = 25000

low_pwr = 6553                       #<---------- THIS NEEDS ADJUSTMENT FOR FINAL BOT WEIGHT
#low_pwr = external_knob.read_u16()

wdt.feed()

### VIA TESTING FOUND ~6553 TO BE THE LOWER BOUND FOR PWM ON THE BULKY TEST SET UP
### ACTUAL BOT THIS VALUE WILL BE DIFFERENT

pwm_full = min(max(int(2**16 * abs(1)), 0), FULL_PWR)   
  
pwm_fwd = min(max(int(2**16 * abs(1)), 0), FORWARD_PWR)   

pwm_back_left = min(max(int(2**16 * abs(1)), 0), BACK_LEFT)

pwm_back_right = min(max(int(2**16 * abs(1)), 0), BACK_RIGHT)

pwm_slow = min(max(int(2**16 * abs(1)), 0), low_pwr) # <---- toggle the slower drive setting

pwm_adjust = min(max(int(2**16 * abs(1)), 0), low_pwr) # <---- toggle the slower drive setting

wdt.feed()

def turn_adjust(turning_code):
    global low_pwr, pwm_adjust
    PARTITIONS = 9 # since we use 0-9 in the hex number for turning instructions
    interval_size = (FORWARD_PWR - low_pwr) / 9
    turning_pwm = int(FORWARD_PWR - (turning_code * interval_size))
    pwm_adjust = min(max(int(2**16 * abs(1)), 0), turning_pwm) # <---- toggle the slower drive setting
    print(f"pwm adjust: {turning_pwm}")


def spin_adjust(turning_code):
    global low_pwr, pwm_adjust
    PARTITIONS = 9 # since we use 0-9 in the hex number for turning instructions
    interval_size = (FORWARD_PWR - low_pwr) / 9
    turning_pwm = int(low_pwr + (turning_code * interval_size))
    pwm_adjust = min(max(int(2**16 * abs(1)), 0), turning_pwm) # <---- toggle the slower drive setting
    print(f"pwm adjust: {turning_pwm}")


# SET INITIAL DIRECTION OF MOTORS TO BE FORWARD       # .low() is FORWARD     # .high() is BACKWARD
ain1_ph.low()           # RIGHT WHEEL
bin1_ph.low()           # 

#0x97
def power_push(data):
    ain2_en.duty_u16(0)     # STOPS THE MOTORS
    bin2_en.duty_u16(0)
    ain1_ph.high()           # SET MOTOR DIRECTION FORWARD
    bin1_ph.high()
    ain2_en.duty_u16(pwm_full)   # DRIVES THE MOTORS
    bin2_en.duty_u16(pwm_full)

def full_forward(data):
    #print(f"    FULL FORWARD {data}")
    #adjust_pwm_slow()       #update the amount of slow per the external knob's adc value
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()           # SET MOTOR DIRECTION FORWARD
    bin1_ph.high()
    ain2_en.duty_u16(pwm_fwd)   # DRIVES THE MOTORS
    bin2_en.duty_u16(pwm_fwd)

def forward_left(turning_code):
    #print("    FWD/LEFT")
    turn_adjust(turning_code)      
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()           # SET MOTOR DIRECTION FORWARD
    bin1_ph.high()
    ain2_en.duty_u16(pwm_fwd)   # DRIVES THE MOTORS
    bin2_en.duty_u16(pwm_adjust)

def forward_right(turning_code):
    #print("    FWD/RIGHT")
    turn_adjust(turning_code)        
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()           # SET MOTOR DIRECTION FORWARD
    bin1_ph.high()
    bin2_en.duty_u16(pwm_fwd)
    ain2_en.duty_u16(pwm_adjust)   # DRIVES THE MOTORS

def full_backward(data):
    #print(f"    FULL BACKWARD{data}")
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()          # SET MOTOR DIRECTION BACKWARD
    bin1_ph.low()
    ain2_en.duty_u16(pwm_back_right)   # DRIVES THE MOTORS
    bin2_en.duty_u16(pwm_back_left)

def backward_left(turning_code):
    #print("    BACK/LEFT")
    turn_adjust(turning_code)            
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()          # LEFT WHEEL BACKWARD
    bin1_ph.low()           # RIGHT WHEEL FORWARD
    ain2_en.duty_u16(pwm_fwd)   # DRIVES THE MOTORS
    bin2_en.duty_u16(pwm_adjust)

def backward_right(turning_code):
    #print("    BACK/RIGHT")
    turn_adjust(turning_code)           
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()          # LEFT WHEEL BACKWARD
    bin1_ph.low()           # RIGHT WHEEL FORWARD
    bin2_en.duty_u16(pwm_fwd)
    ain2_en.duty_u16(pwm_adjust)   # DRIVES THE MOTORS

def spin_left(turning_code): #turning code will determine how fast of a spin to do
    #print("    SPIN LEFT")
    spin_adjust(turning_code)            
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()         # RIGHT WHEEL FORWARD 
    bin1_ph.low()           # LEFT WHEEL BACKWARD
    bin2_en.duty_u16(pwm_adjust)
    ain2_en.duty_u16(pwm_adjust)
    
def spin_right(turning_code): #turning code will determine how fast of a spin to do
    #print("    SPIN/RIGHT")
    spin_adjust(turning_code)
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()           # RIGHT WHEEL BACKWARD
    bin1_ph.high()          # LEFT WHEEL FORWARD
    ain2_en.duty_u16(pwm_adjust)
    bin2_en.duty_u16(pwm_adjust)

def motors_stop(data):
    #print(f"    motors stop{data}")
    #flash_board_led(1)        
    ain2_en.duty_u16(0)     # STOPS THE MOTORS
    bin2_en.duty_u16(0)

wdt.feed() 

#testline
#motors_stop(0x00) # incase of restart or brownout, issue a motor_stop command on restart


#========================================================================================================#
# IR RECEIVER SETUP AND FUNCTIONS
#========================================================================================================#

#data input from IR
ir_pin = Pin(18, Pin.IN, Pin.PULL_UP)

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
            schedule(motors_stop, data)
            wdt.feed()
        elif(data == 0x97):
            print("rx'd powerpush command!")
            schedule(power_push, data)
            wdt.feed()
        elif(data == 0x33):
            schedule(full_forward, data)
            wdt.feed()
        elif(data == 0x88):
            schedule(full_backward, data)
            wdt.feed()
        elif(0x3f < data < 0x50):
            schedule(forward_left, turning_code(data))
            wdt.feed()
        elif(0x4f < data < 0x60):
            schedule(forward_right, turning_code(data))
            wdt.feed()
        elif(0x5f < data < 0x70):
            schedule(backward_left, turning_code(data))
            wdt.feed()
        elif(0x6f < data < 0x80):
            schedule(backward_right, turning_code(data))
            wdt.feed()
        elif(0x0f < data < 0x20):
            schedule(spin_left, turning_code(data))
            wdt.feed()
        elif(0x1f < data < 0x30):
            schedule(spin_right, turning_code(data))
            wdt.feed()
       
       
ir_receiver = NEC_8(ir_pin, callback=ir_callback)

wdt.feed()

#========================================================================================================#
# TOGGLE COMMS PUSHBUTTON SETUP AND FUNCTIONS
#========================================================================================================#

push_button = Pin(16, Pin.IN, Pin.PULL_UP)    # BUTTON IS CONFIGURED IN PULL-UP TO 3.3V 

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

wdt.feed()

wdt_loop = False   # IF RF IS ON WE FEED DOG CONTINUOUSLY IN MAIN. WHEN OFF, IR IS FEEDING DOG VIA RX SIGNALS

# Optional: Use the print_error function for debugging
# Note: Causes "INVALID START PULSE" WHEN TOGGLING BETWEEN THE IR/RF RECEIVER MODES
ir_receiver.error_function(print_error)        # USE FOR TROUBLESHOOTING IF NEEDED

def switch_receiver():
    global ir_mode, ir_receiver, wdt_loop
    print("Switching Receiver Mode!")
    motors_stop(0x00)
    if (ir_mode == True):
        print("~RF RECEIVER ACTIVE~")
        wdt_loop = True
        ir_mode = False
        turn_off_ir()                                                              
        enable_rf_irq()             # TURN ON THE RF INTERRUPTS
        wdt.feed()
        flash_board_led(4)          # Flash onboard LED to signal execution
    elif (ir_mode == False):
        print("~IR RECEIVER ACTIVE~")
        wdt_loop = False
        ir_mode = True
        ir_receiver = NEC_8(ir_pin, callback=ir_callback)
        disable_rf_irq()            # TURN OFF THE RF INTERRUPTS
        wdt.feed()
        flash_board_led(4)          # Flash onboard LED to signal execution
   


#========================================================================================================#
# MAIN SETUP AND FUNCTIONS
#========================================================================================================#

wdt.feed()

time.sleep(.5) # 1 second delay before main loop
print("Goliath online!")

wdt.feed()

#flash lights to signal the main loop is starting
flash_board_led(4)

wdt.feed()

def main():
    global wdt_loop
    while True:

        if wdt_loop == True:
            wdt.feed()


        pass

if __name__ == "__main__":
    main()
