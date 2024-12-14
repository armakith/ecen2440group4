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

wdt = WDT(timeout=500) # .5 seconds

# IR FEEDS WATCHDOG WHEN RECEIVING COMMAND, SO WATCHDOG WILL RESET IF IT GETS STUCK IN ANOTHER COMMAND
# OR GOES OUT OF RANGE OF THE IR CONTROLLER

# RF DOESNT HAVE LOCK UP PROBLEMS SO WE ENABLE FEEDING THE DOG IN MAIN WHEN BOT IS IN RF MODE

wdt.feed()

#========================================================================================================#
# LED SETUP AND FUNCTIONS
#========================================================================================================#

BOARD_LED_PIN = 25
board_led = Pin(BOARD_LED_PIN, Pin.OUT)

def flash_board_led(x_times):
    global wdt
    for i in range(x_times):
        wdt.feed()
        board_led.value(1)
        time.sleep(0.1)
        board_led.value(0)
        time.sleep(0.1)
    wdt.feed()

wdt.feed()

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

FULL_PWR = 50000  #65535 IS FULL SPEED (100% DUTY)

FORWARD_PWR = 30000

BACK_LEFT = 30000

BACK_RIGHT = 30000

low_pwr = 6553                       #<---------- THIS NEEDS ADJUSTMENT FOR FINAL BOT WEIGHT

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
    global low_pwr, pwm_adjust, wdt
    wdt.feed()
    PARTITIONS = 9 # since we use 0-9 in the hex number for turning instructions
    interval_size = (FORWARD_PWR - low_pwr) / 9
    turning_pwm = int(FORWARD_PWR - (turning_code * interval_size))
    pwm_adjust = min(max(int(2**16 * abs(1)), 0), turning_pwm) # <---- toggle the slower drive setting
    print(f"pwm adjust: {turning_pwm}")

def spin_adjust(turning_code):
    global low_pwr, pwm_adjust, wdt
    wdt.feed()
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
    ain2_en.duty_u16(0)     # STOPS THE MOTORS
    bin2_en.duty_u16(0)

wdt.feed() 

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
            wdt.feed()
            schedule(motors_stop, data)
        elif(data == 0x97):
            wdt.feed()
            schedule(power_push, data)
        elif(data == 0x33):
            wdt.feed()
            schedule(full_forward, data)
        elif(data == 0x88):
            wdt.feed()
            schedule(full_backward, data)
        elif(0x3f < data < 0x50):
            wdt.feed()
            schedule(forward_left, turning_code(data))
        elif(0x4f < data < 0x60):
            wdt.feed()
            schedule(forward_right, turning_code(data))
        elif(0x5f < data < 0x70):
            wdt.feed()
            schedule(backward_left, turning_code(data))
        elif(0x6f < data < 0x80):
            wdt.feed()
            schedule(backward_right, turning_code(data))
        elif(0x0f < data < 0x20):
            wdt.feed()
            schedule(spin_left, turning_code(data))
        elif(0x1f < data < 0x30):
            wdt.feed()
            schedule(spin_right, turning_code(data))
            
        
ir_receiver = NEC_8(ir_pin, callback=ir_callback)

wdt.feed()

#========================================================================================================#
# RF RECEIVER SETUP AND FUNCTIONS
#========================================================================================================#

RF_FORWARD = 30000
RF_BACKWARD = 30000
RF_LEFT = 30000
RF_RIGHT = 30000

wdt.feed()

rf_fwd_pwm = min(max(int(2**16 * abs(1)), 0), RF_FORWARD)   
rf_back_pwm = min(max(int(2**16 * abs(1)), 0), RF_BACKWARD)
rf_left_pwm = min(max(int(2**16 * abs(1)), 0), RF_LEFT)
rf_right_pwm = min(max(int(2**16 * abs(1)), 0), RF_RIGHT)

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

D0_PIN = 7     # MAPS TO [D] BUTTON ON CONTROLLER    
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

#  RF HAS SEPERATE DRIVING FUNCTIONS THAN IR

def rf_forward():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()          # SET MOTOR DIRECTION BACKWARD
    bin1_ph.high()
    ain2_en.duty_u16(rf_fwd_pwm)   # DRIVES THE MOTORS
    bin2_en.duty_u16(rf_fwd_pwm)
    
def rf_backward():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()          # SET MOTOR DIRECTION BACKWARD
    bin1_ph.low()
    ain2_en.duty_u16(rf_back_pwm)   # DRIVES THE MOTORS
    bin2_en.duty_u16(rf_back_pwm)

def rf_left():     
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()         # RIGHT WHEEL FORWARD 
    bin1_ph.low()           # LEFT WHEEL BACKWARD
    bin2_en.duty_u16(rf_left_pwm)
    ain2_en.duty_u16(rf_left_pwm)
    
def rf_right():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()           # RIGHT WHEEL BACKWARD
    bin1_ph.high()          # LEFT WHEEL FORWARD
    ain2_en.duty_u16(rf_right_pwm)
    bin2_en.duty_u16(rf_right_pwm)

def rf_stop():      
    ain2_en.duty_u16(0)     # STOPS THE MOTORS
    bin2_en.duty_u16(0)

def RF_D0_callback(pin):
    disable_rf_irq()  # when callback is activated, disable all RF Handlers
    if D0_RF_INPUT.value() == 1:
        #print("BACKWARD")
        full_backward(0x88)
    elif D0_RF_INPUT.value() == 0:
        #print("STOP")
        rf_stop()
    enable_rf_irq()
        
def RF_D1_callback(pin):
    disable_rf_irq()  # when callback is activated, disable all RF Handlers
    if D1_RF_INPUT.value() == 1:
        #print("SPIN_LEFT")
        rf_left()
    elif D1_RF_INPUT.value() == 0:
        #print("STOP")
        rf_stop()
    enable_rf_irq()
    
def RF_D2_callback(pin):
    disable_rf_irq()  # when callback is activated, disable all RF Handlers
    if D2_RF_INPUT.value() == 1:
        #print("SPIN_RIGHT")
        rf_right()
    elif D2_RF_INPUT.value() == 0:
        #print("STOP")
        rf_stop()
    enable_rf_irq()

def RF_D3_callback(pin):
    disable_rf_irq()  # when callback is activated, disable all RF Handlers
    if D3_RF_INPUT.value() == 1:
        #print("FORWARD")
        rf_forward()
    elif D3_RF_INPUT.value() == 0:
        #print("STOP")
        rf_stop()
    enable_rf_irq()

wdt.feed()

#D0_RF_INPUT.irq(trigger=Pin.IRQ_RISING, handler = RF_D0_callback)
#D1_RF_INPUT.irq(trigger=Pin.IRQ_RISING, handler = RF_D1_callback)
#D2_RF_INPUT.irq(trigger=Pin.IRQ_RISING, handler = RF_D2_callback)
#D3_RF_INPUT.irq(trigger=Pin.IRQ_RISING, handler = RF_D3_callback)

D0_RF_INPUT.irq(trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING, handler = RF_D0_callback)
D1_RF_INPUT.irq(trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING, handler = RF_D1_callback)
D2_RF_INPUT.irq(trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING, handler = RF_D2_callback)
D3_RF_INPUT.irq(trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING, handler = RF_D3_callback)

#========================================================================================================#
# TOGGLE COMMS PUSHBUTTON SETUP AND FUNCTIONS
#========================================================================================================#

push_button = Pin(16, Pin.IN, Pin.PULL_UP)    # BUTTON IS CONFIGURED IN PULL-UP TO 3.3V 

# button debounce to deal with button bounce signals
debounce_time = 0
# Callback function to execute when an IR code is received
def push_button_callback(push_button):
    global debounce_time, wdt
    if (time.ticks_ms()-debounce_time) > 500:
        wdt.feed()
        switch_receiver()
        debounce_time=time.ticks_ms()
    
push_button.irq(trigger=Pin.IRQ_FALLING, handler=push_button_callback)

# WE ARE HAVING THE IR RECEIVER ON BY DEFAULT, SO TURN THE RF INPUT INTERRUPT HANDLERS OFF
# so disable the RF interrupt handlers
disable_rf_irq()

def turn_off_ir():
    global wdt
    wdt.feed() 
    ir_receiver.close()
    time.sleep(0.1)

global ir_mode 
ir_mode = True   # False means comms are set to RF Receiver instead

wdt.feed()

wdt_loop = False   # IF RF IS ON WE FEED DOG CONTINUOUSLY IN MAIN. WHEN OFF, IR IS FEEDING DOG VIA RX SIGNALS

# Optional: Use the print_error function for debugging
# Note: Causes "INVALID START PULSE" WHEN TOGGLING BETWEEN THE IR/RF RECEIVER MODES
ir_receiver.error_function(print_error)        # USE FOR TROUBLESHOOTING IF NEEDED

def switch_receiver():
    global ir_mode, ir_receiver, wdt_loop
    wdt.feed()
    print("Switching Receiver Mode!")
    motors_stop(0x00)
    if (ir_mode == True):
        print("~RF RECEIVER ACTIVE~")
        wdt_loop = True
        ir_mode = False
        wdt.feed() 
        turn_off_ir()    
        wdt.feed()                                                           
        enable_rf_irq()             # TURN ON THE RF INTERRUPTS
        wdt.feed()
        flash_board_led(1)          # Flash onboard LED to signal execution
    elif (ir_mode == False):
        print("~IR RECEIVER ACTIVE~")
        wdt_loop = False
        ir_mode = True
        wdt.feed() 
        ir_receiver = NEC_8(ir_pin, callback=ir_callback)
        wdt.feed() 
        disable_rf_irq()            # TURN OFF THE RF INTERRUPTS
        wdt.feed()
        flash_board_led(1)          # Flash onboard LED to signal execution
   
#========================================================================================================#
# MAIN SETUP AND FUNCTIONS
#========================================================================================================#

wdt.feed()

time.sleep(.1) # .1 second delay before main loop
print("Goliath online!")

wdt.feed()

#flash lights to signal the main loop is starting
flash_board_led(4)

wdt.feed()

def main():
    global wdt_loop, drive_forward
    while True:

        if wdt_loop == True: #enabled when RF is chosen as comms
            pass
            wdt.feed()
            
        pass

if __name__ == "__main__":
    main()
