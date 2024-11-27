import time
from ir_tx.nec import NEC
from machine import Pin, I2C, ADC
import seesaw


#======================================================================================================#
# SETUP ON BOARD LED 
#======================================================================================================#

onboard_led = Pin(25, Pin.OUT) # this is the onboard led, using to visually show when the board transmits

def flash_onboard_led(x_times):
  for i in range(x_times):
    onboard_led.value(1)
    time.sleep(0.1)
    onboard_led.value(0)
    time.sleep(0.1)



#======================================================================================================#
# SETUP JOYSTICK
#======================================================================================================#

# Initialize I2C. Adjust pin numbers based on your Pico's configuration
i2c = I2C(1, scl=Pin(7), sda=Pin(6))    #this was originally 27 and 26
#         ^ this refers to the I2C bus. See Pico pinout diag.

# Initialize the Seesaw driver with the I2C interface
# Use the Gamepad QT's I2C address from the Arduino code (0x50)
seesaw_device = seesaw.Seesaw(i2c, addr=0x50)

JOYSTICK_X_PIN = 14
JOYSTICK_Y_PIN = 15                                         # DISABLING Y-AXIS


# Initialize joystick center position
j_ctr_x = seesaw_device.analog_read(JOYSTICK_X_PIN)
#j_ctr_y = seesaw_device.analog_read(JOYSTICK_Y_PIN)         # DISABLING Y-AXIS

# centering can drift. so center it to what it is sitting at on initialization
# WHEN TURNING ON CONTROLLER, DONT TOUCH THE JOYSTICK!!!
print("Initializing:")
print(f"joystick center x: {j_ctr_x}")
#print(f"joystick center y: {j_ctr_y}")                       # DISABLING Y-AXIS


#======================================================================================================#
# SETUP BIG JOYSTICK
#======================================================================================================#
#"""
B_STICK_X_PIN = 26    # ADC 0
B_STICK_Y_PIN = 27    # ADC 1                                # DISABLING Y-AXIS

b_stick_x = ADC(B_STICK_X_PIN)
b_stick_y = ADC(B_STICK_Y_PIN)                               # DISABLING Y-AXIS
#"""
#======================================================================================================#
# SETUP BUTTONS
#======================================================================================================#

# Define button and joystick pin numbers as per the Arduino code
BUTTON_A = 5
BUTTON_B = 1
BUTTON_X = 6
BUTTON_Y = 2
BUTTON_START = 16
BUTTON_SELECT = 0


# Button mask based on Arduino code
BUTTONS_MASK = (1 << BUTTON_X) | (1 << BUTTON_Y) | \
              (1 << BUTTON_A) | (1 << BUTTON_B) | \
              (1 << BUTTON_SELECT) | (1 << BUTTON_START)

# Initialize last button states
last_buttons = 0

def setup_buttons():
  """Configure the pin modes for buttons."""
  seesaw_device.pin_mode_bulk(BUTTONS_MASK, seesaw_device.INPUT_PULLUP)

def read_buttons():
  """Read and return the state of each button."""
  return seesaw_device.digital_read_bulk(BUTTONS_MASK)

def handle_button_press(button):
  
  if button == BUTTON_A:
    print("BUTTON A PRESSED")
  elif button == BUTTON_B:
    print("BUTTON B PRESSED")
  elif button == BUTTON_X:
    print("BUTTON X PRESSED ")
  elif button == BUTTON_Y:
    print("BUTTON Y PRESSED")
  elif button == BUTTON_START:
    print("START BUTTON PRESSED")
  elif button == BUTTON_SELECT:
    print("SELECT BUTTON PRESSED")




#======================================================================================================#
# SETUP IR TRANSMITTER
#======================================================================================================#

tx_pin = Pin(16,Pin.OUT,value=0)

device_addr = 0x61

transmitter = NEC(tx_pin)

#======================================================================================================#
# STEERING FUNCTIONS
#======================================================================================================#

global current_x  #, current_y
current_x = seesaw_device.analog_read(JOYSTICK_X_PIN)
#current_y = seesaw_device.analog_read(JOYSTICK_Y_PIN)    #disabling the y-axis

# Adjust threshold as needed
global j_thresh
j_thresh = 10    # ORIG VALUE = 50

global divisions
divisions = 10

def turn_right(current_x):
  threshold = j_ctr_x - j_thresh
  interval = threshold/divisions
  hex_code = int((threshold-current_x)/(interval+2))
  return hex_code
  
def turn_left(current_x):
  threshold = j_ctr_x + j_thresh
  interval = (1023-threshold)/divisions
  hex_code = int((current_x-threshold)/(interval+2))
  return hex_code



def encode_cmd(direction, hex_code):
  command = 0x00
  if direction == "fwr":
    command = 0x50
  elif direction == "bwr":
    command = 0x70
  elif direction == "spr":
    command = 0x20
  elif direction == "fwl":
    command = 0x40
  elif direction == "bwl":
    command = 0x60
  elif direction == "spl":
    command = 0x10
  turn_code = hex_code
  command_word = command + turn_code
  print(f"    encoded: {hex(command_word)}")
  return command_word


def forward_drive(current_x):
  if abs(j_ctr_x - current_x) < j_thresh:
    print("DRIVE FORWARD")
    #transmit full forward command 0x33
    transmitter.transmit(device_addr, 0x33)
  elif abs(j_ctr_x - current_x) > j_thresh and current_x < j_ctr_x:
    print("FORWARD RIGHT")
    encoded = encode_cmd("fwr", turn_right(current_x))
    #transmit fwd/right command 0x5_
    transmitter.transmit(device_addr, encoded)
  elif abs(j_ctr_x - current_x) > j_thresh and current_x > j_ctr_x:
    print("FORWARD LEFT")
    encoded = encode_cmd("fwl", turn_left(current_x))
    #transmit fwd/left command 0x4_
    transmitter.transmit(device_addr, encoded)

def backward_drive(current_x):
  if abs(j_ctr_x - current_x) < j_thresh:
    print("DRIVE BACKWARD")
    #transmit full backward command 0x88
    transmitter.transmit(device_addr, 0x88)
  elif abs(j_ctr_x - current_x) > j_thresh and current_x < j_ctr_x:
    print("BACKWARD RIGHT")
    encoded = encode_cmd("bwr", turn_right(current_x))
    #transmit fwd/right command 0x7_
    transmitter.transmit(device_addr, encoded)
  elif abs(j_ctr_x - current_x) > j_thresh and current_x > j_ctr_x:
    print("BACKWARD LEFT")
    encoded = encode_cmd("bwl", turn_left(current_x))
    #transmit fwd/left command 0x6_
    transmitter.transmit(device_addr, encoded)

def spin_lr(current_x):
  if abs(j_ctr_x - current_x) > j_thresh and current_x > j_ctr_x:
    print("SPIN LEFT")
    encoded = encode_cmd("spl", turn_left(current_x))
    #transmit spin left command 0x1_
    transmitter.transmit(device_addr, encoded)
  elif abs(j_ctr_x - current_x) > j_thresh and current_x < j_ctr_x:
    print("SPIN RIGHT")
    encoded = encode_cmd("spr", turn_right(current_x))
    #transmit spin right command 0x2_
    transmitter.transmit(device_addr, encoded)

def spin_right(current_x):
  if abs(j_ctr_x - current_x) > j_thresh and current_x < j_ctr_x:
    print("SPIN RIGHT")
    encoded = encode_cmd("spr", turn_right(current_x))
    #transmit spin right command 0x2_
    transmitter.transmit(device_addr, encoded)


def stop_driving():
  transmitter.transmit(device_addr, 0x00)


# Read joystick values




#======================================================================================================#
# Main program loop #
#======================================================================================================#

time.sleep(1)                 # delay for usb
print("IR Beacon Online!")
flash_onboard_led(4)





def main():

    global last_buttons  # Ensure last_buttons is recognized as a global variable

    setup_buttons()

    
    last_x = seesaw_device.analog_read(JOYSTICK_X_PIN)
    #last_y = seesaw_device.analog_read(JOYSTICK_Y_PIN)          # DISABLING Y-AXIS
    
    

    while True:
      
      """
      # B_stick values:
      x_axis = b_stick_x.read_u16()
      y_axis = b_stick_y.read_u16()                        # DISABLING Y-AXIS
      #print(f"                x-axis: {x_axis}")
      print(f"x-axis: {x_axis}, y-axis: {y_axis}")         # DISABLING Y-AXIS

      time.sleep(0.1)  # Delay to prevent overwhelming the output (DEFAULT 0.1)
      """


      #""" 
      # Read joystick values
      current_x = seesaw_device.analog_read(JOYSTICK_X_PIN)
      #current_y = seesaw_device.analog_read(JOYSTICK_Y_PIN)    #disabling the y axis

      #use to test output of joystick x,y
      #print(f"x,y: {current_x}, {current_y} ")

      buttons = read_buttons()

      if not buttons & (1 << BUTTON_X):           # FORWARD
        print(f"steering pos: {current_x}")
        forward_drive(current_x)
      elif not buttons & (1 << BUTTON_A):         # SPIN LEFT/RIGHT
        print("SPIN LEFT/RIGHT")
        spin_lr(current_x)
      elif not buttons & (1 << BUTTON_B):         # REVERSE
        print(f"steering pos: {current_x}")
        backward_drive(current_x)
      else:                                       # always send stop commands when not in use
        print("stop driving()")
        stop_driving()

      time.sleep(0.1)  # Delay to prevent overwhelming the output (DEFAULT 0.1)
      
      #"""

if __name__ == "__main__":
   main()
