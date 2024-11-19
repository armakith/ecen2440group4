import time
from ir_tx.nec import NEC
from machine import Pin, I2C
import seesaw

#======================================================================================================#
# SETUP JOYSTICK
#======================================================================================================#

# Initialize I2C. Adjust pin numbers based on your Pico's configuration
i2c = I2C(1, scl=Pin(27), sda=Pin(26))
#         ^ this refers to the I2C bus. See Pico pinout diag.

# Initialize the Seesaw driver with the I2C interface
# Use the Gamepad QT's I2C address from the Arduino code (0x50)
seesaw_device = seesaw.Seesaw(i2c, addr=0x50)

JOYSTICK_X_PIN = 14
JOYSTICK_Y_PIN = 15

# Initialize joystick center position
#joystick_center_x = 511
#joystick_center_y = 497



#======================================================================================================#
# SETUP IR TRANSMITTER
#======================================================================================================#

tx_pin = Pin(16,Pin.OUT,value=0)

device_addr = 0x61

transmitter = NEC(tx_pin)

commands = [0x01,0x02,0x03,0x04,0x05]

#   IR TRANSMITTER MAPPING TO SUMOBOT COMMANDS
#
#   IR COMMAND:             SUMOBOT INSTRUCTION:
#
#   0x01                    move forward
#   0x02                    move backward
#   0x03                    spin left
#   0x04                    spin right
#   0x05                    stop motors

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
# Main program loop #
#======================================================================================================#

time.sleep(1)                 # delay for usb
print("IR Beacon Online!")
flash_onboard_led(4)

FULL_FWD = 10
FULL_BACK = 1010

SPIN_LEFT = 1010
SPIN_RIGHT = 10

# Initialize joystick center position
j_ctr_x = seesaw_device.analog_read(JOYSTICK_X_PIN)
j_ctr_y = seesaw_device.analog_read(JOYSTICK_Y_PIN)

# centering can drift. so center it to what it is sitting at on initialization
# WHEN TURNING ON CONTROLLER, DONT TOUCH THE JOYSTICK!!!
print("Initializing:")
print(f"joystick center x: {j_ctr_x}")
print(f"joystick center y: {j_ctr_y}")

def main():
    
    last_x, last_y = seesaw_device.analog_read(JOYSTICK_X_PIN), seesaw_device.analog_read(JOYSTICK_Y_PIN)

    # Adjust threshold as needed
    j_thresh = 50

    while True:
      
      # Read joystick values
      current_x = seesaw_device.analog_read(JOYSTICK_X_PIN)
      current_y = seesaw_device.analog_read(JOYSTICK_Y_PIN)

      #use to test output of joystick x,y
      #print(f"x,y: {current_x}, {current_y} ")

      #"""
      # Check if joystick position has changed significantly
      if abs(current_x - last_x) >  j_thresh or abs(current_y - last_y) >  j_thresh:
        #print("Joystick moved - X:", current_x, ", Y:", current_y)   # use for troubleshooting if needed
        last_x, last_y = current_x, current_y
      
        if (FULL_FWD < current_y < j_ctr_y -  j_thresh):            # joystick fwd
          transmitter.transmit(device_addr, 0x11)
          print(f"J_fwd: {current_x}, {current_y}")
          time.sleep(0.05)
        elif (FULL_FWD > current_y) and (600 > current_x > 400):    # FULL FWD
          transmitter.transmit(device_addr, 0x17)
          print(f"FULL_FWD: {current_x}, {current_y}")
          time.sleep(0.05)
        elif (FULL_FWD > current_y) and (current_x < 400):          # FULL VEER LEFT
          transmitter.transmit(device_addr, 0x1f)
          print(f"FULL_LEFT: {current_x}, {current_y}")
          time.sleep(0.05)
        elif (FULL_FWD > current_y) and (600 < current_x):          # FULL VEER RIGHT
          transmitter.transmit(device_addr, 0x2b)
          print(f"FULL_RIGHT: {current_x}, {current_y}")
          time.sleep(0.05)
        elif (FULL_BACK > current_y > j_ctr_y +  j_thresh):         # joystick back
          transmitter.transmit(device_addr, 0x35)
          print(f"J_back: {current_x}, {current_y}")
          time.sleep(0.05)
        elif FULL_BACK < current_y:                                 # FULL BACK
          transmitter.transmit(device_addr, 0x3b)
          print(f"FULL_BACK: {current_x}, {current_y}")
          time.sleep(0.05)
        elif SPIN_LEFT > current_x > j_ctr_x +  j_thresh:           # Joystick moved LEFT
          transmitter.transmit(device_addr, 0x59)
          print(f"J_left: {current_x}, {current_y}")
          time.sleep(0.05)
        elif SPIN_LEFT < current_x:                                 # SPIN LEFT
          transmitter.transmit(device_addr, 0x6b)
          print(f"SPIN_LEFT: {current_x}, {current_y}")
          time.sleep(0.05)
        elif SPIN_RIGHT < current_x < j_ctr_x -  j_thresh:          # Joystick moved RIGHT
          transmitter.transmit(device_addr, 0x71)
          print(f"J_right: {current_x}, {current_y}")
          time.sleep(0.05)  
        elif SPIN_RIGHT > current_x:                                # SPIN RIGHT
          transmitter.transmit(device_addr, 0xa7)
          print(f"SPIN_RIGHT: {current_x}, {current_y}")
          time.sleep(0.05)
        else:                                                       # Joystick is "centered" aka not being used, execute motor_stop()
          transmitter.transmit(device_addr, 0xbf)
          print(f"    MOTORS STOP: {current_x}, {current_y}")
          time.sleep(0.05) 
      #"""
      


      time.sleep(0.1)  # Delay to prevent overwhelming the output

if __name__ == "__main__":
   main()
