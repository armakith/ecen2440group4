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
joystick_center_x = 511
joystick_center_y = 497

#======================================================================================================#
# SETUP IR TRANSMITTER
#======================================================================================================#

tx_pin = Pin(16,Pin.OUT,value=0)
device_addr = 0x01
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
      
def main():
    
    last_x, last_y = seesaw_device.analog_read(JOYSTICK_X_PIN), seesaw_device.analog_read(JOYSTICK_Y_PIN)

    # Adjust threshold as needed
    joystick_threshold = 300 

    while True:

      # Read joystick values
      current_x = seesaw_device.analog_read(JOYSTICK_X_PIN)
      current_y = seesaw_device.analog_read(JOYSTICK_Y_PIN)

      # Check if joystick position has changed significantly
      if abs(current_x - last_x) > joystick_threshold or abs(current_y - last_y) > joystick_threshold:
        #print("Joystick moved - X:", current_x, ", Y:", current_y)   # use for troubleshooting if needed
        last_x, last_y = current_x, current_y
      
        if current_y < joystick_center_y - joystick_threshold:    # Joystick moved up
          transmitter.transmit(device_addr, 0x01)
          time.sleep(0.1)
        elif current_y > joystick_center_y + joystick_threshold:  # Joystick moved down
          transmitter.transmit(device_addr, 0x02)
          time.sleep(0.1)
        elif current_x > joystick_center_x + joystick_threshold:  # Joystick moved LEFT
          transmitter.transmit(device_addr, 0x03)
          time.sleep(0.1)
        elif current_x < joystick_center_x - joystick_threshold:  # Joystick moved RIGHT
          transmitter.transmit(device_addr, 0x04)
          time.sleep(0.1)
        else:                                                     # Joystick is "centered" aka not being used, execute motor_stop()
          transmitter.transmit(device_addr, 0x05)
          time.sleep(0.1)
        
      time.sleep(0.1)  # Delay to prevent overwhelming the output

if __name__ == "__main__":
   main()
