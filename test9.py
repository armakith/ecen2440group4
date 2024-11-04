import math, time
import machine
# load the MicroPython pulse-width-modulation module for driving hardware
from machine import PWM
from machine import Pin

from machine import I2C
import seesaw

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

# Initialize I2C. Adjust pin numbers based on your Pico's configuration
i2c = I2C(1, scl=Pin(27), sda=Pin(26))

# Initialize the Seesaw driver with the I2C interface
# Use the Gamepad QT's I2C address from the Arduino code (0x50)
seesaw_device = seesaw.Seesaw(i2c, addr=0x50)

# Initialize joystick center position
joystick_center_x = 511
joystick_center_y = 497

# motor settings
pwm_rate = 2000   #ORIG:    2000
ain1_ph = Pin(12, Pin.OUT)                                  # MOTOR CHANNEL A 
ain2_en = PWM(13, freq = pwm_rate, duty_u16 = 0)
bin1_ph = Pin(14, Pin.OUT)                                  # MOTOR CHANNEL B
bin2_en = PWM(15, freq = pwm_rate, duty_u16 = 0)
pwm = min(max(int(2**16 * abs(1)), 0), 65535)   ##Orig: 65535

# INITIAL DIRECTION OF MOTORS DRIVE DIRECTION       # .low() is      # .high() is 
ain1_ph.low()
bin1_ph.low()

def motors_stop():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)

def joystick_up():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()
    bin1_ph.low()
    ain2_en.duty_u16(pwm)
    bin2_en.duty_u16(pwm)
    #forward

def joystick_down():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()
    bin1_ph.high()
    ain2_en.duty_u16(pwm)
    bin2_en.duty_u16(pwm)
    #backward

def joystick_left():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.low()
    bin1_ph.high()
    ain2_en.duty_u16(pwm)
    bin2_en.duty_u16(pwm)
    #left back right forward

def joystick_right():
    ain2_en.duty_u16(0)
    bin2_en.duty_u16(0)
    ain1_ph.high()
    bin1_ph.low()
    ain2_en.duty_u16(pwm)
    bin2_en.duty_u16(pwm)
    #left fwd right back

def main():
   
   last_x, last_y = seesaw_device.analog_read(JOYSTICK_X_PIN), seesaw_device.analog_read(JOYSTICK_Y_PIN)
   
   # Adjust threshold as needed
   joystick_threshold = 50  

   while True:
       
       # Read joystick values
       current_x = seesaw_device.analog_read(JOYSTICK_X_PIN)
       current_y = seesaw_device.analog_read(JOYSTICK_Y_PIN)

       # Check if joystick position has changed significantly
       if abs(current_x - last_x) > joystick_threshold or abs(current_y - last_y) > joystick_threshold:
           
           print("Joystick moved - X:", current_x, ", Y:", current_y)
           last_x, last_y = current_x, current_y

           # JOYSTICK
           # Determine which LED to turn on based on joystick direction
           if current_y < joystick_center_y - joystick_threshold:  # Joystick moved up
               #set_led(Pin(LED_1_PIN, Pin.OUT), True)
               joystick_up()
           elif current_y > joystick_center_y + joystick_threshold:  # Joystick moved down
               #set_led(Pin(LED_2_PIN, Pin.OUT), True)
               joystick_down()
           elif current_x < joystick_center_x - joystick_threshold:  # Joystick moved left
               #set_led(Pin(LED_4_PIN, Pin.OUT), True)
               joystick_left()
           elif current_x > joystick_center_x + joystick_threshold:  # Joystick moved right
               #set_led(Pin(LED_3_PIN, Pin.OUT), True)
               joystick_right()
           else:
               motors_stop()

       time.sleep(0.1)  # Delay to prevent overwhelming the output

if __name__ == "__main__":
   main()
