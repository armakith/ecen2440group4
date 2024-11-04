# ECEN2440 Group 4

READ THIS FILE IN THE CODE TAB (FORMATTED)



This github repository has been made to keep track of our group's Sumobot robot hardware test code as well as version control for the final project's code.




LAB 15 TESTS          DESCRIPTION                                      PURPOSE OF CODE

TEST 1 -              Measure voltage of VREG                          (NO CODE)

TEST 2 -              Verify 3.3V out on Pico                          (NO CODE)             

TEST 3 -              Measure current draw of Pico                     Blink 3 user LEDs
                      while it is blinking 3 LEDS

TEST 4 -              Benchtop Power Supply to power Pico &            Controlling the motors (NO LOAD)
                      DRV8835, Oscilloscope to measure the
                      voltage @ aen1, aph1, ben2, bph2, AO1,
                      AO2, BO1, BO2, be sure to document any
                      voltage spikes exceeding expectations

TEST 5 -              Benchtop Power Supply to power Pico &            Controlling the motors (WITH MOTOR LOAD)
                      DRV8835, Oscilloscope to measure the
                      voltage @ aen1, aph1, ben2, bph2, AO1,
                      AO2, BO1, BO2, current from Benchtop
                      Output using a multimeter through VIN of
                      the DRV8835, be sure to document any
                      voltage spikes exceeding expectations,
                      document these measurements and
                      summarize the results in the report.

TEST 6 -              Benchtop Power Supply to power Pico &              Serial Monitor IR Receiver signals from
                      use oscilloscope at that Receive pin on the        another Pico sending IR transmissions
                      Pico to verify reception of IR Signal.
                      Conduct several measurements to
                      experimentally determine the range of the
                      IR Receiver, document these
                      measurements and summarize the results in
                      the report.

TEST 7 -              Measure elapsed time from sending the              Serial Monitor IR Receiver signals from
                      signal to reception of the IR signal using         another Pico sending IR transmissions
                      the Oscilloscope to measure both the IR
                      Transmitter output and the IR Receiver
                      input, be sure to have common ground.
                      Use Benchtop Power Supply to provide
                      power to the system through the voltage
                      regulator.
                      Use Multimeter to measure the loaded
                      output voltage of the regulator. Use the
                      multimeter to measure the current draw of
                      the SumoBot. Document these
                      measurements and summarize the resilts in
                      the report.

TEST 8 -              Oscilloscope to measure the Voltage across          Controlling the motors and Serial Monitor
                      a motor. Include a screenshot of the
                      measurement and summary in the Lab
                      Report. Run the motors for ~1minute to
                      measure the batteries output voltage.
                      Monitor the voltage from the battery and
                      record both the initial voltage across the
                      battery and the final voltage across the
                      battery.
                      Verify that the elapsed time from sending
                      the signal to reception of the IR signal
                      using the Oscilloscope to measure both the
                      IR Transmitter output and the IR Receiver
                      input, be sure to have common ground is
                      approximately the same as previously
                      measured.

TEST 9 -              Visual verification of functionality through        Integrate Joystick with IR transmitter, 
                      recording a 2second video. Video or link to         Joystick mapped to drive forward, backwards,
                      the video should be included in the report.         turn left and turn right.

TEST 10 -             Oscilloscope to measure the voltage at the          Code to receive input from RF receiver
                      output of the voltage dividers, benchtop            Code for 4 LEDS to toggle for each of 4 buttons
                      power supplies.                                     on the RF transmitter remot

TEST 11 -             Student's Design                                    Integration of the RF Receiver w/
                                                                          the rest of the code base.
                                                                          Ability to switch between the RF
                                                                          and IR receivers.
                                                                          The Joystick controls should be
                                                                          mapped to the following
                                                                          functionality of the SumoBot:
                                                                          Drive Forward, Backward, Turn
                                                                          Left, Turn Right.
                                                                          The RF Transmitter code is also
                                                                          mapped to the following:
                                                                          functionality of the SumoBot:
                                                                          Drive Forward, Backward, Turn
                                                                          Left, Turn Right.

