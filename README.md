## FruitFlys Arena Firmware(arduino Due)

-------------------------------------------------------------------------------

 Firmware enabling the control of the "FruitFly Arena" by Ruijsink Dynamic
 Engineering. Communication to the arena is done using the Arduino Due

 Typical use: Example here

 M.M.Span / D.H.van Rijn

 Department of Psychology
 Faculty of Behavioral Sciences
 University of Groningen, the Netherlands

 (C) 2013, 2014, 2017


 # History:

 1.0a - 1.0d: MS: Initial versions
 1.0e: HvR: Added (again) the boost auto-shut-off functionality. After issueing a boost command, the boost will stay on for at most BOOSTDURATION.
 1.0f: HvR: GetTemps now works as expected & added PID Control library to update temperature

 Problem with PID control is that our boost works way to fast to use derivatives. The old system (large increases if temperature is clearly below set
 value worked a lot better. Reinstate that for 1.0g, and get rid of the PID.

 1.0g: HvR: Get rid of PID control. Change PWM by 1 until at 100 Hz, and extreme changes if bigger temperature changes are requested. Also copied
 all control code to main loop, and disabled timer. Reason for this is that the missing characters in the serial input stream seem to be caused by
 the timing interrupt (i.e., it doesn't happen anymore after the timer was disabled). This does mean that the execution of *all* commands passed
 over the serial loop should be executed *quickly*, as PWM and boost will not be updated during commands that need long for execution. Therefore removed
 flash and demo. Added p command, which gives a human readable output of current temperatures and PWM values.

 1.0h MS: Using native USB line (enabling higher resolution measurements) and adding LCD support. HvR: Changed order of tiles by switching pin positions. (The original code refered to the left tile as
 right, and vice versa.)
 2.0a MS: changed inteface from LCD to Nextion
 https://www.itead.cc/wiki/Nextion_HMI_Solution#Nextion_Editor_Quick_Start_Guide
 2.0b MS: (re)using the PID library the control the temperatures of the tiles.
 https://github.com/br3ttb/Arduino-PID-Library
 2.0c MS: code cleanup and PID calibration. Added PID, XAXIS and YAXIS commands
 https://github.com/br3ttb/Arduino-PID-Library
 2.0d MS: Created version to compile for both the LCD and the Nextion versions of the Arena
 2.0e MS/RR Changed PWM output of Copperblock cooler to DAC, Changed default PID parameters for Copperblock. Slower changes: longer life of peltier elements
