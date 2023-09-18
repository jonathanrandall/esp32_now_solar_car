# esp32_now_solar_car
 solar powered rc buggy

 The video instructions are here (__These are contained in the file: Solar-Powered-RC-Buggy-With-Sun-Tracking-and-Esp32.pdf__):
 
 https://youtu.be/O6xnOEPSv2g
 


 There are detailed build instructions on instructables:
 https://www.instructables.com/Solar-Powered-RC-Buggy-With-Sun-Tracking-and-Esp32/

## This repository contains the following:

### car_side_esp32_solar_v2 
This is the sketch for the esp32 on the car side. It receives esp32-now commands and controls the servos to track the sun.

### remote_side_solar
This is the sketch for the remote control, which reads analogue inputs from the joystick and sends these to the car

### solar_tracker_calibrate
This is the sketch to calibrate the pan-tilt mechanism, work out maximum and minimum angles the mechanism can move

### motor_ddreiver_shield.fzz
This is the pcb fritzing file for the esp32 and two motor drivers.

### tracker_wiring.fzz
Fritzing diagram of tracker wiring with photoresistors and servos.

### new_tracerv2-Body 4.stl
The stl for the tracker mount
