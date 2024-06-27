This is a Proof-of-Concept for proportional control of a Mould King Servo Motor

Requirements
1) MK 4.0 or 6.0 hub and controller
2) MK Servo Motor
3) Arduino Nano + L298N mini motor driver (or compatible) + some other minor components (see wiring diagram)

What the PoC is doing
1) It intercepts the connection between MK Hub and MK Servo
2) It reads C1/C2 signals at 480 Hz from the MK hub (which are proportional to the inputs on MK Controller) 
3) It outputs these C1/C2 signals at 980 Hz for the MK Servo
4) There are also some workarounds specific for MK Servo to solve fluttering and power saving 

Limitations
- huge size of the assembly (for now)
- the code is specific for Mould King
- it is not working with other servo motors (also tested it with CaDa but with no luck)
- it can be modified to work also with original LEGO PF Servo (but not done yet)

Wiring

PoC photos
