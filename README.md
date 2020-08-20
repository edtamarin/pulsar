# pulsar
Line Follower robot for the 2017 RobotChallenge.

## Hardware
### Chassis
The body of the robot is a custom-made PCB. In hindsight, this added more weight to the overall design. The PCB was designed in EagleCAD.
### Wheels
To increase traction, the wheels were custom-made. The rims were 3D printed to be wide for more grip. The tyres were made from a silicone tube to increase grip on the surface.

## Code
The code is very simple. This design does not use a PID, however it would benefit from one. 

The array of sensors was placed in the front of the robot. The algorithm calculates the deviation from the straight line and runs the motors accordingly.


