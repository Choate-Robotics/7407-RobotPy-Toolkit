# 7407-RobotPy-Toolkit  
### FRC Wired Boars Team 7407's toolkit for usage with RobotPy  

## TODO:
 - UnitTests (IN PROGRESS)
 - Smart Dashboard Wrapper (NOT STARTED)
 - Fiducial Odometry (IN PROGRESS)

## Features:  
### Motor Wrappers:  
 - Ctre Motors
 - Rev Motors
### Network Control:
 - Robot Statuses
 - DS to Robot Comms
### Operator Interface:
 - Keymap simplification
 - Mapped controllers
### Sensors:
 - Gyrometer
 - Color Sensors (Usage with I2C Multiplexer)
 - Limelight
 - Limit Switch (I2C)
### Subsystem Templates:
 - Swerve Drive
 - Differential Drive
### Optimized Unum
### Utils
 - Logging
 - Math
 - Units (Unum Based)
### In Progress: Swerve Simulation

## TO BUILD AND DEPLOY:
 - python -m build .
 - twine upload dist/*