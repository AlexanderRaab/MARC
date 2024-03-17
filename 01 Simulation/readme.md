# Simulation 
This folder contains data and code for simulating parts of the robots software. 

## Data
The _data_ subfolder includes measurement and parameter files for physical components used in the actual robot. Currently only results for the parameter identification of the DC motors can be found there.

## Matlab
Simulations and identification routines can be found in the _Matlab_ folder. 
The following files are currently included:
*   Routines and Matlab code for identifying model parameters of DC motors
    (using an ARX or a 1st order lag element approximation)
*   A SIMULINK model for simulating the Differential Drive kinematics including voltage controlled motors and encoder based odometry

## Example images
![Simulink model](04%20Documentation\img\simulink_model_diff_drive.png)
Motor identification ARX model           |  Motor identification PT1 model 
:-------------------------:|:-------------------------:
![MARC](04%20Documentation\img\motor_ident_ARX.jpg)  |  ![MARC](04%20Documentation\img\motor_ident_pt1.jpg)