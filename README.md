# Simulation and Manipulation of a RPP Robot using Matlab

### Coursework work project for Modelling and Control of Robots

## Objective 

The objective of the project is to employ trajectory calculation and workspace plotting of a RPP robot using robotic toolbox in MATLAB.

## Concepts 

1. Forward and Inverse Kinematics
2. Trajectory Planning
3. DH Table 

## Instructions to Run the Program

1. Run the [main.m](main.m) file in the folder "MATLAB Code"
2. Enter the Initial Coordinates in the format (x1,y1,z1)
3. Enter the Next Coordinates in the format (x2,y2,z2)
4. The Trajectory will be generated and animation will be shown
5. The joint variables trajectory, velocity and acceleration graphs will be shown.
6. Furthur trajectory points can be entered if needed. 
7. Press "Ctrl+C" to terminate the program.

## DH Table for the Mechanism

<img src = "resources\Picture1.png" width="300" height="300">

|Links|Theta|d|a|Alpha|
|---|---|---|---|---|
|1|theta1|5|0|0|
|2|0|d2|0|-pi/2|
|3|0|d3|0|0|


## Results

The workspace of the Robot can be seen below : 

<img src = "resources\Picture2.png" width="500" height="400">

The Simulation for a small workspace can be seen below : 

<img src = "resources\Picture3.png" width="500" height="400">

The Simulation for a larger workspace can be seen below : 

<img src = "resources\Picture4.png" width="500" height="400">

## Joint Variables Trajectory, Velocity and Acceleration Graphs

### Small Movement 

<img src = "resources\Picture5.png" width="500" height="400">

### Large Movement

<img src = "resources\Picture6.png" width="500" height="400">


## Simulation Video

The simulation video for this project can be found in this [Link.](https://youtu.be/a-GioV7z1Pk)