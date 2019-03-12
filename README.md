# DE3-ROB1 Building a well with DENIRO
## Design Engineering MEng - 2019
DENIRO is a Baxter robot which belongs to the Design Engineering department at Imperial College London. This repository contains the code and documentation needed to understand how to build a well with DENIRO.

## About the project
Blah blah blah

## The Team
- Hugo Hasted
- Jacob Tan
- Ryan Dai
- Oliver Thompson
- Omer Quads
- Richard Rui Zhang
- Pilar Zhang Qiu

## Contents
### 01 Setting Up Workstation
**Setting up DENIRO & Worsktation** -------- Ric
> - [x] Downloading Scripts and Installation of Packages
> - [x] Setting up and Connecting to Baxter

### 02 Project Development
**Milestone 01 - Running Pick and Place in Gazebo**
>SUMMARY AND EXECUTION -------- Ric
>
>EXPLANATION 
>- [ ] Pick and Place Demo scripts -------- Jacob  
>- [ ] Editing the code to spawn the brick instead of a block -------- Hugo   
>- [ ] Gripper width -------- Pilar  
>- [ ] Correcting brick physics (CoM and friction) -------- Hugo  
>- [ ] Stopping DENIRO's unwanted movement -------- Jacob   
>
>TROUBLESHOOTING  

**Milestone 02 - Building a Well using DENIRO (Single Arm)**
>SUMMARY AND EXECUTION
>
>EXPLANATION
>- [ ] Motion Basics
>   - [ ] Well Plan and Brick Positions for Single Arm -------- Hugo/Omer
>   - [ ] IK Motion Planning for Single Arm -------- Ryan
>   - [ ] Quaternion - Mapping the cartesian space to the joint space / Finding the right quaternion -------- Jacob
>   - [ ] Redundancy Reduction -------- Ryan
>- [x] Solving Overshooting when Moving to the Spawn Location -------- Ric
>- [x] Curvature in the Motion Planning -------- Ric
>- [x] Failure to Pick up a Brick and Recovery
>   - [ ] Approach 1: Force Sensing -------- Ryan
>   - [ ] Approach 2: Gripper Distance -------- Ric
>
>TROUBLESHOOTING

**Milestone 03 - Building a Well using DENIRO (Dual Arm)**
>SUMMARY AND EXECUTION
>
>EXPLANATION
>- [ ] Simultaneous Placement of Bricks (Multithreading) -------- Jacob
>- [ ] Well Plan and Brick Positions for Dual Arm -------- Hugo/Omer
>
>TROUBLESHOOTING

**Milestone 04 - Image Perception**
>SUMMARY
>
>- [ ] General Overview - Image Perception for the Spawn Location -------- Oli
>
>EXPLANATION
>- [ ] Setting Up the Camera -------- Pilar
>- [ ] Taking and Saving a Picture -------- Omer
>- [ ] Image Recognition - Position and Angle of the Spawned Brick -------- Oli
>
>EXECUTION
>
>TROUBLESHOOTING
>- [ ] Troubleshooting: Camera Not Working on Physical Robot -------- Pilar
>- [ ] Troubleshooting: Camera Not Working on Simulator -------- Pilar
>- [ ] Troubleshooting: Removing the Gripper -------- Oli
>- [ ] Troubleshooting: Image Not Processed -------- Oli

### 02 Project Specifications and Requirements

### 03 Trials and Errors - Working with Franka Emika Panda
**Trial 01 - Using MoveIt with the RViz & Gazebo Implementation** -------- Pilar
>- [ ] Introduction to MoveIt
>- [ ] Running the Pick and Place demo in RViz
>- [ ] Error when executing the RViz trajectory in Gazebo
>- [ ] Debugging
>- [ ] Exploring possible solutions / Giving up on MoveIt

**Trial 02 - New IK Python Code** -------- Ryan
>- [ ] Redundancy Reduction & Pseudoinverse Jacobian
---
## Useful Resources

## Bibliography
