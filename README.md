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
---
## Contents
### 01 Setting Up Workstation
> **Setting up DENIRO & Worsktation** -------- Ric
> - [ ] Downloading Scripts and Installation of Packages
> - [ ] Setting up and Connecting to Baxter

### 02 Project Development
**Milestone 01 - Running Pick and Place in Gazebo**
>  SUMMARY
>  EXPLANATION
>- [ ] Pick and Place Demo scripts -------- Jacob
>- [ ] Editing the code to spawn the brick instead of a block -------- Hugo
>  CHALLENGES
>- [ ] Correcting brick physics (CoM and friction) -------- Hugo
>- [ ] Gripper width -------- Pilar
>- [ ] Stopping DENIRO's unwanted movement -------- Jacob
>  EXECUTION
>- [ ] Running demo in Gazebo -------- Ric
>  TROUBLESHOOTING

**Milestone 02 - Building a Well using DENIRO (Single Arm)**
> - SUMMARY
> - EXPLANATION
>   - [ ] Well Plan and Brick Positions for Single Arm -------- Hugo/Omer
>   - [ ] IK Motion Planning for Single Arm -------- Ryan
>   - [ ] Quaternion - Mapping the cartesian space to the joint space / Finding the right quaternion -------- Jacob
>   - [ ] Redundancy Reduction -------- Ryan\
> - CHALLENGES
>   - [ ] Solving Overshooting when Moving to the Spawn Location (Single Arm) -------- Ric
>   - [ ] Enhancing Motion Planning: Curvature in the Motion Planning -------- Ric\
> - EXECUTION
> - TROUBLESHOOTING

> **Milestone 03 - Building a Well using DENIRO (Dual Arm)**
> - SUMMARY
> - EXPLANATION
>   - [ ] Simultaneous Placement of Bricks (Multithreading) -------- Jacob\
> - CHALLENGES
> - [ ] Well Plan and Brick Positions for Dual Arm -------- Hugo/Omer\
> EXECUTION\
> TROUBLESHOOTING

> **Milestone 04 - Image Perception**
> SUMMARY
> - [ ] General Overview - Image Perception for the Spawn Location -------- Oli
> EXPLANATION
> - [ ] Setting Up the Camera -------- Pilar
> - [ ] Taking and Saving a Picture -------- Omer
> - [ ] Image Recognition - Position and Angle of the Spawned Brick -------- Oli
> CHALLENGES\
> EXECUTION\
> TROUBLESHOOTING
>   - [ ] Troubleshooting: Camera Not Working on Physical Robot -------- Pilar
>   - [ ] Troubleshooting: Camera Not Working on Simulator -------- Pilar
>   - [ ] Troubleshooting: Removing the Gripper -------- Oli
>   - [ ] Troubleshooting: Image Not Processed -------- Oli

> **Milestone 05 - Error Detection and Failure Recovery when Picking a Brick**
> SUMMARY\
> EXPLANATION
> - [ ] Solution 1: Force Sensing -------- Ryan
> - [ ] Solution 2: Distance Gripped -------- Ric
> - [ ] Solution Implementation -------- Ric
> CHALLENGES\
> EXECUTION\
> TROUBLESHOOTING
---
### 02 Project Specifications and Requirements
---
### 03 Trials and Errors - Working with Franka Emika Panda
> **Trial 01 - Using MoveIt with the RViz & Gazebo Implementation** -------- Pilar
> - [ ] Introduction to MoveIt
> - [ ] Running the Pick and Place demo in RViz
> - [ ] Error when executing the RViz trajectory in Gazebo
> - [ ] Debugging
> - [ ] Exploring possible solutions / Giving up on MoveIt

> **Trial 02 - New IK Python Code** -------- Ryan
> - [ ] Redundancy Reduction & Pseudoinverse Jacobian
---
### 04 Project Management
---
## Useful Resources
---
## Bibliography
