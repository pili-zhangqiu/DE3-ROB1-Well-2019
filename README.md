# DE3-ROB1 Building a well with DENIRO
## Design Engineering MEng - 2019
DENIRO is a Baxter robot which belongs to the Design Engineering department at Imperial College London. This repository contains the code and documentation needed to understand how to build a well with DENIRO.

## About the project
(Blah blah blah)

## The Team
- Hugo Hasted
- Jacob Tan
- Ryan Dai
- Oliver Thompson
- Omer Quads
- Richard Rui Zhang
- Pilar Zhang Qiu

## Contents
### 01 Working with DENIRO

> **Setting up DENIRO & Worsktation**
> - [ ] Downloading Scripts and Installation
> - [ ] Setting up and Connecting to Baxter

> **Milestone 01 - Running Pick and Place in Gazebo**
> - [ ] Understanding the Pick and Place Demo scripts
> - [ ] Running demo to pick and place the block
> - [ ] Editing the code to spawn the brick instead of a block
> - [ ] Challenges when running the demo to pick and place the brick
>   - [ ] Solving the brick CoM issue
>   - [ ] Brick orientation and URDF values & Gripper width
>   - [ ] Collision sensitivity in brick - Friction (DENIRO was just flying away)

> **Milestone 02 - Building a Well using DENIRO (Single Arm)**
> - [ ] Well Plan and Brick Positions for Single Arm
> - [ ] IK Motion Planning for Single Arm
> - [ ] Quaternion - Mapping the cartesian space to the joint space / Finding the right quaternion
> - [ ] Redundancy Reduction
> - [ ] Enhancing Motion Planning: Curvature in the Motion Planning
> - [ ] Troubleshooting (Single Arm)
>   - [ ] Order of Brick Placement to Avoid Collisions (Single Arm)
>   - [ ] Solving Overshooting when Moving to the Spawn Location (Single Arm)

> **Milestone 03 - Building a Well using DENIRO (Dual Arm)**
> - [ ] Enhancing Speed: Simultaneous Placement of Bricks
> - [ ] Well Plan and Brick Positions for Dual Arm
> - [ ] IK Motion Planning for Dual Arm
> - [ ] Troubleshooting (Dual Arm)
>   - [ ] Order of Brick Placement to Avoid Collisions (Dual Arm)

> **Milestone 04 - Image Perception**
> - [ ] General Overview - Image Perception for the Spawn Location
> - [ ] Setting Up the Camera
>   - [ ] Troubleshooting: Camera Not Working on Physical Robot
>   - [ ] Troubleshooting: Camera Not Working on Simulator
> - [ ] IK to the Spawn Location
> - [ ] Taking and Saving a Picture
> - [ ] Image Recognition - Position and Angle of the Spawned Brick
>   - [ ] Troubleshooting: Removing the Gripper
>   - [ ] Troubleshooting: Image Not Processed

> **Milestone 05 - Error Detection and Failure Recovery when Picking a Brick**
> - [ ] Solution 1: Force Sensing
> - [ ] Solution 2: Distance Gripped
> - [ ] Critical Incident Approach

### 02 Project Specifications and Requirements

### 03 Trials and Errors - Working with Franka Emika Panda

> **Setting up Franka Emika Panda**
> - [ ] Task 1
> - [ ] Task 2
> - [ ] Task 3

> **Trial 01 - Using MoveIt with the RViz & Gazebo Implementation**
> - [ ] Introduction to MoveIt
> - [ ] Running the Pick and Place demo in RViz
> - [ ] Error when executing the RViz trajectory in Gazebo
> - [ ] Debugging
> - [ ] Exploring possible solutions / Giving up on MoveIt

> **Trial 02 - New IK Python Code**
> - [ ] Task 1
> - [ ] Task 2
> - [ ] [Redundancy Reduction & Pseudoinverse Jacobian](https://github.com/pz716/DE3-ROB1-Building-a-well-with-DENIRO/wiki/03_3_C---Redundancy-Reduction)

### 04 Project Management

## Useful Resources

## Bibliography
