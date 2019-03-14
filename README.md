# DE3-ROB1 Building a well with DENIRO | 2019

## About the Project
DE NIRO is a Baxter robot which belongs to the Design Engineering department at Imperial College London. This repository contains the code and documentation required to understand how to do the following:
1. Building a Well in Gazebo
2. Building a Well with Baxter
3. Building a Well with Baxter Using Dual Arms
4. Building a Well Using Image Perception

![octocat](https://github.com/pz716/DE3-ROB1-Well-2019/blob/master/Wiki%20Images/BAXTER.png)

## Contents
### 01 Setting Up Workstation
[Setting up Gazebo](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Setting-Up-Gazebo)    
[Setting up DE NIRO](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Setting-up-Deniro)    

### 02 Milestones
**[[Milestone 01 - Building a Well in Gazebo| https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Milestone-01:-Building-a-Well-in-Gazebo]]**
>[Pick and Place](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Pick-and-Place)    
>[Spawning Models](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Spawning-Models)    
>[Adjusting the Gripper's Width](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Adjusting-the-Gripper's-Width)    
>[Correcting Brick Model Physics](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Correcting-Brick-Model-Physics)    
>[Stopping Unwanted Movement of DE NIRO](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Stopping-Unwanted-Movement-of-DE-NIRO)    

**[[Milestone 02 - Building a Well with Baxter| https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Milestone-02:-Building-a-Well-with-Baxter]]**
>[Well Structure for Single Arm](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Well-Structure-for-Single-Arm)    
>[Inverse Kinematics and Redundancy Reduction](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Inverse-Kinematics-and-Redundancy-Reduction)    
>[Quaternion](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Quaternion)    
>[Solving Overshooting](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Solving-Overshooting)    
>[Motion Planning](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Motion-Planning)    
>[Brick Grip Detection](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Brick-Grip-Detectio)    

**[[Milestone 03 - Building a Well with Baxter Using Dual Arms| https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Milestone-03:-Building-a-Well-with-Baxter-Using-Dual-Arms]]**
>[Dual Arm Control](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Dual-Arm-Control)    
>[Well Structure for Dual Arms](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Well-Structure-for-Dual-Arm)    

**[[Milestone 04 - Building a Well with Baxter using Image Recognition|https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Milestone-04:-Building-a-Well-with-Baxter-Using-Image-Perception]]**
>[Camera Settings in Baxter](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Understanding-the-Camera-Settings-in-Baxter)    
>[Opening the Camera Feed](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Opening-the-Camera-Feed-in-DENIRO)    
>[Taking a Photo Using Baxter Cameras](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Taking-a-Picture-Using-Baxter-Cameras)    
>[Image Analysis](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Image-Analysis)    

### 04 Additional Research - Franka Emika Panda Robot
>[Failure when running Franka Panda in Gazebo](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Failure-Running-Panda-in-Gazebo)    
>[Redundancy Reduction](https://github.com/pz716/DE3-ROB1-Well-2019/wiki/Redundancy-Reduction)    

---
## The Team
- Hugo Hasted
- Jacob Tan
- Ryan Dai
- Oliver Thompson
- Omer Quadri
- Richard Rui Zhang
- Pilar Zhang Qiu
***
## Useful Resources and Bibliography
### GitHub and Git
* https://guides.github.com
* Git basics: https://git-scm.com/book/en/v2/Getting-Started-Git-Basics
* The super simple beginners guide to Git: http://rogerdudler.github.io/git-guide/
* Understanding the workflow of git version control: https://www.git-tower.com/learn/cheat-sheets/vcs-workflow
* Guidance to git commands you may need in the command line: https://www.git-tower.com/blog/git-cheat-sheet/

### Python
Structuring large Python projects
* Python documentation on what package, module, script are: https://docs.python.org/3/tutorial/modules.html
* Simple example of this structure and how it is documented: https://github.com/brandon-rhodes/sphinx-tutorial/blob/master/triangle-project/trianglelib

### Writing Code: Python Conventions & Documentation
* Overall guide to documentation in Python http://docs.python-guide.org/en/latest/writing/documentation/
* The PEP8 on writing your code keeping to the convention (supported by PyCharm): https://www.python.org/dev/peps/pep-0008/
* The PEP257 on documenting your code:  https://www.python.org/dev/peps/pep-0257/
* Handling errors with exceptions and raising errors: https://docs.python.org/2/tutorial/errors.html

### Differences between Python 2 and Python 3
* http://sebastianraschka.com/Articles/2014_python_2_3_key_diff.html
* Porting code from Python 2 to Python 3: https://docs.python.org/3/howto/pyporting.html

### Python commands
* ``pyclean .`` will clean the current directory of \_pycache_ and .pyc

### Getting Started with ROS
To 'get started' with learning ROS, you may find doing the following helps you to understand ROS better:
* In your home directory, ensure you have [set up a complete catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)  * Within that workspace, [create a catkin package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
