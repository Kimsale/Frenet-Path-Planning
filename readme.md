# CppRobotics

This is the cpp implementation of the [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)

## Requirment
- cmake
- opencv 3.3
- Eigen 3
- [CppAD](https://www.coin-or.org/CppAD/Doc/install.htm) / [IPOPT](https://www.coin-or.org/Ipopt/documentation/node14.html) (*for MPC convex optimization*) [install tips](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/install_Ipopt_CppAD.md)
- ~~ROS~~ (*~~To make the repo lightweight :)~~. Yet, we may still need it for 3D visualization.*)

## Build
     $ mkdir build
     $ cd build
     $ cmake ../
     $ make -j 8

Find all the executable files in ***build/bin***.


# Path Planning

## Frenet Frame Trajectory

* black line: the planned spline path
* red circle: the obstacle
* blue circle: the planned trajectory
* green circle: the real-time position of robot

<img src="https://ram-lab.com/file/tailei/gif/frenet.gif" alt="frenet" width="400"/>

[Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame)
