# ROS
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](LICENSE)

ROS packages for the drone-based 3D reconstruction project. It is developed by **David Fridovich-Keil**, a first year graduate student in the [Video and Image Processing Lab](http://www-video.eecs.berkeley.edu) at UC Berkeley, and **James Smith**, a third year undergraduate at UC Berkeley.

## Overview
This repository includes packages to perform all the tasks required for localization and mapping, as well as for exploration and motion planning, and it is entirely written in C++, using ROS middleware to communicate with the hardware onboard. For localization and mapping, We rely on PCL (Point Cloud Library) for algorithms like ICP to align point clouds. While our approach to localization and mapping is not especially novel, we believe that our methods for exploration and planning are a significant extension of existing methods.

## Status
Currently, we are capable of running simultaneous localization and mapping in real time. We have also implemented several standard approaches to path planning. We are still in the theoretical stages of developing algorithms for exploration; expect more progress to follow.

## Build Instructions
We follow the standard ROS build scheme. Just download the repository and from the top directory type:

```bash
catkin_make
```

To build and run tests (created in the Google test suite), run:

```bash
catkin_make tests
catkin_make run_tests
```
