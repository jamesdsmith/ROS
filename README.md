# ROS
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](LICENSE)

ROS packages for the drone-based 3D reconstruction project. It is developed by **David Fridovich-Keil**, a first year graduate student in the [Video and Image Processing Lab](http://www-video.eecs.berkeley.edu) at UC Berkeley.

## Overview
This repository is entirely written in C++, using ROS middleware to communicate with the hardware onboard. We rely on PCL (Point Cloud Library) for algorithms like ICP to align point clouds. We also rely on algorithms from the **Berkeley Path Planning** library to do exploration and path planning.

## Status
This library is currently in development, and right now it really doesn't do all that much. Don't worry! I'm working on it...

## Build Instructions
We follow the standard ROS build scheme. Just download the repository and from the top directory type:

```bash
catkin_make
```

