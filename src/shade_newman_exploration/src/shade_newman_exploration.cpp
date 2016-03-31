/*
 * Copyright (c) 2015, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Author: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// This defines the shade_newman_exploration class, which implements the method
// described in the paper published by Shade and Newman (2011), at
// http://rjshade.com/work/files/papers/pdf/shade_newman_icra2011_choosing.pdf.
//
///////////////////////////////////////////////////////////////////////////////

#include <shade_newman_exploration/shade_newman_exploration.h>

// Constructor/destructor.
ShadeNewmanExploration::ShadeNewmanExploration() : initialized_(false) {}
ShadeNewmanExploration::~ShadeNewmanExploration() {}

// Initialize.
bool ShadeNewmanExploration::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "shade_newman_exploration");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Initialize grids.
  length_ = static_cast<size_t>((xmax_ - xmin_) / resolution_);
  width_ = static_cast<size_t>((ymax_ - ymin_) / resolution_);
  height_ = static_cast<size_t>((zmax_ - zmin_) / resolution_);

  potential_ = new Array3D<double>(length_, width_, height_);
  occupancy_ = new Array3D<OccupancyType>(length_, width_, height_);

  initialized_ = true;
  return true;
}

// Load parameters.
bool ShadeNewmanExploration::LoadParameters(const ros::NodeHandle& n) {
  // Bounding box and resolution.
  if (!ros::param::get("/uav_slam/shade_newman/resolution", resolution_))
    return false;
  if (!ros::param::get("/uav_slam/shade_newman/xmin", xmin_))
    return false;
  if (!ros::param::get("/uav_slam/shade_newman/xmax", xmax_))
    return false;
  if (!ros::param::get("/uav_slam/shade_newman/ymin", ymin_))
    return false;
  if (!ros::param::get("/uav_slam/shade_newman/ymax", ymax_))
    return false;
  if (!ros::param::get("/uav_slam/shade_newman/zmin", zmin_))
    return false;
  if (!ros::param::get("/uav_slam/shade_newman/zmax", zmax_))
    return false;

  // Laplace solver.
  if (!ros::param::get("/uav_slam/shade_newman/tolerance", tolerance_))
    return false;
  if (!ros::param::get("/uav_slam/shade_newman/niter", niter_))
    return false;

  // I/O.
  if (!ros::param::get("/uav_slam/shade_newman/octomap_topic", octomap_topic_))
    return false;
  if (!ros::param::get("/uav_slam/shade_newman/goal_topic", goal_topic_))
    return false;

  return true;
}

// Register callbacks.
bool ShadeNewmanExploration::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle node(n);

  // Subscriber.
  octomap_subscriber_ =
    node.subscribe<octomap::Octomap>(octomap_topic_.c_str(), 20,
                                     &ShadeNewmanExploration::MapCallback, this);

  // Publisher.
  goal_publisher_ =
    node.advertise<geometry_msgs::Vector3Stamped>(goal_topic_.c_str(),
                                                  10, false);
  return true;
}

// Main callback. For each new map update, choose a direction.
void ShadeNewmanExploration::MapCallback(const octomap::Octomap& msg) {
  stamp_.fromNSec(msg.header.stamp);

  // Deserialize octree.
  octomap::OcTree* octree = octomap_msgs::fullMsgToMap(msg);

  // Generate a dense, regular occupancy grid.
  if (!GenerateOccupancyGrid(octree)) {
    ROS_ERROR("%s: Failed to generate a regular occupancy grid.", name_.c_str());
    return;
  }

  // Find frontiers.
  if (!FindFrontiers()) {
    ROS_WARN("%s: Did not find any frontiers.", name_.c_str());
    return;
  }

  // Solve the Laplace equation on this regular grid.
  double x, y, z;
  if (!SolveLaplace(x, y, z)) {
    ROS_WARN("%s: Laplace solving did not converge.", name_.c_str());
  }

  // Publish the goal location.
  PublishGoal(x, y, z);
}

// Convert an Octomap octree to a regular grid.
bool ShadeNewmanExploration::GenerateOccupancyGrid(octomap::OcTree* octree) {
  // Iterate over voxels.
}

// Solve Laplace's equation on the grid. Helper LaplaceIteration() does
// one iteration of Laplace solving, and returns the maximum relative
// error.
bool ShadeNewmanExploration::SolveLaplace(double& x, double& y, double& z);
double ShadeNewmanExploration::LaplaceIteration();

// Update list of frontier voxels.
bool ShadeNewmanExploration::FindFrontiers();

// Publish the goal location.
void ShadeNewmanExploration::PublishGoal(double x, double y, double z) const;
