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

#ifndef SHADE_NEWMAN_EXPLORATION_H
#define SHADE_NEWMAN_EXPLORATION_H

#include <ros/ros.h>
#include <utils/map/array_3d.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap/AbstractOcTree.h>

#include <Eigen/Dense>
#include <cmath>
#include <unordered_set>
#include <limits>
#include <tuple>

class ShadeNewmanExploration {
public:
  explicit ShadeNewmanExploration();
  ~ShadeNewmanExploration();

  bool Initialize(const ros::NodeHandle& n);

  // Solve Laplace's equation on the grid. SolveLaplace() sets dir_x/y/z to
  // the direction of steepest descent.
  bool SolveLaplace(double pose_x, double pose_y, double pose_z,
                    double& dir_x, double& dir_y, double& dir_z);


private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Main callback. For each new map update, choose a direction.
  void MapCallback(const octomap_msgs::Octomap& msg);

  // Convert an Octomap octree to a regular grid.
  bool GenerateOccupancyGrid(octomap::OcTree* octree);
  bool IndicesToCoordinates(size_t ii, size_t jj, size_t kk,
                            double& x, double& y, double& z) const;
  bool CoordinatesToIndices(double x, double y, double z,
                            size_t& ii, size_t& jj, size_t& kk) const;
  bool IndicesToIndex(size_t ii, size_t jj, size_t kk, size_t& idx) const;
  bool IndexToIndices(size_t idx, size_t& ii, size_t& jj, size_t& kk) const;

  // Helper LaplaceIteration() does one iteration of Laplace solving, and
  // returns the maximum relative error.
  double LaplaceIteration(size_t pose_ii, size_t pose_jj, size_t pose_kk);
  double GetLocalMean(size_t ii, size_t jj, size_t kk) const;

  // Helper GetSteepestDescent() finds the direction of steepest descent
  // from the robot's current position.
  bool GetSteepestDescent(double pose_x, double pose_y, double pose_z,
                          double& dir_x, double& dir_y, double& dir_z) const;
  bool GetGradient(size_t ii, size_t jj, size_t kk,
                   double& dir_x, double& dir_y, double& dir_z) const;

  // Update list of frontier voxels.
  bool FindFrontiers();

  // Types for occupancy grid.
  typedef enum OccupancyEnum {OCCUPIED, FREE, UNKNOWN} OccupancyType;

  // Member variables.
  Array3D<double>* potential_;
  Array3D<OccupancyType>* occupancy_;
  std::unordered_set<size_t> frontiers_, obstacles_;
  ros::Subscriber octomap_subscriber_;

  double occupied_lower_threshold_; // lower bound on occupied likelihood
  double free_upper_threshold_;     // upper bound on free likelihood
  double xmin_, xmax_, ymin_, ymax_, zmin_, zmax_; // bounding box
  size_t length_, width_, height_;
  double resolution_; // grid resolution
  double tolerance_;  // tolerance for Laplace solver
  int niter_;         // number of interations in Laplace solver
  bool initialized_;
  std::string octomap_topic_;
  std::string goal_topic_;
  std::string name_;
};

#endif
