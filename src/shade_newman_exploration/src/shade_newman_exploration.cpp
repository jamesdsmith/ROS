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
}

// Get the coordinates in 3D of a given voxel.
bool ShadeNewmanExploration::IndicesToCoordinates(size_t ii, size_t jj, size_t kk,
                                                  double& x, double& y,
                                                  double& z) const {
  if (ii >= length_ || jj >= width_ || kk >= height_) {
    ROS_ERROR("%s: Indices are out of bounds.", name_.c_str());
    return false;
  }

  // Set x, y, z.
  x = xmin_  + (static_cast<double>(ii) + 0.5) * resolution_;
  y = ymin_  + (static_cast<double>(jj) + 0.5) * resolution_;
  z = zmin_  + (static_cast<double>(kk) + 0.5) * resolution_;

  return true;
}

// Get the voxel indices for a given set of 3D coordinates.
bool ShadeNewmanExploration::CoordinatesToIndices(double x, double y, double z,
                                                  size_t& ii, size_t& jj,
                                                  size_t& kk) const {
  if (x <= xmin_ || x >= x_max_ ||
      y <= ymin_ || y >= y_max_ ||
      z <= zmin_ || z >= z_max_) {
    ROS_ERROR("%s: Coordinates are out of bounds.", name_.c_str());
    return false;
  }

  // Set ii, jj, kk.
  ii = static_cast<size_t>((x - xmin_) / resolution_);
  jj = static_cast<size_t>((y - ymin_) / resolution_);
  kk = static_cast<size_t>((z - zmin_) / resolution_);

  return true;
}


// Convert an Octomap octree to a regular grid.
bool ShadeNewmanExploration::GenerateOccupancyGrid(octomap::OcTree* octree) {
  double x, y, z;

  // Iterate over voxels.
  for (size_t ii = 0; ii < length_; ii++) {
    for (size_t jj = 0; jj < width_; jj++) {
      for (size_t kk = 0; kk < height_; kk++) {
        if (!IndicesToCoordinates(ii, jj, kk, x, y, z)) {
          ROS_WARN("%s: Indices out of bounds.", name_.c_str());
          continue;
        }

        // Query octree and set voxel grid.
        double occupancy_probability = octree->search(x, y, z)->getOccupancy();
        if (occupancy_probability > occupied_lower_threshold_)
          *occupancy_(ii, jj, kk) = OCCUPIED;
        else if (occupancy_probability < free_upper_threshold)
          *occupancy_(ii, jj, kk) = FREE;
        else
          *occupancy_(ii, jj, kk) = UNKNOWN;
      }
    }
  }

}

// Solve Laplace's equation on the grid. SolveLaplace() sets its arguments to
// the direction of steepest descent.
bool ShadeNewmanExploration::SolveLaplace(double pose_x, double pose_y,
                                          double pose_z, double& dir_x,
                                          double& dir_y, double& dir_z) {
  // Find frontiers.
  if (!FindFrontiers()) {
    ROS_WARN("%s: Did not find any frontiers.", name_.c_str());
    return;
  }

  // Solve the Laplace equation on this regular grid.
  for (size_t ii = 0; ii < niter_; ii++) {
    if (LaplaceIteration() < tolerance_) {
      double x, y, z;
      if (!GetSteepestDescent(pose_x, pose_y, pose_z, dir_x, dir_y, dir_z)) {
        ROS_ERROR("%s: Error finding direction of steepest descent.",
                  name_.c_str());
        return false;
      }

      return true;
    }
  }

  // Did not converge. Set dir_x/y/z to zero.
  ROS_WARN("%s: Laplace solver did not converge after %ul iterations.",
           name_.c_str(), niter_);
  dir_x = 0.0; dir_y = 0.0; dir_z = 0.0;
  return false;
}

// Helper GetSteepestDescent() finds the direction of steepest descent
// from the robot's current position. If there's an out of bounds error,
// return dir_x/y/z = 0.
bool ShadeNewmanExploration::GetSteepestDescent(double pose_x, double pose_y,
                                                double pose_z, double& dir_x,
                                                double& dir_y, double& dir_z) const {
  // Get indices.
  size_t ii, jj, kk;
  if (!CoordinatesToIndices(pose_x, pose_y, pose_z, ii, jj, kk)) {
    ROS_ERROR("%s: Pose is out of bounds.", name_.c_str());
    dir_x = 0.0; dir_y = 0.0; dir_z = 0.0;
    return false;
  }

  // Find the local gradient in the 26-connected neighborhood.
  double best_dir_x, best_dir_y, best_dir_z;
  double best_dir_mag = -1.0;
  for (di = -1; di <= 1; di++) {
    for (dj = -1; dj <= 1; dj++) {
      for (dk = -1; dk <= 1; dk++) {
        if (di == 0 || dj == 0 || dk == 0)
          continue;

        // Get gradient.
        if (!GetGradient(ii + di, jj + dj, kk + dk, dir_x, dir_y, dir_z))
          continue;

        // Set best.
        if (dir_x*dir_x + dir_y*dir_y + dir_z*dir_z > best_dir_mag) {
          best_dir_x = dir_x;
          best_dir_y = dir_y;
          best_dir_z = dir_z;
          best_dir_mag = dir_x*dir_x + dir_y*dir_y + dir_z*dir_z;
        }
      }
    }
  }

  // Return best if it exists.
  if (best_dir_mag <= 0.0) {
    ROS_ERROR("%s: No valid direction.", name_.c_str());
    return false;
  }

  dir_x = best_dir_x;
  dir_y = best_dir_y;
  dir_z = best_dir_z;
  return true;
}

// Get the gradient at a particular voxel. Use two-sided finite differences.
bool ShadeNewmanExploration::GetGradient(size_t ii, size_t jj, size_t kk,
                                         double& dir_x, double& dir_y,
                                         double& dir_z) const {
  // Check valid and free.
  if (!potential_->IsValid(ii, jj, kk))
    return false;
  if (*occupancy_(ii, jj, kk) != FREE)
    return false;

  // Compute two-sided finite differences when possible.
  // Assume that frontiers are set to zero potential in the solve step.
  double left, right, front, back, up, down;
  size_t x_length, y_length, z_length;
  x_length = y_length = z_length = 2;

  // Check left.
  if (!occupancy_->IsValid(ii - 1, jj, kk) ||
      *occupancy_(ii - 1, jj, kk) == OCCUPIED) {
    left = *potential_(ii, jj, kk);
    x_length--;
  } else {
    left = *potential_(ii - 1, jj, kk);
  }

  // Check right.
  if (!occupancy_->IsValid(ii + 1, jj, kk) ||
      *occupancy_(ii + 1, jj, kk) == OCCUPIED) {
    right = *potential_(ii, jj, kk);
    x_length--;
  } else {
    right = *potential_(ii + 1, jj, kk);
  }

  // Check back.
  if (!occupancy_->IsValid(ii, jj - 1, kk) ||
      *occupancy_(ii, jj - 1, kk) == OCCUPIED) {
    back = *potential_(ii, jj, kk);
    y_length--;
  } else {
    back = *potential_(ii, jj - 1, kk);
  }

  // Check front.
  if (!occupancy_->IsValid(ii, jj + 1, kk) ||
      *occupancy_(ii, jj + 1, kk) == OCCUPIED) {
    front = *potential_(ii, jj, kk);
    y_length--;
  } else {
    front = *potential_(ii, jj + 1, kk);
  }

  // Check down.
  if (!occupancy_->IsValid(ii, jj, kk - 1) ||
      *occupancy_(ii, jj, kk - 1) == OCCUPIED) {
    down = *potential_(ii, jj, kk);
    x_length--;
  } else {
    down = *potential_(ii, jj, kk - 1);
  }

  // Check up.
  if (!occupancy_->IsValid(ii, jj, kk + 1) ||
      *occupancy_(ii, jj, kk + 1) == OCCUPIED) {
    up = *potential_(ii, jj, kk);
    x_length--;
  } else {
    up = *potential_(ii, jj, kk + 1);
  }

  // Compute slopes.
  double dir_x = (x_length > 0) ?
    (right - left) / (static_cast<double>(x_length) * resolution_) : 0.0;
  double dir_y = (y_length > 0) ?
    (front - back) / (static_cast<double>(y_length) * resolution_) : 0.0;
  double dir_z = (z_length > 0) ?
    (up - down) / (static_cast<double>(z_length) * resolution_) : 0.0;

  return true;
}


// Helper LaplaceIteration() does one iteration of Laplace solving, and
// returns the maximum relative error.
double ShadeNewmanExploration::LaplaceIteration() {
  
}

// Update set of frontier and obstacle boundary voxels.
// Note: This is a very naive implementation right now. To speed up, it
//       should be possible to examine only the deltas between this map
//       and the last map.
bool ShadeNewmanExploration::FindFrontiers() {
  frontiers_.clear();
  obstacles_.clear();

  // Iterate over all voxels.
  for (size_t ii = 0; ii < length_; ii++) {
    for (size_t jj = 0; jj < width_; jj++) {
      for (size_t kk = 0; kk < height_; kk++) {
        // Frontier identification.
        if (*occupancy_(ii, jj, kk) == UNKNOWN) {
          if ((occupancy_->IsValid(ii - 1, jj, kk) &&
               *occupancy_(ii - 1, jj, kk) == FREE) ||
              (occupancy_->IsValid(ii + 1, jj, kk) &&
               *occupancy_(ii + 1, jj, kk) == FREE) ||
              (occupancy_->IsValid(ii, jj - 1, kk) &&
               *occupancy_(ii, jj - 1, kk) == FREE) ||
              (occupancy_->IsValid(ii, jj + 1, kk) &&
               *occupancy_(ii, jj + 1, kk) == FREE) ||
              (occupancy_->IsValid(ii, jj, kk - 1) &&
               *occupancy_(ii, jj, kk - 1) == FREE) ||
              (occupancy_->IsValid(ii, jj, kk + 1) &&
               *occupancy_(ii, jj, kk + 1) == FREE))
            frontiers_.insert(std::make_tuple(ii, jj, kk));
        }

        // Obstacle boundary identification.
        else if (*occupancy_(ii, jj, kk) == OCCUPIED) {
          if ((occupancy_->IsValid(ii - 1, jj, kk) &&
               *occupancy_(ii - 1, jj, kk) == FREE) ||
              (occupancy_->IsValid(ii + 1, jj, kk) &&
               *occupancy_(ii + 1, jj, kk) == FREE) ||
              (occupancy_->IsValid(ii, jj - 1, kk) &&
               *occupancy_(ii, jj - 1, kk) == FREE) ||
              (occupancy_->IsValid(ii, jj + 1, kk) &&
               *occupancy_(ii, jj + 1, kk) == FREE) ||
              (occupancy_->IsValid(ii, jj, kk - 1) &&
               *occupancy_(ii, jj, kk - 1) == FREE) ||
              (occupancy_->IsValid(ii, jj, kk + 1) &&
               *occupancy_(ii, jj, kk + 1) == FREE))
            obstacles_.insert(std::make_tuple(ii, jj, kk));
        }
      }
    }
  }
}

// Publish the goal location.
void ShadeNewmanExploration::PublishGoal(double x, double y, double z) const {
  geometry_msgs::Vector3Stamped msg;

  msg.vector.x = x;
  msg.vector.y = y;
  msg.vector.z = z;
  msg.header.stamp = stamp_;

  goal_publisher_.publish(msg);
}
