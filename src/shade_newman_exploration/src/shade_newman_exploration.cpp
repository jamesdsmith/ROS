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
    node.subscribe<octomap_msgs::Octomap>(octomap_topic_.c_str(), 20,
                                     &ShadeNewmanExploration::MapCallback, this);

  return true;
}

// Main callback. For each new map update, choose a direction.
void ShadeNewmanExploration::MapCallback(const octomap_msgs::Octomap& msg) {
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
  if (x <= xmin_ || x >= xmax_ ||
      y <= ymin_ || y >= ymax_ ||
      z <= zmin_ || z >= zmax_) {
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
          occupancy_->At(ii, jj, kk) = OCCUPIED;
        else if (occupancy_probability < free_upper_threshold_)
          occupancy_->At(ii, jj, kk) = FREE;
        else
          occupancy_->At(ii, jj, kk) = UNKNOWN;
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
#if 0
  if (!FindFrontiers()) {
    ROS_WARN("%s: Did not find any frontiers.", name_.c_str());
    return;
  }
#endif

  // Get robot pose indices.
  size_t pose_ii, pose_jj, pose_kk;
  if (!CoordinatesToIndices(pose_x, pose_y, pose_z, pose_ii, pose_jj, pose_kk)) {
    ROS_ERROR("%s: Robot is out of bounds.", name_.c_str());
    return false;
  }

  // Solve the Laplace equation on this regular grid.
  for (size_t ii = 0; ii < niter_; ii++) {
    if (LaplaceIteration(pose_ii, pose_jj, pose_kk) < tolerance_) {
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
  for (size_t di = 0; di <= 2; di++) {
    for (size_t dj = 0; dj <= 2; dj++) {
      for (size_t dk = 0; dk <= 2; dk++) {
        // Leave out query voxel.
        if (di == 1 && dj == 1 && dk == 1)
          continue;

        // Check not out of bounds.
        size_t test_ii, test_jj, test_kk;
        test_ii = ii + di; test_jj = jj + dj; test_kk = kk + dk;
        if (test_ii == 0 || test_jj == 0 || test_kk == 0)
          continue;
        if (test_ii > length_ || test_jj > width_ || test_kk > height_)
          continue;

        // Get gradient.
        if (!GetGradient(test_ii - 1, test_jj - 1, test_kk - 1,
                         dir_x, dir_y, dir_z))
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
  if (occupancy_->At(ii, jj, kk) != FREE)
    return false;

  // Compute two-sided finite differences when possible.
  // Assume that frontiers are set to zero potential in the solve step.
  double left, right, front, back, up, down;
  size_t x_length, y_length, z_length;
  x_length = y_length = z_length = 2;

  // Check left.
  if (!occupancy_->IsValid(ii - 1, jj, kk) ||
      occupancy_->At(ii - 1, jj, kk) == OCCUPIED) {
    left = potential_->At(ii, jj, kk);
    x_length--;
  } else {
    left = potential_->At(ii - 1, jj, kk);
  }

  // Check right.
  if (!occupancy_->IsValid(ii + 1, jj, kk) ||
      occupancy_->At(ii + 1, jj, kk) == OCCUPIED) {
    right = potential_->At(ii, jj, kk);
    x_length--;
  } else {
    right = potential_->At(ii + 1, jj, kk);
  }

  // Check back.
  if (!occupancy_->IsValid(ii, jj - 1, kk) ||
      occupancy_->At(ii, jj - 1, kk) == OCCUPIED) {
    back = potential_->At(ii, jj, kk);
    y_length--;
  } else {
    back = potential_->At(ii, jj - 1, kk);
  }

  // Check front.
  if (!occupancy_->IsValid(ii, jj + 1, kk) ||
      occupancy_->At(ii, jj + 1, kk) == OCCUPIED) {
    front = potential_->At(ii, jj, kk);
    y_length--;
  } else {
    front = potential_->At(ii, jj + 1, kk);
  }

  // Check down.
  if (!occupancy_->IsValid(ii, jj, kk - 1) ||
      occupancy_->At(ii, jj, kk - 1) == OCCUPIED) {
    down = potential_->At(ii, jj, kk);
    x_length--;
  } else {
    down = potential_->At(ii, jj, kk - 1);
  }

  // Check up.
  if (!occupancy_->IsValid(ii, jj, kk + 1) ||
      occupancy_->At(ii, jj, kk + 1) == OCCUPIED) {
    up = potential_->At(ii, jj, kk);
    x_length--;
  } else {
    up = potential_->At(ii, jj, kk + 1);
  }

  // Compute slopes.
  dir_x = (x_length > 0) ?
    (right - left) / (static_cast<double>(x_length) * resolution_) : 0.0;
  dir_y = (y_length > 0) ?
    (front - back) / (static_cast<double>(y_length) * resolution_) : 0.0;
  dir_z = (z_length > 0) ?
    (up - down) / (static_cast<double>(z_length) * resolution_) : 0.0;

  return true;
}

// Helper LaplaceIteration() does one iteration of Laplace solving, and
// returns the maximum relative change.
double ShadeNewmanExploration::LaplaceIteration(size_t pose_ii, size_t pose_jj,
                                                size_t pose_kk) {
  double max_delta = -std::numeric_limits<double>::infinity();

  // Set robot pose potential to unity.
  potential_->At(pose_ii, pose_jj, pose_kk) = 1.0;

  // Iterate over all free voxels and update.
  for (size_t ii = 0; ii < length_; ii++) {
    for (size_t jj = 0; jj < width_; jj++) {
      for (size_t kk = 0; kk < height_; kk++) {
        if (!occupancy_->At(ii, jj, kk) == FREE)
          continue;

        if (ii == pose_ii && jj == pose_jj && kk == pose_kk)
          continue;

        // Calculate mean with boundary conditions.
        double mean = GetLocalMean(ii, jj, kk);

        // Update delta.
        double delta = mean - potential_->At(ii, jj, kk);
        if (delta > max_delta)
          max_delta = delta;

        // Update voxel.
        potential_->At(ii, jj, kk) = mean;
      }
    }
  }

  return max_delta;
}

// GetLocalMean() finds the local mean (including boundary conditions)
// on the potential field.
double ShadeNewmanExploration::GetLocalMean(size_t ii, size_t jj, size_t kk) const {
  if (!potential_->IsValid(ii, jj, kk)) {
    ROS_ERROR("%s: Out of bounds error.", name_.c_str());
    return std::numeric_limits<double>::infinity();
  }

  // Handle object boundaries.
  size_t num_neighbors = 6;
  double left, right, front, back, up, down;
  left = right = front = back = up = down = 0.0;

  // Check left/right.
  if (!potential_->IsValid(ii - 1, jj, kk) ||
      occupancy_->At(ii - 1, jj, kk) == OCCUPIED ||
      !potential_->IsValid(ii + 1, jj, kk) ||
      occupancy_->At(ii + 1, jj, kk) == OCCUPIED) {
    num_neighbors -= 2;
  } else {
    left = potential_->At(ii - 1, jj, kk);
    right = potential_->At(ii + 1, jj, kk);
  }

  // Check back/front.
  if (!potential_->IsValid(ii, jj - 1, kk) ||
      occupancy_->At(ii, jj - 1, kk) == OCCUPIED ||
      !potential_->IsValid(ii, jj + 1, kk) ||
      occupancy_->At(ii, jj + 1, kk) == OCCUPIED) {
    num_neighbors -= 2;
  } else {
    back = potential_->At(ii, jj - 1, kk);
    front = potential_->At(ii, jj + 1, kk);
  }

  // Check dwn/up.
  if (!potential_->IsValid(ii, jj, kk - 1) ||
      occupancy_->At(ii, jj, kk - 1) == OCCUPIED ||
      !potential_->IsValid(ii, jj, kk + 1) ||
      occupancy_->At(ii, jj, kk + 1) == OCCUPIED) {
    num_neighbors -= 2;
  } else {
    down = potential_->At(ii, jj, kk - 1);
    up = potential_->At(ii, jj, kk + 1);
  }

  // Set mean and return.
  double mean = (num_neighbors > 0) ?
    (left + right + front + back + up + down) : 0.0;
  return mean;
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
        if (occupancy_->At(ii, jj, kk) == UNKNOWN) {
          if ((occupancy_->IsValid(ii - 1, jj, kk) &&
               occupancy_->At(ii - 1, jj, kk) == FREE) ||
              (occupancy_->IsValid(ii + 1, jj, kk) &&
               occupancy_->At(ii + 1, jj, kk) == FREE) ||
              (occupancy_->IsValid(ii, jj - 1, kk) &&
               occupancy_->At(ii, jj - 1, kk) == FREE) ||
              (occupancy_->IsValid(ii, jj + 1, kk) &&
               occupancy_->At(ii, jj + 1, kk) == FREE) ||
              (occupancy_->IsValid(ii, jj, kk - 1) &&
               occupancy_->At(ii, jj, kk - 1) == FREE) ||
              (occupancy_->IsValid(ii, jj, kk + 1) &&
               occupancy_->At(ii, jj, kk + 1) == FREE)) {
            size_t idx;
            if (!IndicesToIndex(ii, jj, kk, idx)) {
              ROS_ERROR("%s: Out of bounds error.", name_.c_str());
              continue;
            }

            frontiers_.insert(idx);
          }
        }

        // Obstacle boundary identification.
        else if (occupancy_->At(ii, jj, kk) == OCCUPIED) {
          if ((occupancy_->IsValid(ii - 1, jj, kk) &&
               occupancy_->At(ii - 1, jj, kk) == FREE) ||
              (occupancy_->IsValid(ii + 1, jj, kk) &&
               occupancy_->At(ii + 1, jj, kk) == FREE) ||
              (occupancy_->IsValid(ii, jj - 1, kk) &&
               occupancy_->At(ii, jj - 1, kk) == FREE) ||
              (occupancy_->IsValid(ii, jj + 1, kk) &&
               occupancy_->At(ii, jj + 1, kk) == FREE) ||
              (occupancy_->IsValid(ii, jj, kk - 1) &&
               occupancy_->At(ii, jj, kk - 1) == FREE) ||
              (occupancy_->IsValid(ii, jj, kk + 1) &&
               occupancy_->At(ii, jj, kk + 1) == FREE)) {
            size_t idx;
            if (!IndicesToIndex(ii, jj, kk, idx)) {
              ROS_ERROR("%s: Out of bounds error.", name_.c_str());
              continue;
            }

            obstacles_.insert(idx);
          }
        }
      }
    }
  }
}

// Indices to index. Get a single 1D index from a 3D index.
bool ShadeNewmanExploration::IndicesToIndex(size_t ii, size_t jj, size_t kk,
                                            size_t& idx) const {
  idx = ii * width_ * height_ + jj * height_ + kk;

  if (idx >= length_ * width_ * height_)
    return false;
  return true;
}

// Index to indices. Get 3D indices corresponding to the given 1D index.
bool ShadeNewmanExploration::IndexToIndices(size_t idx, size_t& ii, size_t& jj,
                                            size_t& kk) const {
  if (idx >= length_ * width_ * height_)
    return false;

  ii = idx / (width_ * height_);
  jj = (idx - ii * width_ * height_) / height_;
  kk = idx - ii * width_ * height_ - jj * height_;
  return true;
}
