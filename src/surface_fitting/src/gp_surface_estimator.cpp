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
// Defining the GPSurfaceEstimator class.
//
///////////////////////////////////////////////////////////////////////////////

#include <surface_fitting/gp_surface_estimator.h>

// Constructor/destructor.
GPSurfaceEstimator::GPSurfaceEstimator() : initialized_(false) {}
GPSurfaceEstimator::~GPSurfaceEstimator() {}

// Initialize.
bool GPSurfaceEstimator::Initialize(const ros::NodeHandle& n, UAVMapper* map) {
  name_ = ros::names::append(n.getNamespace(), "gp_surface_estimator");

  if (!map) {
    ROS_ERROR("%s: Pointer to UAVMapper object was null.", name_.c_str());
    return false;
  }

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Set map.
  map_ = map;

  // Initialize matrices.
  K11_ = Eigen::MatrixXd::Zero(knn_, knn_);
  K12_ = Eigen::VectorXd::Zero(knn_);
  training_distances_ = Eigen::VectorXd::Zero(knn_);

  initialized_ = true;
  return true;
}

// Load parameters.
bool GPSurfaceEstimator::LoadParameters(const ros::NodeHandle& n) {
  if (!ros::param::get("/uav_slam/surface/noise_sd", noise_sd_))
    return false;
  if (!ros::param::get("/uav_slam/surface/gamma", gamma_))
    return false;
  if (!ros::param::get("/uav_slam/surface/knn", knn_))
    return false;
  if (!ros::param::get("/uav_slam/surface/training_delta", training_delta_))
    return false;

  return true;
}

// Register callbacks.
bool GPSurfaceEstimator::RegisterCallbacks(const ros::NodeHandle& n) {
  return true;
}

// Compute signed distance and uncertainty to query point.
void GPSurfaceEstimator::SignedDistance(const pcl::PointXYZ& query,
                                        const pcl::PointXYZ& pose,
                                        double& distance, double& variance) const {
  // Get nearest neigbors.
  std::vector<pcl::PointXYZ> neighbors;
  if (!map_->KNearestNeighbors(query, knn, neighbors)) {
    ROS_ERROR("%s: UAVMapper returned error code on knn search.", name_.c_str());
    distance = std::numeric_limits<double>::infinity();
    variance = std::numeric_limits<double>::infinity();
    return;
  }

  // Generate training data.
  // TODO!! MAKE THIS MORE EFFICIENT!
  std::vector<pcl::PointXYZ> training_points;
  for (size_t ii = 0; ii < neighbors.size(); ii++) {
    pcl::PointXYZ front, back;
    GenerateTrainingPoints(neighbors[ii], pose, front, back);
    training_points.push_back(front);
    training_distances_[2*ii] = -training_delta_;
    training_points.push_back(back);
    training_distances_[2*ii + 1] = training_delta_;
  }

  // Compute covariance and cross covariance.
  TrainingCovariance(training_points);
  CrossCovariance(training_points, query);

  // Gaussian conditioning.
  distance = K12_.transpose() * K11_.inverse() * training_distances_;
  variance = 1.0 - K12_.transpose() * K11_.inverse() * K12_;
}


// Generate a points in front of and behind a query point.
void GenerateTrainingPoints(const pcl::PointXYZ& query, const pcl::PointXYZ& pose,
                            const pcl::PointXYZ& front, const pcl::PointXYZ& back) {
  double dx = query.x - pose.x;
  double dy = query.y - pose.y;
  double dz = query.z - pose.z;
  double norm = std::sqrt(dx*dx + dy*dy + dz*dz);
  dx /= norm; dy /= norm; dz /= norm;

  front.x = query.x - training_delta_ * dx;
  front.y = query.y - training_delta_ * dy;
  front.z = query.z - training_delta_ * dz;
  back.x = query.x + training_delta_ * dx;
  back.y = query.y + training_delta_ * dy;
  back.z = query.z + training_delta_ * dz;
}

// RBF covariance kernel.
double GPSurfaceEstimator::RBF(const pcl::PointXYZ& p1,
                               const pcl::PointXYZ& p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return std::exp(-gamma_ * (dx*dx + dy*dy + dz*dz));
}

// Compute covariance of the training points.
void GPSurfaceEstimator::TrainingCovariance(const std::vector<pcl::PointXYZ> points) {
  for (size_t ii = 0; ii < points.size(); ii++) {
    for (size_t jj = 0; jj < ii; jj++) {
      double rbf = RBF(points[ii], points[jj]);
      K11_(ii, jj) = rbf;
      K11_(jj, ii) = rbf;
    }

    // Handle diagonal.
    K11_(ii, ii) = 1.0 + noise_sd_*noise_sd_;
  }
}

// Compute cross covariance vector of this query point against
// all the training data.
void GPSurfaceEstimator::CrossCovariance(const std::vector<pcl::PointXYZ>& points,
                                         const pcl::PointXYZ& query) {
  for (size_t ii = 0; ii < points.size(); ii++)
    K12_(ii) = RBF(points[ii], query);
}
