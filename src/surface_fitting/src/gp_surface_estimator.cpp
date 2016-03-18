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
bool GPSurfaceEstimator::Initialize(const ros::NodeHandle& n,
                                    const std::vector<pcl::PointXYZ>& points,
                                    const std::vector<double>& distances) {
  name_ = ros::names::append(n.getNamespace(), "gp_surface_estimator");

  // Check dimensions.
  if (points.size() != distances.size()) {
    ROS_ERROR("%s: Training points and distances have different lengths.",
              name_.c_str());
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

  // Set training points vector.
  for (const auto& p : points)
    training_points_.push_back(p);

  // Set training covariance and inverse.
  TrainingCovariance();
  K11_inv_ = K11_.inverse();

  // Set training mean.
  mu_training_ = Eigen::VectorXd::Zero(distances.size());
  for (size_t ii = 0; ii < distances.size(); ii++)
    mu_training_(ii) = distances[ii];

  initialized_ = true;
  return true;
}

// Load parameters.
bool GPSurfaceEstimator::LoadParameters(const ros::NodeHandle& n) {
  if (!ros::param::get("/uav_slam/surface/noise_sd", noise_sd_))
    return false;
  if (!ros::param::get("/uav_slam/surface/gamma", gamma_))
    return false;

  return true;
}

// Register callbacks.
bool GPSurfaceEstimator::RegisterCallbacks(const ros::NodeHandle& n) {
  return true;
}

// Compute signed distance and uncertainty to query point.
void GPSurfaceEstimator::SignedDistance(const pcl::PointXYZ& query,
                                        double& distance, double& variance) const {
  distance = K12_.transpose() * K11_inv_ * mu_training_;
  variance = 1.0 - K12_.transpose() * K11_inv_ * K12_;
}

// Add a point to the surface.
void GPSurfaceEstimator::AddPoint(const pcl::PointXYZ& point) {
  // TODO!!!
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
void GPSurfaceEstimator::TrainingCovariance() {
  K11_ = Eigen::MatrixXd::Zero(training_points_.size(),
                               training_points_.size());
  for (size_t ii = 0; ii < training_points_.size(); ii++) {
    for (size_t jj = 0; jj < ii; jj++) {
      double rbf = RBF(training_points_[ii], training_points_[jj]);
      K11_(ii, jj) = rbf;
      K11_(jj, ii) = rbf;
    }

    // Handle diagonal.
    K11_(ii, ii) = 1.0 + noise_sd_*noise_sd_;
  }
}

// Compute cross covariance vector of this query point against
// all the training data.
void GPSurfaceEstimator::CrossCovariance(const pcl::PointXYZ& query) {
  K12_ = Eigen::VectorXd::Zero(training_points_.size());
  for (size_t ii = 0; ii < training_points_.size(); ii++)
    K12_(ii) = RBF(training_points_[ii], query);
}

