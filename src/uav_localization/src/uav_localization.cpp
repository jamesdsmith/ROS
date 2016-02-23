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
// Definitions for the UAVLocalization class.
//
///////////////////////////////////////////////////////////////////////////////

#include <uav_localization/uav_localization.h>

// Constructor/destructor.
UAVLocalization::UAVLocalization() : initialized_(false) {}
UAVLocalization::~UAVLocalization() {}

// Initialize.
bool UAVLocalization::Initialize(const ros::NodeHandle& n,
                                 UAVMapper *mapper, UAVOdometry *odometry) {
  name_ = ros::names::append(n.getNamespace(), "uav_localization");
  refined_rotation_ = Eigen::Matrix3d::Identity();
  refined_translation_ = Eigen::Vector3d::Zero();

  odometry_ = odometry;
  mapper_ = mapper;

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Load parameters.
bool UAVLocalization::LoadParameters(const ros::NodeHandle& n) {
  return true;
}

// Register callbacks.
bool UAVLocalization::RegisterCallbacks(const ros::NodeHandle& n) {
  return true;
}

// Get refined rotation and translation.
Eigen::Matrix3d& UAVLocalization::GetRefinedRotation() {
  return refined_rotation_;
}

Eigen::Vector3d& UAVLocalization::GetRefinedTranslation() {
  return refined_translation_;
}

// Localize a new scan against the map.
void UAVLocalization::Localize(const PointCloud::ConstPtr& scan) {
  PointCloud::Ptr neighbors(new PointCloud);
  PointCloud::Ptr transformed(new PointCloud);

  // Calculate odometry.
  odometry_->SetIntegratedRotation(refined_rotation_);
  odometry_->SetIntegratedTranslation(refined_translation_);
  odometry_->UpdateOdometry(scan);
  PointCloud::Ptr filtered = odometry_->GetPreviousCloud();

  // Extract initial transform and update odometry estimate.
  const Eigen::Matrix3d initial_rotation = odometry_->GetIntegratedRotation();
  const Eigen::Vector3d initial_translation = odometry_->GetIntegratedTranslation();

  // Transform cloud into world frame.
  Eigen::Matrix4d initial_tf = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d refined_tf = Eigen::Matrix4d::Identity();
  initial_tf.block(0, 0, 3, 3) = initial_rotation;
  initial_tf.block(0, 3, 3, 1) = initial_translation;
  pcl::transformPointCloud(*filtered, *transformed, initial_tf);

  if (mapper_->Size() > 0) {
    // Grab nearest neighbors in map.
    if (!mapper_->NearestNeighbors(transformed, neighbors)) {
      ROS_ERROR("%s: UAVMapper could not find nearest neighbors.", name_.c_str());
      return;
    }

    // Refine initial guess.
    RefineTransformation(neighbors, transformed, initial_tf, refined_tf);
  } else {
    refined_tf = initial_tf;
  }

  // Update integrated rotation and translation.
  refined_rotation_ = refined_tf.block(0, 0, 3, 3);
  refined_translation_ = refined_tf.block(0, 3, 3, 1);

  // Add to the map.
  pcl::transformPointCloud(*scan, *transformed, refined_tf);
  mapper_->InsertPoints(*transformed);
}

// Refine initial guess.
void UAVLocalization::RefineTransformation(const PointCloud::Ptr& target,
                                           const PointCloud::Ptr& source,
                                           const Eigen::Matrix4d& initial_tf,
                                           Eigen::Matrix4d& refined_tf) {

  // Setup.
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(0.5);
  icp.setMaximumIterations(10);
  icp.setTransformationEpsilon(1e-12);
  icp.setEuclideanFitnessEpsilon(1e-8);
  icp.setRANSACOutlierRejectionThreshold(0.5);

  // Align.
  PointCloud aligned_scan;
  icp.align(aligned_scan);
  refined_tf = icp.getFinalTransformation().cast<double>() * initial_tf;
}
