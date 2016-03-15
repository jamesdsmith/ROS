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
  // ICP params.
  if (!ros::param::get("/uav_slam/icp/ransac_thresh", ransac_thresh_))
    return false;
  if (!ros::param::get("/uav_slam/icp/tf_epsilon", tf_epsilon_))
    return false;
  if (!ros::param::get("/uav_slam/icp/corr_dist", corr_dist_))
    return false;
  if (!ros::param::get("/uav_slam/icp/max_iters", max_iters_))
    return false;
  if (!ros::param::get("/uav_slam/localization/rough_alignment", rough_alignment_))
    return false;

  return true;
}

// Register callbacks.
bool UAVLocalization::RegisterCallbacks(const ros::NodeHandle& n) {
  return true;
}

// Get refined transform.
Transform3D& UAVLocalization::GetRefinedTransform() {
  if (!initialized_) {
    ROS_ERROR("%s: Tried to get refined transform before initializing.",
              name_.c_str());
  }

  return refined_transform_;
}

Transform3D& UAVLocalization::GetOdometryTransform() {
  if (!initialized_) {
    ROS_ERROR("%s: Tried to get odometry transform before initializing.",
              name_.c_str());
  }

  return odometry_transform_;
}

// Localize a new (filtered) scan against the map.
void UAVLocalization::Localize(const PointCloud::ConstPtr& scan) {
  if (!initialized_) {
    ROS_ERROR("%s: Tried to localize before initializing.", name_.c_str());
    return;
  }

  // Do rough alignment using odometry (if flag is set).
  PointCloud::Ptr neighbors(new PointCloud);
  PointCloud::Ptr transformed(new PointCloud);
  if (rough_alignment_) {
    // Calculate odometry.
    odometry_->ResetIntegratedTransform();
    odometry_->UpdateOdometry(scan);

    // Extract initial transform and update odometry estimate.
    const Transform3D incremental_transform = odometry_->GetIntegratedTransform();
    odometry_transform_ *= incremental_transform;
    refined_transform_ *= incremental_transform;

    // Transform cloud into world frame.
    Eigen::Matrix4d initial_tf = refined_transform_.GetTransform();
    pcl::transformPointCloud(*scan, *transformed, initial_tf);
  } else {
    // Just set transformed to scan.
    *transformed = *scan;
  }

  // Do fine alignment.
  if (mapper_->Size() > 0) {
    // Grab nearest neighbors in map.
    if (!mapper_->NearestNeighbors(transformed, neighbors)) {
      ROS_ERROR("%s: UAVMapper could not find nearest neighbors.", name_.c_str());
      return;
    }

    // Refine initial guess.
    RefineTransformation(neighbors, transformed);
  }

  // Add to the map.
  Eigen::Matrix4d refined_tf = refined_transform_.GetTransform();
  pcl::transformPointCloud(*scan, *transformed, refined_tf);
  mapper_->InsertPoints(*transformed);
}

// Refine initial guess.
void UAVLocalization::RefineTransformation(const PointCloud::Ptr& target,
                                           const PointCloud::Ptr& source) {
  if (!initialized_) {
    ROS_ERROR("%s: Tried to refine transform before initializing.",
              name_.c_str());
    return;
  }


  // Setup.
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(corr_dist_);
  icp.setMaximumIterations(max_iters_);
  icp.setTransformationEpsilon(tf_epsilon_);
  icp.setRANSACOutlierRejectionThreshold(ransac_thresh_);

  // Align.
  PointCloud aligned_scan;
  icp.align(aligned_scan);

  Transform3D refinement(icp.getFinalTransformation().cast<double>());
  refined_transform_ = refinement * refined_transform_;
}
