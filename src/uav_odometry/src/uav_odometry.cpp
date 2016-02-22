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
// Start up a new UAVOdometry node.
//
///////////////////////////////////////////////////////////////////////////////

#include <uav_odometry/uav_odometry.h>
#include <message_synchronizer/message_synchronizer.h>

// Constructor/destructor.
UAVOdometry::UAVOdometry() : initialized_(false) {
  previous_cloud_.reset(new PointCloud);
  aligned_cloud_.reset(new PointCloud);
}

UAVOdometry::~UAVOdometry() {}

// Initialize.
bool UAVOdometry::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "uav_odometry");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

// Load parameters.
bool UAVOdometry::LoadParameters(const ros::NodeHandle& n) {
  // Integrated transform.
  integrated_rotation_ = Eigen::Matrix3d::Identity();
  integrated_translation_ = Eigen::Vector3d::Zero();

  return true;
}

// Register callbacks.
bool UAVOdometry::RegisterCallbacks(const ros::NodeHandle& n) {
  return true;
}

// Get integrated transform.
Eigen::Matrix3d& UAVOdometry::GetIntegratedRotation() {
  return integrated_rotation_;
}

Eigen::Vector3d& UAVOdometry::GetIntegratedTranslation() {
  return integrated_translation_;
}

// Get previous cloud.
PointCloud::Ptr UAVOdometry::GetPreviousCloud() {
  return previous_cloud_;
}

PointCloud::Ptr UAVOdometry::GetAlignedCloud() {
  return aligned_cloud_;
}

// Reset integrated transform.
void UAVOdometry::SetIntegratedRotation(Eigen::Matrix3d& rotation) {
  integrated_rotation_ = rotation;
}

void UAVOdometry::SetIntegratedTranslation(Eigen::Vector3d& translation) {
  integrated_translation_ = translation;
}


// Update odometry estimate with next point cloud.
void UAVOdometry::UpdateOdometry(const PointCloud::ConstPtr& cloud) {
  RunICP(cloud);
}


// Calculate incremental transform.
void UAVOdometry::RunICP(const PointCloud::ConstPtr& cloud) {
  PointCloud::Ptr sor_cloud(new PointCloud);
  PointCloud::Ptr grid_cloud(new PointCloud);

  // Voxel grid filter.
  pcl::VoxelGrid<pcl::PointXYZ> grid_filter;
  grid_filter.setInputCloud(cloud);
  grid_filter.setLeafSize(0.75, 0.75, 0.75);
  grid_filter.filter(*grid_cloud);

  // Statistical outlier removal.
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
  sor_filter.setInputCloud(grid_cloud);
  sor_filter.setMeanK(30);
  sor_filter.setStddevMulThresh(1.0);
  sor_filter.filter(*sor_cloud);

  // Handle base case.
  if (!initialized_) {
    pcl::copyPointCloud(*sor_cloud, *previous_cloud_);
    initialized_ = true;
  }

  // Setup.
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(sor_cloud);
  icp.setInputTarget(previous_cloud_);
  icp.setMaxCorrespondenceDistance(0.5);
  icp.setMaximumIterations(10);
  icp.setTransformationEpsilon(1e-12);
  icp.setEuclideanFitnessEpsilon(1e-8);
  icp.setRANSACOutlierRejectionThreshold(0.5);

  // Align.
  icp.align(*aligned_cloud_);

  // Update pointer to last point cloud.
  pcl::copyPointCloud(*sor_cloud, *previous_cloud_);

  // Get transform.
  Eigen::Matrix4f pose = icp.getFinalTransformation();
  Eigen::Matrix3d rotation = pose.block(0, 0, 3, 3).cast<double>();
  Eigen::Vector3d translation = pose.block(0, 3, 3, 1).cast<double>();

  integrated_translation_ =
    integrated_rotation_ * translation + integrated_translation_;
  integrated_rotation_ = integrated_rotation_ * rotation;
}
