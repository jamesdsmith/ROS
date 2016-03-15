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
// Defining the PointCloudFilter class.
//
///////////////////////////////////////////////////////////////////////////////

#include <point_cloud_filter/point_cloud_filter.h>

// Constructor/destructor.
PointCloudFilter::PointCloudFilter() : initialized_(false) {}
PointCloudFilter::~PointCloudFilter() {}

// Initialize.
bool PointCloudFilter::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "point_cloud_filter");

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
bool PointCloudFilter::LoadParameters(const ros::NodeHandle& n) {
  if (!ros::param::get("/uav_slam/filter/voxel_leaf_size", voxel_leaf_size_))
    return false;
  if (!ros::param::get("/uav_slam/filter/sor_knn", sor_knn_))
    return false;
  if (!ros::param::get("/uav_slam/filter/sor_zscore", sor_zscore_))
    return false;

  return true;
}

// Register callbacks.
bool PointCloudFilter::RegisterCallbacks(const ros::NodeHandle& n) {
  return true;
}

// Filter the incoming point cloud.
PointCloud::Ptr PointCloudFilter::Filter(const PointCloud::ConstPtr& cloud) {
  PointCloud::Ptr filtered_cloud(new PointCloud);

  // Voxel grid filter.
  pcl::VoxelGrid<pcl::PointXYZ> grid_filter;
  grid_filter.setInputCloud(cloud);
  grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  grid_filter.filter(*filtered_cloud);

  // Statistical outlier removal.
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
  sor_filter.setInputCloud(filtered_cloud);
  sor_filter.setMeanK(sor_knn_);
  sor_filter.setStddevMulThresh(sor_zscore_);
  sor_filter.filter(*filtered_cloud);

  return filtered_cloud;
}
