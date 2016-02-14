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
// Start up a new UAVMapper node.
//
///////////////////////////////////////////////////////////////////////////////

#include <uav_mapper/uav_mapper.h>

// Constructor/destructor.
UAVMapper::~UAVMapper() {}
UAVMapper::UAVMapper() {}

// Initialize.
bool UAVMapper::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "uav_mapper");
  integrated_rotation_.setIdentity();
  integrated_translation_ = Eigen::Vector3d::Zero();

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
bool UAVMapper::LoadParameters(const ros::NodeHandle& n) { return true; }

// Register callbacks.
bool UAVMapper::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle node(n);

  point_cloud_subscriber_ =
    node.subscribe<PointCloud>("/velodyne_points", 10,
                   &UAVMapper::AddPointCloudCallback, this);
  point_cloud_publisher_ = node.advertise<PointCloud>("robot", 10, false);
  point_cloud_publisher_filtered_ = node.advertise<PointCloud>("filtered", 10, false);
  point_cloud_publisher_aligned_ = node.advertise<PointCloud>("aligned", 10, false);

  return true;
}

// Point cloud callback.
void UAVMapper::AddPointCloudCallback(const PointCloud::ConstPtr& cloud) {
  // Get odometry estimate.
  PointCloudOdometry(cloud);

  // Send transform.
  geometry_msgs::TransformStamped stamped;

  stamped.transform.rotation.x = integrated_rotation_.x();
  stamped.transform.rotation.y = integrated_rotation_.y();
  stamped.transform.rotation.z = integrated_rotation_.z();
  stamped.transform.rotation.w = integrated_rotation_.w();
  stamped.transform.translation.x = integrated_translation_(0);
  stamped.transform.translation.y = integrated_translation_(1);
  stamped.transform.translation.z = integrated_translation_(2);

  stamped.header.stamp.fromNSec(cloud->header.stamp * 1000);
  stamped.header.frame_id = "world";
  stamped.child_frame_id = "robot";
  transform_broadcaster_.sendTransform(stamped);

  // Send point cloud.
  PointCloud msg = *cloud;
  msg.header.frame_id = "robot";
  point_cloud_publisher_.publish(msg);
}

// Calculate incremental transform.
Eigen::Matrix4f UAVMapper::PointCloudOdometry(const PointCloud::ConstPtr& cloud) {
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
  sor_filter.setStddevMulThresh(0.3);
  sor_filter.filter(*sor_cloud);

  // Handle base case.
  if (!previous_cloud_) {
    previous_cloud_ = sor_cloud;
  }

  // Setup.
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(sor_cloud);
  icp.setInputTarget(previous_cloud_);
  icp.setMaxCorrespondenceDistance(1.0);
  icp.setMaximumIterations(10);
  icp.setTransformationEpsilon(1e-8);
  icp.setRANSACOutlierRejectionThreshold(0.25);

  // Align.
  PointCloud aligned_cloud;
  icp.align(aligned_cloud);

  sor_cloud->header.frame_id = "robot";
  aligned_cloud.header.frame_id = "robot";
  point_cloud_publisher_filtered_.publish(sor_cloud);
  point_cloud_publisher_aligned_.publish(aligned_cloud);

  // Update pointer to last point cloud.
  previous_cloud_ = sor_cloud;

  // Get transform.
  Eigen::Matrix4f pose = icp.getFinalTransformation();
  Eigen::Matrix3d rotation = pose.block(0, 0, 3, 3).cast<double>();
  Eigen::Vector3d translation = pose.block(0, 3, 3, 1).cast<double>();

  integrated_rotation_ = rotation * integrated_rotation_;
  integrated_translation_ = rotation * integrated_translation_ + translation;
}
