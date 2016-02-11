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
UAVMapper::UAVMapper() { integrated_tf_.setIdentity(); }

// Initialize.
bool UAVMapper::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "uav_mapper");

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

  return true;
}

// Point cloud callback.
void UAVMapper::AddPointCloudCallback(const PointCloud::ConstPtr& cloud) {
  // Get odometry estimate.
  Eigen::Matrix4f incremental_tf = PointCloudOdometry(cloud);
  integrated_tf_ *= incremental_tf;

  // Extract quaternion from integrated_tf_.
  float qw = std::sqrt(1.0 + integrated_tf_(0, 0) + integrated_tf_(1, 1) +
                       integrated_tf_(2, 2)) / 2.0;
  float qx = (integrated_tf_(2, 1) - integrated_tf_(1, 2)) / (4.0 * qw);
  float qy = (integrated_tf_(0, 2) - integrated_tf_(2, 0)) / (4.0 * qw);
  float qz = (integrated_tf_(1, 0) - integrated_tf_(0, 1)) / (4.0 * qw);

  // Send transform.
  geometry_msgs::TransformStamped stamped;

  stamped.transform.rotation.x = qx;
  stamped.transform.rotation.y = qy;
  stamped.transform.rotation.z = qz;
  stamped.transform.rotation.w = qw;
  stamped.transform.translation.x = integrated_tf_(0, 3);
  stamped.transform.translation.y = integrated_tf_(1, 3);
  stamped.transform.translation.z = integrated_tf_(2, 3);

  stamped.header.stamp = ros::Time().fromNSec(cloud->header.stamp * 1000);
  stamped.header.frame_id = "robot";
  stamped.child_frame_id = "world";
  transform_broadcaster_.sendTransform(stamped);

  // Send point cloud.
  PointCloud::Ptr msg(new PointCloud);
  msg->header = cloud->header;
  msg->header.frame_id = "robot";
  msg->points = cloud->points;
  point_cloud_publisher_.publish(msg);

  // Update pointer to last point cloud.
  previous_cloud_ = cloud;
}

// Calculate incremental transform.
Eigen::Matrix4f UAVMapper::PointCloudOdometry(const PointCloud::ConstPtr& cloud) {
  if (!previous_cloud_) {
    ROS_INFO("[DEBUG] previous_cloud_ null. Returning identity transform.");
    Eigen::Matrix4f identity_tf = Eigen::Matrix4f::Identity();
    return identity_tf;
  }

  // Setup.
  ROS_INFO("[DEBUG] Just got a new point cloud. Running icp.");
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(previous_cloud_);
  icp.setInputTarget(cloud);

  // Align.
  PointCloud aligned_cloud;
  icp.align(aligned_cloud);

  // Get transform.
  Eigen::Matrix4f pose = icp.getFinalTransformation();
  ROS_INFO("[DEBUG] icp completed. Returning a pose.");
  return pose;
}
