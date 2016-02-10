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

  return true;
}

// Point cloud callback.
void UAVMapper::AddPointCloudCallback(const PointCloud::ConstPtr& cloud) {
  // Get odometry estimate.
  tf::Transform incremental_tf = PointCloudOdometry(cloud);
  integrated_tf_ *= incremental_tf;

  // Send transform.
  geometry_msgs::TransformStamped stamped;
  tf::Quaternion quaternion = integrated_tf_.getRotation();
  tf::Vector3 translation = integrated_tf_.getOrigin();

  stamped.transform.rotation.x = quaternion.getAxis().getX();
  stamped.transform.rotation.y = quaternion.getAxis().getY();
  stamped.transform.rotation.z = quaternion.getAxis().getZ();
  stamped.transform.rotation.w = quaternion.getW();
  stamped.transform.translation.x = translation.getX();
  stamped.transform.translation.y = translation.getY();
  stamped.transform.translation.z = translation.getZ();

  stamped.header.stamp = ros::Time().fromNSec(cloud->header.stamp);
  stamped.header.frame_id = "velodyne";
  stamped.child_frame_id = "world";
  transform_broadcaster_.sendTransform(stamped);

  // Update pointer to last point cloud.
  previous_cloud_ = cloud;
}

// Calculate incremental transform.
tf::Transform UAVMapper::PointCloudOdometry(const PointCloud::ConstPtr& cloud) {
  if (!previous_cloud_) {
    tf::Transform identity_tf;
    identity_tf.setIdentity();
    return identity_tf;
  }

  // Setup.
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(previous_cloud_);
  icp.setInputTarget(cloud);

  // Align.
  PointCloud aligned_cloud;
  icp.align(aligned_cloud);

  // Get transform.
  Eigen::Matrix4f pose = icp.getFinalTransformation();
  std::cout << pose << std::endl;

  return tf::Transform(tf::Matrix3x3(pose(0, 0), pose(0, 1), pose(0, 2),
                                     pose(1, 0), pose(1, 1), pose(1, 2),
                                     pose(2, 0), pose(2, 1), pose(2, 2)),
                       tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
}
