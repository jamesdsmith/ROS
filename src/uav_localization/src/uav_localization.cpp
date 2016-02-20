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
UAVLocalization::UAVLocalization() : initialized_(false), first_step_(true) {}
UAVLocalization::~UAVLocalization() {}

// Initialize.
bool UAVLocalization::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "uav_localization");
  integrated_rotation_ = Eigen::Matrix3d::Identity();
  integrated_translation_ = Eigen::Vector3d::Zero();

  if (!odometry_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize UAVOdometry.", name_.c_str());
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

  initialized_ = true;
  return true;
}

// Load parameters.
bool UAVLocalization::LoadParameters(const ros::NodeHandle& n) {
  return true;
}

// Register callbacks.
bool UAVLocalization::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle node(n);

  // Subscriber.
  point_cloud_subscriber_ =
    node.subscribe<PointCloud>("/velodyne_points", 10,
                               &UAVLocalization::AddPointCloudCallback, this);

  // Publishers.
  scan_publisher_full_ = node.advertise<PointCloud>("robot", 10, false);
  scan_publisher_filtered_ = node.advertise<PointCloud>("filtered", 10, false);

  // Timer.
  timer_ = n.createTimer(ros::Duration(0.25), &UAVLocalization::TimerCallback, this);

  return true;
}

// Timer callback.
void UAVLocalization::TimerCallback(const ros::TimerEvent& event) {
  std::vector<PointCloud::ConstPtr> sorted_clouds;
  synchronizer_.GetSorted(sorted_clouds);

  for (size_t ii = 0; ii < sorted_clouds.size(); ii++) {
    const PointCloud::ConstPtr cloud = sorted_clouds[ii];
    PointCloud::Ptr transformed_cloud(new PointCloud);
    PointCloud::Ptr neighbors(new PointCloud);

    // Calculate odometry.
    odometry_.SetIntegratedRotation(integrated_rotation_);
    odometry_.SetIntegratedTranslation(integrated_translation_);
    odometry_.UpdateOdometry(cloud);
    PointCloud::Ptr filtered_cloud = odometry_.GetPreviousCloud();

    // Extract initial transform.
    const Eigen::Matrix3d initial_rotation = odometry_.GetIntegratedRotation();
    const Eigen::Vector3d initial_translation = odometry_.GetIntegratedTranslation();

    // Transform cloud into world frame.
    Eigen::Matrix4d initial_tf = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d refined_tf = Eigen::Matrix4d::Identity();
    initial_tf.block(0, 0, 3, 3) = initial_rotation;
    initial_tf.block(0, 3, 3, 1) = initial_translation;
    pcl::transformPointCloud(*filtered_cloud, *transformed_cloud, initial_tf);

    if (!first_step_) {
      // Grab nearest neighbors in map.
      if (!mapper_.NearestNeighbors(transformed_cloud, neighbors)) {
        ROS_ERROR("%s: UAVMapper could not find nearest neighbors.", name_.c_str());
        continue;
      }

      // Refine initial guess.
      RefineTransformation(neighbors, cloud, initial_tf, refined_tf);
    } else {
      first_step_ = false;
      refined_tf = initial_tf;
    }

    // Update integrated rotation and translation.
    integrated_rotation_ = refined_tf.block(0, 0, 3, 3);
    integrated_translation_ = refined_tf.block(0, 3, 3, 1);

    // Add to the map.
    pcl::transformPointCloud(*cloud, *transformed_cloud, refined_tf);
    mapper_.InsertPoints(*transformed_cloud);

    // Publish transform and point clouds.
    stamp_.fromNSec(cloud->header.stamp * 1000);
    PublishPose();
    PublishFullScan(cloud);
    PublishFilteredScan(filtered_cloud);
  }
}


// Point cloud callback.
void UAVLocalization::AddPointCloudCallback(const PointCloud::ConstPtr& cloud) {
  synchronizer_.AddMessage(cloud);
}

// Run ICP.
void UAVLocalization::RefineTransformation(const PointCloud::Ptr& map,
                                           const PointCloud::ConstPtr& scan,
                                           const Eigen::Matrix4d& initial_tf,
                                           Eigen::Matrix4d& refined_tf) {
  // Setup.
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(scan);
  icp.setInputTarget(map);
  icp.setMaxCorrespondenceDistance(0.5);
  icp.setMaximumIterations(10);
  icp.setTransformationEpsilon(1e-12);
  icp.setEuclideanFitnessEpsilon(1e-8);
  icp.setRANSACOutlierRejectionThreshold(0.5);

  // Align.
  PointCloud aligned_cloud;
  icp.align(aligned_cloud, initial_tf.cast<float>());
  refined_tf = icp.getFinalTransformation().cast<double>();
}

// Publish transform.
void UAVLocalization::PublishPose() {
  geometry_msgs::TransformStamped stamped;

  Eigen::Quaterniond quat(integrated_rotation_);
  quat.normalize();

  stamped.transform.rotation.x = quat.x();
  stamped.transform.rotation.y = quat.y();
  stamped.transform.rotation.z = quat.z();
  stamped.transform.rotation.w = quat.w();
  stamped.transform.translation.x = integrated_translation_(0);
  stamped.transform.translation.y = integrated_translation_(1);
  stamped.transform.translation.z = integrated_translation_(2);

  stamped.header.stamp = stamp_;
  stamped.header.frame_id = "world";
  stamped.child_frame_id = "robot";
  transform_broadcaster_.sendTransform(stamped);
}


// Publish full scan.
void UAVLocalization::PublishFullScan(const PointCloud::ConstPtr& cloud) {
  PointCloud msg = *cloud;
  msg.header.stamp = stamp_.toNSec() / 1000;
  msg.header.frame_id = "robot";
  scan_publisher_full_.publish(msg);
}

// Publish filtered scan.
void UAVLocalization::PublishFilteredScan(const PointCloud::Ptr& cloud) {
  PointCloud msg = *cloud;
  msg.header.stamp = stamp_.toNSec() / 1000;
  msg.header.frame_id = "robot";
  scan_publisher_filtered_.publish(msg);
}
