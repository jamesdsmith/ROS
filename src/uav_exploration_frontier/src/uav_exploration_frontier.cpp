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
// Definitions for the UAVSlam class.
//
///////////////////////////////////////////////////////////////////////////////

#include <uav_slam/uav_slam.h>

// Constructor/destructor.
UAVSlam::UAVSlam() : initialized_(false) {}
UAVSlam::~UAVSlam() {}

// Initialize.
bool UAVSlam::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "uav_slam");

  if (!odometry_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize UAVOdometry.", name_.c_str());
    return false;
  }

  if (!mapper_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize UAVMapper.", name_.c_str());
    return false;
  }

  if (!localization_.Initialize(n, &mapper_, &odometry_)) {
    ROS_ERROR("%s: Failed to initialize UAVLocalization.", name_.c_str());
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
bool UAVSlam::LoadParameters(const ros::NodeHandle& n) {
  // Input/output paramaters.
  if (!ros::param::get("/uav_slam/io/scanner_topic", scanner_topic_))
    return false;
  if (!ros::param::get("/uav_slam/io/filtered_topic", filtered_topic_))
    return false;
  if (!ros::param::get("/uav_slam/io/unfiltered_topic", unfiltered_topic_))
    return false;
  if (!ros::param::get("/uav_slam/io/world_frame", world_frame_))
    return false;
  if (!ros::param::get("/uav_slam/io/odometry_frame", odometry_frame_))
    return false;
  if (!ros::param::get("/uav_slam/io/localized_frame", localized_frame_))
    return false;


  return true;
}

// Register callbacks.
bool UAVSlam::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle node(n);

  // Subscriber.
  point_cloud_subscriber_ =
    node.subscribe<PointCloud>(scanner_topic_.c_str(), 20,
                               &UAVSlam::AddPointCloudCallback, this);

  // Publishers.
  scan_publisher_full_ =
    node.advertise<PointCloud>(unfiltered_topic_.c_str(), 10, false);
  scan_publisher_filtered_ =
    node.advertise<PointCloud>(filtered_topic_.c_str(), 10, false);

  // Timer.
  timer_ = n.createTimer(ros::Duration(0.1), &UAVSlam::TimerCallback, this);

  return true;
}

// Timer callback.
void UAVSlam::TimerCallback(const ros::TimerEvent& event) {
  std::vector<PointCloud::ConstPtr> sorted_clouds;
  synchronizer_.GetSorted(sorted_clouds);

  for (size_t ii = 0; ii < sorted_clouds.size(); ii++) {
    const PointCloud::ConstPtr cloud = sorted_clouds[ii];

    // Localize.
    localization_.Localize(cloud);

    // Publish.
    stamp_.fromNSec(cloud->header.stamp * 1000);
    PublishPose(localization_.GetRefinedTransform(), localized_frame_);
    PublishPose(localization_.GetOdometryTransform(), odometry_frame_);
    PublishFullScan(cloud);
    PublishFilteredScan(odometry_.GetPreviousCloud());
  }
}


// Point cloud callback.
void UAVSlam::AddPointCloudCallback(const PointCloud::ConstPtr& cloud) {
  synchronizer_.AddMessage(cloud);
}

// Publish refimed transform.
void UAVSlam::PublishPose(const Transform3D& transform,
                          const std::string& child_frame_id) {
  geometry_msgs::TransformStamped stamped;

  Eigen::Matrix3d rotation = transform.GetRotation();
  Eigen::Vector3d translation = transform.GetTranslation();

  Eigen::Quaterniond quat(rotation);
  quat.normalize();

  stamped.transform.rotation.x = quat.x();
  stamped.transform.rotation.y = quat.y();
  stamped.transform.rotation.z = quat.z();
  stamped.transform.rotation.w = quat.w();
  stamped.transform.translation.x = translation(0);
  stamped.transform.translation.y = translation(1);
  stamped.transform.translation.z = translation(2);

  stamped.header.stamp = stamp_;
  stamped.header.frame_id = world_frame_.c_str();
  stamped.child_frame_id = child_frame_id.c_str();
  transform_broadcaster_.sendTransform(stamped);
}

// Publish full scan.
void UAVSlam::PublishFullScan(const PointCloud::ConstPtr& cloud) {
  if (scan_publisher_full_.getNumSubscribers() == 0)
    return;

  PointCloud msg = *cloud;
  msg.header.stamp = stamp_.toNSec() / 1000;
  msg.header.frame_id = localized_frame_.c_str();
  scan_publisher_full_.publish(msg);
}

// Publish filtered scan.
void UAVSlam::PublishFilteredScan(const PointCloud::Ptr& cloud) {
  if (scan_publisher_filtered_.getNumSubscribers() == 0)
    return;

  PointCloud msg = *cloud;
  msg.header.stamp = stamp_.toNSec() / 1000;
  msg.header.frame_id = localized_frame_.c_str();
  scan_publisher_filtered_.publish(msg);
}
