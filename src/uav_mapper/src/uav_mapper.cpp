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
#include <message_synchronizer/message_synchronizer.h>

// Constructor/destructor.
UAVMapper::UAVMapper() : initialized_(false) { previous_cloud_.reset(new PointCloud); }
UAVMapper::~UAVMapper() {}

// Initialize.
bool UAVMapper::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "uav_mapper");
  odometry_.Initialize(n);

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
bool UAVMapper::LoadParameters(const ros::NodeHandle& n) {
  return true;
}

// Register callbacks.
bool UAVMapper::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle node(n);

  // Subscriber.
  point_cloud_subscriber_ =
    node.subscribe<PointCloud>("/velodyne_points", 10,
                               &UAVMapper::AddPointCloudCallback, this);

  // Timer.
  timer_ = n.createTimer(ros::Duration(0.25), &UAVMapper::TimerCallback, this);

  return true;
}

// Timer callback.
void UAVMapper::TimerCallback(const ros::TimerEvent& event) {
  std::vector<PointCloud::ConstPtr> sorted_clouds;
  synchronizer_.GetSorted(sorted_clouds);

  for (size_t ii = 0; ii < sorted_clouds.size(); ii++) {
    const PointCloud::ConstPtr cloud = sorted_clouds[ii];

    // Calculate odometry.
    odometry_.UpdateOdometry(cloud);

    // Extract transform.
    Eigen::Matrix3f rotation = odometry_.GetIntegratedRotation().cast<float>();
    Eigen::Vector3f translation = odometry_.GetIntegratedTranslation().cast<float>();
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    tf.block(0, 0, 3, 3) = rotation;
    tf.block(0, 3, 3, 1) = translation;

    // Transform cloud into world frame.
    pcl::transformPointCloud(*cloud, *cloud, tf);

    // Add to the map.
    
  }
}


// Point cloud callback.
void UAVMapper::AddPointCloudCallback(const PointCloud::ConstPtr& cloud) {
  synchronizer_.AddMessage(cloud);
}
