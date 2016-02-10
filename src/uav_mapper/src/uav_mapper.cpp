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

#include <ros/ros.h>

// Constructor/destructor.
UAVMapper::UAVMapper() {}
UAVMapper::~UAVMapper() {}

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
  ros::NodeHandle nl(n);

  point_cloud_subscriber_ =
    nl.subscribe("/velodyne_points", 10,
                 &UAVMapper::AddPointCloudCallback, this);
  transform_publisher_ =
    nl.advertise<geometry_msgs::TransformStamped>("", 10, false);

  return true;
}

// Point cloud callback.
void UAVMapper::AddPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
  if (point_cloud_publisher_.getNumSubscribers() == 0)
    return;

  // Get odometry estimate.
  geometry_msgs::TransformStamped tf = PointCloudOdometry(cloud);
  transform_publisher_.publish(tf);
}

// Calculate transform from this cloud to the w.
