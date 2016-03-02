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
// Definitions for the DepthCloudProjector class.
//
///////////////////////////////////////////////////////////////////////////////

#include <depth_cloud_projector/depth_cloud_projector.h>
#include <mapper/mapper.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv/cv.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// Constructor/destructor.
DepthCloudProjector::DepthCloudProjector() : initialized_(false) {}
DepthCloudProjector::~DepthCloudProjector() {}

// Initialize.
bool DepthCloudProjector::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "depth_cloud_projector");

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
bool DepthCloudProjector::LoadParameters(const ros::NodeHandle& n) {
  return true;
}

// Register callbacks.
bool DepthCloudProjector::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle node(n);

  // Subscriber.
  depth_sub_ =
    node.subscribe("/guidance/depth_image", 10,
                   &DepthCloudProjector::DepthMapCallback, this);

  // Publishers.
  cloud_pub_ = node.advertise<PointCloud>("/mapper/cloud", 10, false);

  return true;
}

// Point cloud callback.
void DepthCloudProjector::DepthMapCallback(const sensor_msgs::Image& map) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(map, sensor_msgs::image_encodings::MONO16);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat depth8(320, 240, CV_8UC1);
  cv_ptr->image.convertTo(depth8, CV_8UC1);

  DepthMap dm(depth8);
  dm.SetInverted(false);
  Mapper m(true);

  PointCloud cl = m.ProjectDepthMap(dm);

  std::cout << "Projecting " << cl.size() << " points" << std::endl;

  cl.header.frame_id = "guidance";
  cl.header.stamp = map.header.stamp.toNSec() / 1000;

  cloud_pub_.publish(cl.makeShared());
}
