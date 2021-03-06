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
// This defines the DepthCloudProjector class.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DEPTH_CLOUD_PROJECTOR_H
#define DEPTH_CLOUD_PROJECTOR_H

#include <memory>
#include <ros/ros.h>
#include <utils/image/depth_map.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Dense>
#include <cmath>
#include <dji_guidance/multi_image.h>
#include <utils/math/transform_3d.h>
#include <point_cloud_filter/point_cloud_filter.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//using namespace math;

class DepthCloudProjector {
 public:
  explicit DepthCloudProjector();
  ~DepthCloudProjector();

  bool Initialize(const ros::NodeHandle& n);

 private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callbacks.
  void DepthMapCallback(const sensor_msgs::Image& map);
  void MultiImageCallback(const dji_guidance::multi_image::ConstPtr& msg);

  math::Transform3D GetRotationFor(int index);
  math::Transform3D GetOffsetFor(int index);

  // Publishers/subscribers.
  ros::Publisher cloud_pub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber multi_img_sub_;

  // Time stamp.
  ros::Time stamp_;

  bool initialized_;
  std::string name_;
};

#endif
