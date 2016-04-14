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
// This defines the uav_localization node.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef UAV_LOCALIZATION_H
#define UAV_LOCALIZATION_H

#include <ros/ros.h>
#include <message_synchronizer/message_synchronizer.h>
#include <utils/math/transform_3d.h>
#include <uav_odometry/uav_odometry.h>
#include <uav_mapper/uav_mapper.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Dense>
#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace math;

class UAVLocalization {
 public:
  explicit UAVLocalization();
  ~UAVLocalization();

  bool Initialize(const ros::NodeHandle& n,
                  UAVMapper *mapper, UAVOdometry *odometry);

  // Localize against the map.
  void Localize(const PointCloud::ConstPtr& cloud);

  // Get transforms.
  Transform3D& GetRefinedTransform();
  Transform3D& GetOdometryTransform();

 private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Refine initial guess.
  void RefineTransformation(const PointCloud::Ptr& target,
                            const PointCloud::Ptr& source);

  // Member variables.
  UAVMapper *mapper_;
  UAVOdometry *odometry_;
  Transform3D refined_transform_;
  Transform3D odometry_transform_;

  double ransac_thresh_, tf_epsilon_, corr_dist_;
  int max_iters_;
  bool initialized_, rough_alignment_;
  std::string name_;
};

#endif
