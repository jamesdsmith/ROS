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
// This defines the uav_odometry node.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef UAV_ODOMETRY_H
#define UAV_ODOMETRY_H

#include <ros/ros.h>
#include <message_synchronizer/message_synchronizer.h>
#include <utils/math/transform_3d.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace math;

class UAVOdometry {
 public:
  explicit UAVOdometry();
  ~UAVOdometry();

  bool Initialize(const ros::NodeHandle& n);

  // Update odometry estimate with next point cloud.
  void UpdateOdometry(const PointCloud::ConstPtr& cloud);

  // Get current pose estimate.
  Transform3D& GetIntegratedTransform();

  // Reset integrated transform.
  void SetIntegratedTransform(Transform3D& transform);
  void ResetIntegratedTransform();

  // Get previous cloud.
  PointCloud::Ptr GetPreviousCloud();
  PointCloud::Ptr GetAlignedCloud();

 private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Run ICP.
  void RunICP(const PointCloud::ConstPtr& cloud);

  // Member variables.
  Transform3D integrated_transform_;
  PointCloud::Ptr previous_cloud_;
  PointCloud::Ptr aligned_cloud_;

  double voxel_leaf_size_, sor_zscore_, ransac_thresh_, tf_epsilon_, corr_dist_;
  int sor_knn_, max_iters_;
  bool initialized_;
  std::string name_;
};

#endif
