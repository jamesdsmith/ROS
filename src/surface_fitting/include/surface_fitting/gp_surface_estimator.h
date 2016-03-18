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
// The GPSurfaceEstimator class. Uses Gaussian process regression to learn a
// signed distance function (and hence an implicit surface) in 3D.
//
// For a detailed description of the underlying mathematics, refer to pp. 13-19
//    C. E. Rasmussen and C. K. I. Williams, "Gaussian Processes for
//    Machine Learning," MIT Press, 2006.
// which is freely available at www.GaussianProcess.org/gpml.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef GP_SURFACE_ESTIMATOR_H
#define GP_SURFACE_ESTIMATOR_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

class GPSurfaceEstimator {
 public:
  GPSurfaceEstimator();
  ~GPSurfaceEstimator();

  // Set initial points.
  bool Initialize(const ros::NodeHandle& n,
                  const std::vector<pcl::PointXYZ>& points,
                  const std::vector<double>& distances);

  // Compute signed distance and uncertainty to query point.
  void SignedDistance(const pcl::PointXYZ& query, double& distance,
                      double& variance) const;

  // Add a point to the surface.
  void AddPoint(const pcl::PointXYZ& point);

 private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  double RBF(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);
  void TrainingCovariance();
  void CrossCovariance(const pcl::PointXYZ& query);

  // Member variables.
  Eigen::MatrixXd K11_, K11_inv_;
  Eigen::VectorXd K12_, mu_training_;
  std::vector<pcl::PointXYZ> training_points_;
  double noise_sd_, gamma_;
  bool initialized_;
  std::string name_;
};

#endif
