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
// This defines the PoseFilter class, which implements a flavor of Kalman
// filtering for poses in SE(3).
//
// Currently, we keep 18 states (3 for position, 3 for Euler angles, plus
// first and second derivatives) and our process model is just integrating.
// See http://capar.in.tum.de/Chair/KalmanFilter for details.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef POSE_FILTER_H
#define POSE_FILTER_H

#include <ros/ros.h>
#include <utils/math/transform_3d.h>
#include <utils/math/acceleration_filter_1d.h>

class PoseFilter {
 public:
  explicit PoseFilter();
  ~PoseFilter();

  // Setters/getters.
  void SetPose(Transform3D& pose);
  Transform3D GetPose() const;

  // Propagate and update steps.
  void Propagate(double dt);
  void UpdateIMU(double ax, double ay, double az,
                 double yaw, double pitch, double roll);
  void UpdateICP(Transform3D& pose);
  void UpdateVO(Transform3D& pose);

 private:
  AccelerationFilter1D x_, y_, z_, roll_, pitch_, yaw_;
};

// ------------------------------- IMPLEMENTATION ----------------------------- //

// Constructor/destructor.
PoseFilter::PoseFilter() {}
PoseFilter::~PoseFilter() {}

// Setters/getters.
void PoseFilter::SetPose(Transform3D& pose) {
  Eigen::Vector3d position = pose.GetTranslation();
  Eigen::Vector3d euler = math::MatrixToEulerAngles(pose.GetRotation());

  x_.SetPosition(position(0));
  y_.SetPosition(position(1));
  z_.SetPosition(position(2));
  yaw_.SetPosition(euler(0));
  pitch_.SetPosition(euler(1));
  roll_.SetPosition(euler(2));
}

Transform3D PoseFilter::GetPose() const {
  Eigen::Vector3d position(x_.GetPosition(), y_.GetPosition(), z_.GetPosition());
  Eigen::Matrix3d rotation = math::EulerAnglesToMatrix(yaw_.GetPosition(),
                                                       pitch_.GetPosition(),
                                                       roll_.GetPosition());
  Transform3D pose(rotation, position);
  return pose;
}

// Propagate and update steps.
// TODO -- Estimate actual noise characteristics!
void PoseFilter::Propagate(double dt) {
  x_.Propagate(dt);
  y_.Propagate(dt);
  z_.Propagate(dt);
  yaw_.Propagate(dt);
  pitch_.Propagate(dt);
  roll_.Propagate(dt);
}

void PoseFilter::UpdateIMU(double ax, double ay, double az,
                           double yaw, double pitch, double roll) {
  x_.UpdateAcceleration(ax);
  y_.UpdateAcceleration(ay);
  z_.UpdateAcceleration(az);
  yaw_.UpdatePosition(yaw);
  pitch_.UpdatePosition(pitch);
  roll_.UpdatePosition(roll);
}

void PoseFilter::UpdateICP(Transform3D& pose) {
  Eigen::Vector3d position = pose.GetTranslation();
  Eigen::Vector3d euler = math::MatrixToEulerAngles(pose.GetRotation());

  x_.UpdatePosition(position(0));
  y_.UpdatePosition(position(1));
  z_.UpdatePosition(position(2));
  yaw_.UpdatePosition(euler(0));
  pitch_.UpdatePosition(euler(1));
  roll_.UpdatePosition(euler(2));
}

void PoseFilter::UpdateVO(Transform3D& pose) {
  Eigen::Vector3d position = pose.GetTranslation();
  Eigen::Vector3d euler = math::MatrixToEulerAngles(pose.GetRotation());

  x_.UpdatePosition(position(0));
  y_.UpdatePosition(position(1));
  z_.UpdatePosition(position(2));
  yaw_.UpdatePosition(euler(0));
  pitch_.UpdatePosition(euler(1));
  roll_.UpdatePosition(euler(2));
}

#endif
