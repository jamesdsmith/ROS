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
// The AccelerationFilter1D class. A simple 1D Kalman filter with a constant
// acceleration motion model and homogeneous noise characteristics.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef UTILS_MATH_ACCELERATION_FILTER_1D_H
#define UTILS_MATH_ACCELERATION_FILTER_1D_H

#include <Eigen/Dense>

typedef Eigen::Matrix<double, 1, 3> RowVector3d;

namespace math {

  class AccelerationFilter1D {
  public:
    ~AccelerationFilter1D() {}
    AccelerationFilter1D() {
      x_ = Eigen::Vector3d::Zero();
      P_ = Eigen::Matrix3d::Identity();

      // Set default motion model.
      A_ = Eigen::Matrix3d::Identity();

      // Set measurement matrices.
      Hx_ << 1.0, 0.0, 0.0;
      Ha_ << 0.0, 0.0, 1.0;

      // Set process noise.
      Q_ = Eigen::Matrix3d::Identity();
    }

    // Setters.
    void SetPosition(double x) { x_(0) = x; }
    void SetVelocity(double v) { x_(1) = v; }
    void SetAcceleration(double a) { x_(2) = a; }
    void SetState(Eigen::Vector3d& state) { x_ = state; }

    // Getters.
    double GetPosition() const { return x_(0); }
    double GetVelocity() const { return x_(1); }
    double GetAcceleration() const { return x_(2); }
    Eigen::Vector3d GetState() const { return x_; }

    double GetPositionVariance() const { return P_(0, 0); }
    double GetVelocityVariance() const { return P_(1, 1); }
    double GetAccelerationVariance() const { return P_(2, 2); }
    Eigen::Matrix3d GetCovariance() const { return P_; }

    // Propagate.
    void Propagate(double dt) {
      A_(0, 1) = dt;
      A_(1, 2) = dt;
      A_(0, 2) = 0.5 * dt;

      // Update state.
      x_ = A_ * x_;

      // Update covariance.
      P_ = A_ * P_ * A_.transpose() + Q_;
    }

    // Update. Provide a scalar measurement and variance.
    void UpdatePosition(double zx, double R = 1.0) {
      Eigen::Vector3d K = P_ * Hx_.transpose() / (Hx_ * P_ * Hx_.transpose() + R);
      x_ += K * (zx - Hx_ * x_);
      P_ -= K * Hx_ * P_;
    }
    void UpdateAcceleration(double za, double R = 1.0) {
      Eigen::Vector3d K = P_ * Ha_.transpose() / (Ha_ * P_ * Ha_.transpose() + R);
      x_ += K * (za - Ha_ * x_);
      P_ -= K * Ha_ * P_;
    }

  private:
    Eigen::Vector3d x_; // state
    Eigen::Matrix3d P_; // covariance
    Eigen::Matrix3d A_; // motion model
    Eigen::Matrix3d Q_; // process noise
    RowVector3d Hx_; // position measurement
    RowVector3d Ha_; // acceleration measurement

  }; //\ class AccelerationFilter1D
} //\ namespace math

#endif
