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
// The Transform3D class. Keep track of 3D rigid transformations.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef UTILS_MATH_TRANSFORM_3D
#define UTILS_MATH_TRANSFORM_3D

#include <Eigen/Dense>

typedef Eigen::Matrix<double, 3, 4> Matrix34d;

namespace math {

  class Transform3D {
  public:
    ~Transform3D() {}

    // Constructors.
    Transform3D() {
      rotation_ = Eigen::Matrix3d::Identity();
      translation_ = Eigen::Vector3d::Zero();
    }
    Transform3D(const Eigen::Matrix4d& tf) {
      rotation_ = tf.block(0, 0, 3, 3);
      translation_ = tf.block(0, 3, 3, 1);
    }
    Transform3D(const Eigen::Matrix3d& rotation,
                const Eigen::Vector3d& translation) {
      rotation_ = rotation;
      translation_ = translation;
    }

    // Getters.
    Eigen::Matrix3d GetRotation() const { return rotation_; }
    Eigen::Vector3d GetTranslation() const { return translation_; }
    Eigen::Matrix4d GetTransform() const {
      Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
      tf.block(0, 0, 3, 3) = rotation_;
      tf.block(0, 3, 3, 1) = translation_;
      return tf;
    }
    Matrix34d Dehomogenize() const {
      Matrix34d P;
      P.block(0, 0, 3, 3) = rotation_;
      P.block(0, 3, 3, 1) = translation_;
      return P;
    }

    // Setters.
    void SetIdentity() {
      rotation_ = Eigen::Matrix3d::Identity();
      translation_ = Eigen::Vector3d::Zero();
    }
    void SetRotation(Eigen::Matrix3d& rotation) {
      rotation_ = rotation;
    }
    void SetTranslation(Eigen::Vector3d& translation) {
      translation_ = translation;
    }

    // Operators.
    Transform3D operator=(const Transform3D& rhs) {
      if (this == &rhs)
        return *this;

      rotation_ = rhs.GetRotation();
      translation_ = rhs.GetTranslation();
      return *this;
    }
    Transform3D operator*(const Transform3D& rhs) {
      Eigen::Matrix3d rotation = rotation_ * rhs.rotation_;
      Eigen::Vector3d translation =
        rotation_ * rhs.translation_ + translation_;

      Transform3D product(rotation, translation);
      return product;
    }
    Eigen::Vector3d operator*(const Eigen::Vector3d& rhs) {
      return rotation_ * rhs + translation_;
    }
    void operator*=(const Transform3D& rhs) {
      translation_ = rotation_ * rhs.translation_ + translation_;
      rotation_ = rotation_ * rhs.rotation_;
    }
    Transform3D operator-(const Transform3D& rhs) {
      Eigen::Matrix3d rotation = rotation_.transpose() * rhs.rotation_;
      Eigen::Vector3d translation =
        rotation_.transpose() * (rhs.translation_ - translation_);

      Transform3D delta(rotation, translation);
      return delta;
    }
    bool operator==(const Transform3D& rhs) {
      return (rotation_.isApprox(rhs.rotation_, 1e-8) &&
              translation_.isApprox(rhs.translation_, 1e-8));
    }
    bool operator!=(const Transform3D& rhs) {
      return !(rotation_.isApprox(rhs.rotation_, 1e-8) &&
               translation_.isApprox(rhs.translation_, 1e-8));
    }

    // Other operations.
    Transform3D Inverse() {
      Transform3D inv(rotation_.transpose(), -rotation_.transpose() * translation_);
      return inv;
    }

    // Printing.
    void Print(const std::string& prefix = std::string()) const {
      if (!prefix.empty())
        std::cout << prefix << std::endl;
      std::cout << "Rotation: " << rotation_ << std::endl;
      std::cout << "Translation: " << translation_.transpose() << std::endl;
    }

  private:
    Eigen::Matrix3d rotation_;
    Eigen::Vector3d translation_;
  }; //\ class Transform3D
} //\ namespace math

#endif
