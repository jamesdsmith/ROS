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
 * Author: James Smith   ( james.smith@berkeley.edu )
 */

#ifndef BSFM_DEPTH_MAP_H
#define BSFM_DEPTH_MAP_H

#include <utils/image/image.h>
#include <Eigen/Dense>

using Eigen::Vector3d;

namespace bsfm {

  class DepthMap : public Image {
  public:
    using Image::Image;

    // Constructors.
    DepthMap();
    DepthMap(bool inverted);
    
    // Helpers.
    Vector3d Unproject(size_t u, size_t v) const;
    bool SaturatedAt(size_t u, size_t v) const;

    // Setters.
    void SetInverted(bool value);

    // Getters.
    ushort GetValue(size_t u, size_t v) const;
    bool IsInverted() const;

  private:
    bool inverted_;
  }; //\ class DepthMap

  // ------------------------- IMPLEMENTATION --------------------------------

  DepthMap::DepthMap()
    : inverted_(false) {}

  DepthMap::DepthMap(bool inverted)
    : inverted_(inverted) {}

  void DepthMap::SetInverted(bool value) {
    inverted_ = value;
  }

  bool DepthMap::IsInverted() const {
    return inverted_;
  }

  ushort DepthMap::GetValue(size_t u, size_t v) const {
    ushort value = at<ushort>(v, u * 3);
    if (IsInverted()) {
        value = 65535 - value;
    }
    return value;
  }

  Vector3d DepthMap::Unproject(size_t u, size_t v) const {
    // @TODO jds: Need to load these from a parameter or something
    //            values from the left camera configuration files
    double focal_length_x = 247.357576;
    double focal_length_y = 247.390025;
    double cx = 153.295063;
    double cy = 116.893925;

    // data format:
    //    16 bits
    //    top 9 bits, integer value
    //    bottom 7 bits, decimal
    ushort z_value = GetValue(u, v);
    double z = (z_value >> 7) + (z_value & 127) / 128.0;

    double x = (u - cx) * z / focal_length_x;
    double y = (v - cy) * z / focal_length_y;

    Vector3d point = Vector3d(x, y, z);
    return point;
  }

  bool DepthMap::SaturatedAt(size_t u, size_t v) const {
    return GetValue(u, v) <= 0 || GetValue(u, v) >= 65535;
  }

} //\ namespace bsfm

#endif
