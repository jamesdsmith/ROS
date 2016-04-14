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
// This class defines the Array3D class, which is a very simple wrapper around
// std::vector to support 3D indexing.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef UTILS_ARRAY_3D_H
#define UTILS_ARRAY_3D_H

#include <vector>
#include <iostream>

template<typename T>
class Array3D {
public:
  ~Array3D() {}
  Array3D(size_t length, size_t width, size_t height);

  // Accessor.
  inline T& At(size_t ii, size_t jj, size_t kk);

  // Test out of bounds.
  inline bool IsValid(size_t ii, size_t jj, size_t kk) const;

private:
  const size_t length_, width_, height_;
  std::vector<T> data_;
};

// ---------------------------- IMPLEMENTATION ------------------------------ //

template<typename T>
Array3D<T>::Array3D(size_t length, size_t width, size_t height)
  : length_(length), width_(width), height_(height),
    data_(length * width * height) {}

template<typename T>
T& Array3D<T>::At(size_t ii, size_t jj, size_t kk) {
  if (ii >= length_) {
    std::cerr << "X-index too large. Setting to max." << std::endl;
    ii = length_ - 1;
  }
  if (jj >= width_) {
    std::cerr << "Y-index too large. Setting to max." << std::endl;
    jj = width_ - 1;
  }
  if (kk >= height_) {
    std::cerr << "Z-index too large. Setting to max." << std::endl;
    kk = height_ - 1;
  }

  return data_.at(ii * width_ * height_ + jj * height_ + kk);
}

template<typename T>
bool Array3D<T>::IsValid(size_t ii, size_t jj, size_t kk) const {
  if (ii < length_ && jj < width_ && kk < height_)
    return true;
  return false;
}

#endif
