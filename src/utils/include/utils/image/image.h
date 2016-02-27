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
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 *          David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// This class defines a wrapper around OpenCV's Mat class.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_IMAGE_IMAGE_H
#define BSFM_IMAGE_IMAGE_H

#include <memory>
#include <string>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace bsfm {

using Eigen::MatrixXd;
using Eigen::MatrixXf;

class Image {
 public:
  typedef std::shared_ptr<Image> Ptr;
  typedef std::shared_ptr<const Image> ConstPtr;

  Image() : grayscale_(false) {}
  ~Image() {}

  // Copy ctor.
  Image(const Image& other);

  // Basic ctor.
  Image(size_t width, size_t height, size_t channels);

  // Ctor to load from file.
  Image(const std::string& filename, bool grayscale = false);

  // Construct from OpenCV mat.
  explicit Image(const cv::Mat& other);

  // Access the pixel at (u, v), at a specific channel.
  template <typename T>
  inline T& at(size_t u, size_t v);

  template <typename T>
  const inline T& at(size_t u, size_t v) const;

  // Copy to and from OpenCV mats.
  void ToCV(cv::Mat& out) const;
  void FromCV(const cv::Mat& in);

  // Convert to Eigen matrix.
  void ToEigen(MatrixXf& eigen_out);

  // Save and load.
  void Read(const std::string& filename, bool grayscale = false);
  void Write(const std::string& filename) const;

  // Basic information.
  size_t Width() const;
  size_t Height() const;
  size_t Cols() const { return Width(); }
  size_t Rows() const { return Height(); }
  size_t Channels() const;
  bool IsColor() const { return !grayscale_; }

  // Resize to a specific scale or to a specific width and height.
  void Resize(double scale);
  void Resize(size_t new_width, size_t new_height);

  // Transpose, rotate, and flip.
  void Transpose();
  void RotateClockwise();
  void RotateCounterClockwise();
  void FlipLR();
  void FlipUD();

  // Convert between grayscale and 3-channel color.
  void ConvertToGrayscale();
  void ConvertToRGB();

  // Open a window to display the image.
  void ImShow(const std::string& window_name = std::string(),
              unsigned int wait_time = 0);

 private:
  bool grayscale_;
  std::shared_ptr<cv::Mat> image_;
}; //\class Image

// Non-member conversion from OpenCV to Eigen matrices.
template <typename T>
inline void OpenCVToEigenMat(
    const cv::Mat& cv_mat,
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& eigen_mat);

// Non-member conversions from Eigen to OpenCV matrices.
template <typename T>
inline void EigenMatToOpenCV(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& eigen_mat,
    cv::Mat& cv_mat);

// ------------------- Implementation ------------------- //

template <typename T>
T& Image::at(size_t u, size_t v) {
  CHECK_NOTNULL(image_.get());
  return image_->template at<T>(u, v);
}

template <typename T>
const T& Image::at(size_t u, size_t v) const {
  CHECK_NOTNULL(image_.get());
  return image_->template at<T>(u, v);
}

// Non-member conversion from OpenCV to Eigen matrices.
template <typename T>
void OpenCVToEigenMat(
    const cv::Mat& cv_mat,
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& eigen_mat) {

  // Make sure the data is grayscale and floating point before converting to an
  // eigen matrix.
  if (cv_mat.channels() != 1) {
    cv::Mat grayscale_mat;
    cv::cvtColor(cv_mat, grayscale_mat, CV_RGB2GRAY);
    cv::cv2eigen(grayscale_mat, eigen_mat);
  } else {
    cv::cv2eigen(cv_mat, eigen_mat);
  }
}

// Non-member conversion from Eigen to OpenCV matrices.
template <typename T>
void EigenMatToOpenCV(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& eigen_mat,
    cv::Mat& cv_mat) {
  cv::eigen2cv(eigen_mat, cv_mat);
}

// Deep copy ctor.
Image::Image(const Image& other) : grayscale_(false) {
  image_ = other.image_;
  grayscale_ = other.grayscale_;
}

// Basic ctor.
Image::Image(size_t width, size_t height, size_t channels) {
  if (channels == 1) {
    grayscale_ = true;
    image_ = std::shared_ptr<cv::Mat>(new cv::Mat(width, height, CV_32F, 0.f));
  } else {
    grayscale_ = false;
    image_ = std::shared_ptr<cv::Mat>(
        new cv::Mat(width, height, CV_32FC3, CV_RGB(0.f, 0.f, 0.f)));
  }
}

// Load from file ctor.
Image::Image(const std::string& filename, bool grayscale) : grayscale_(false) {
  image_ = std::shared_ptr<cv::Mat>(new cv::Mat());
  Read(filename, grayscale);
}

// Construct from OpenCV mat.
Image::Image(const cv::Mat& other) {
  image_ = std::shared_ptr<cv::Mat>(new cv::Mat());
  FromCV(other);
  grayscale_ = (this->Channels() == 1);
}

void Image::ToCV(cv::Mat& out) const {
  CHECK(image_.get()) << "Image data is not allocated.";

  if (grayscale_) {
    out = *image_;
  } else {
    cv::cvtColor(*image_, out, CV_RGB2BGR);
  }
}

void Image::FromCV(const cv::Mat& in) {
  CHECK(image_.get()) << "Image data is not allocated.";

  if (in.channels() == 1) {
    cv::cvtColor(in, *image_, CV_GRAY2RGB);
  } else {
    // OpenCV uses BGR by default for color images..
    cv::cvtColor(in, *image_, CV_BGR2RGB);
  }
}

void Image::ToEigen(MatrixXf& eigen_out) {
  CHECK(image_.get()) << "Image data is not allocated.";
  OpenCVToEigenMat(*image_, eigen_out);
}

void Image::Read(const std::string& filename, bool grayscale) {
  if (grayscale) {
    *image_ = cv::imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    cv::cvtColor(*image_, *image_, CV_GRAY2RGB);
  } else {
    *image_ = cv::imread(filename.c_str(), CV_LOAD_IMAGE_COLOR);
    cv::cvtColor(*image_, *image_, CV_BGR2RGB);
  }
  CHECK(image_->data) << "Unable to read image file.";
}

void Image::Write(const std::string& filename) const {
  cv::imwrite(filename.c_str(), *image_);
}

size_t Image::Width() const {
  CHECK(image_.get()) << "Image data is not allocated.";
  return image_->cols;
}

size_t Image::Height() const {
  CHECK(image_.get()) << "Image data is not allocated.";
  return image_->rows;
}

size_t Image::Channels() const {
  CHECK(image_.get()) << "Image data is not allocated.";
  return image_->channels();
}

void Image::Resize(double scale) {
  CHECK(image_.get()) << "Image data is not allocated.";
  cv::resize(*image_, *image_, cv::Size(), scale, scale, CV_INTER_LANCZOS4);
}

void Image::Resize(size_t new_width, size_t new_height) {
  CHECK(image_.get()) << "Image data is not allocated.";
  cv::resize(*image_, *image_, cv::Size(new_width, new_height),
             CV_INTER_LANCZOS4);
}

void Image::Transpose() {
  CHECK(image_.get()) << "Image data is not allocated.";
  cv::transpose(*image_, *image_);
}

void Image::RotateClockwise() {
  CHECK(image_.get()) << "Image data is not allocated.";
  Transpose();
  FlipLR();
}

void Image::RotateCounterClockwise() {
  CHECK(image_.get()) << "Image data is not allocated.";
  Transpose();
  FlipUD();
}

void Image::FlipLR() {
  CHECK(image_.get()) << "Image data is not allocated.";
  cv::flip(*image_, *image_, 1 /*about vertical axis*/);
}

void Image::FlipUD() {
  CHECK(image_.get()) << "Image data is not allocated.";
  cv::flip(*image_, *image_, 0 /*about horizontal axis*/);
}

void Image::ConvertToGrayscale() {
  CHECK(image_.get()) << "Image data is not allocated.";

  if (grayscale_) {
    VLOG(1) << "Cannot convert image to grayscale, image is already grayscale.";
    return;
  }

  cv::cvtColor(*image_, *image_, CV_RGB2GRAY);
  grayscale_ = true;
}

void Image::ConvertToRGB() {
  CHECK(image_.get()) << "Image data is not allocated.";

  if (!grayscale_) {
    VLOG(1) << "Cannot convert image to RGB, image is already RGB.";
    return;
  }

  cv::cvtColor(*image_, *image_, CV_GRAY2RGB);
  grayscale_ = false;
}

void Image::ImShow(const std::string& window_name, unsigned int wait_time) {
  CHECK(image_.get()) << "Image data is not allocated.";

  cv::namedWindow(window_name.c_str(), CV_WINDOW_AUTOSIZE);

  // If the image is not grayscale, convert it from RGB to BGR.
  if (!grayscale_) {
    cv::Mat bgr_image;
    ToCV(bgr_image);
    cv::imshow(window_name.c_str(), bgr_image);
  } else {
    cv::imshow(window_name.c_str(), *image_);
  }

  cv::waitKey(wait_time);
}

} //\namespace bsfm
#endif
