#include "transformations.h"

#include <opencv2/opencv.hpp>
#include <vector>

namespace robot_vision {

Transformation::Transformation(const cv::Mat& translation, const cv::Mat& rotation) {
  mat_ = (cv::Mat_<double>(4, 4) << 
    rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2), translation.at<double>(0, 0),
    rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2), translation.at<double>(1, 0),
    rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2), translation.at<double>(2, 0),
    0, 0, 0, 1
  );
}

Transformation::Transformation(const cv::Vec3d& translation, const cv::Mat& rotation) {
  mat_ = (cv::Mat_<double>(4, 4) << 
    rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2), translation[0],
    rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2), translation[1],
    rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2), translation[2],
    0, 0, 0, 1
  );
}

cv::Mat Transformation::Self() {
  return mat_;
}

std::vector<double> Transformation::ToVector() const {
  std::vector<double> out;
  for (int i=0; i<4; ++i) {
    for (int j=0; j<4; ++j) {
      out.push_back(mat_.at<double>(i, j));
    }
  }
  return out;
}

Transformation Transformation::operator*(const Transformation& other) const {
  return Transformation(mat_*other.mat_);
}

Transformation Transformation::operator/(const Transformation& other) const {
  return Transformation(mat_/other.mat_);
}

Transformation Transformation::operator*(double other) const {
  return Transformation(mat_*other);
}

Transformation Transformation::operator/(double other) const {
  return Transformation(mat_/other);
}

Transformation Transformation::operator+(const Transformation& other) const {
  return Transformation(mat_+other.mat_);
}

Transformation Transformation::operator-(const Transformation& other) const {
  return Transformation(mat_-other.mat_);
}

cv::Mat Transformation::ToVec() {
  cv::Mat out = (cv::Mat_<double>(4, 1) << 
    mat_.at<double>(0, 3),
    mat_.at<double>(1, 3),
    mat_.at<double>(2, 3),
    1
  );
  return out;
}

cv::Vec3d Transformation::ToVec3d() {
  cv::Vec3d out = cv::Vec3d(
    mat_.at<double>(0, 3),
    mat_.at<double>(1, 3),
    mat_.at<double>(2, 3)
  );
  return out;
}

cv::Mat Transformation::ToRot() {
  cv::Mat out = (cv::Mat_<double>(3, 3) << 
    mat_.at<double>(0, 0), mat_.at<double>(0, 1), mat_.at<double>(0, 2),
    mat_.at<double>(1, 0), mat_.at<double>(1, 1), mat_.at<double>(1, 2),
    mat_.at<double>(2, 0), mat_.at<double>(2, 1), mat_.at<double>(2, 2)
  );
  return out;
}

Transformation Transformation::Inverse() {
  cv::Mat inv;
  cv::invert(mat_, inv);
  return Transformation(inv);
}

cv::Mat CombineRotation(Transformation mat) {
  cv::Mat U, W, Vt;
  cv::SVD::compute(mat.ToRot(), W, U, Vt);

  double det_U = cv::determinant(U);
  double det_Vt = cv::determinant(Vt);

  if (abs(det_U*det_Vt - 1) <= 0.01) {
    return U*Vt;
  } else {
    cv::Mat flip = (cv::Mat_<double>(3, 3) <<
      1, 0, 0,
      0, 1, 0,
      0, 0, -1
    );
    return U*(Vt*flip);
  }
}

double GetSquareDist(const cv::Vec3d& a, const cv::Vec3d& b) {
  return (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]);
}

}  // namespace robot_vision