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

Transformation::Transformation(const cv::Mat& rotation, const cv::Vec3d& translation) {
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
  out.reserve(16);
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

Transformation::operator const std::string() {
  std::string out = "[";
  for (int i=0; i<4; ++i) {
    std::string line = "";
    for (int j=0; j<3; ++j) {
      line += std::to_string(mat_.at<double>(i, j))+", ";
    }
    if (i != 3) {
      line += std::to_string(mat_.at<double>(i, 3))+",\n";
    } else {
      line += std::to_string(mat_.at<double>(i, 3))+"]";
    }
    out += line;
  }
  return out;
}

Transformation TransformationAverage(const std::vector<Transformation>& mats) {
  Transformation s((cv::Mat_<double>(4, 4) << 
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0
  ));
  for (Transformation e : mats) {
    s = s + e;
  }
  s = s/mats.size();
  cv::Mat U, W, Vt;
  cv::SVD::compute(s.ToRot(), W, U, Vt);

  double det_U = cv::determinant(U);
  double det_Vt = cv::determinant(Vt);

  if (abs(det_U*det_Vt - 1) <= 0.01) {
    Transformation out(U*Vt, s.ToVec3d());
    return out;
  } else {
    cv::Mat flip = (cv::Mat_<double>(3, 3) <<
      1, 0, 0,
      0, 1, 0,
      0, 0, -1
    );
    Transformation out(U*(Vt*flip), s.ToVec3d());
    return out;
  }
}

double TransformationDifference(const Transformation& a, const Transformation& b, double rotation_weight, double translation_weight) {
  Transformation s = a-b;
  cv::Mat rot = s.ToRot();
  cv::Vec3d pos = s.ToVec3d();
  double sum = 0;
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      double c = rot.at<double>(i, j)*rotation_weight;
      sum += c;
    }
  }
  for (int i=0; i<3; ++i) {
    double c = pos[i]*translation_weight;
    sum += c*c;
  }
  return sum;
}

}  // namespace robot_vision