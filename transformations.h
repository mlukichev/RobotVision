#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace robot_vision {

class Transformation {
 public:
  Transformation(const cv::Mat& mat): mat_{mat} {}

  Transformation(cv::Mat&& mat): mat_{std::move(mat)} {}

  Transformation(const cv::Mat&, const cv::Mat&);

  Transformation(const cv::Vec3d&, const cv::Mat&);

  Transformation(): mat_{cv::Mat::eye(4, 4, CV_64F)} {}

  Transformation(const Transformation& o) = default;

  Transformation(Transformation&& o) = default;

  cv::Mat Self();

  std::vector<double> ToVector() const;

  cv::Vec3d ToVec3d();

  cv::Mat ToRot();

  cv::Mat ToVec();

  Transformation Inverse();

  Transformation operator*(const Transformation& other) const;

  Transformation operator/(const Transformation& other) const;

  Transformation operator*(double other) const;

  Transformation operator/(double other) const;

  Transformation operator+(const Transformation& other) const;

  Transformation operator-(const Transformation& other) const;

 private:
  cv::Mat mat_;
};

double GetSquareDist(const cv::Vec3d& a, const cv::Vec3d& b);

cv::Mat CombineRotation(Transformation mat);

}  // namespace robot_vision

#endif