#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "transformations.h"

namespace robot_vision {

// void SerializeMat(std::ofstream& out, const cv::Mat& mat);

// cv::Mat DeserializeMat(std::istream& in);

class Camera {
 public:
  Camera(int id, const cv::Mat& cam_mat, const cv::Mat& dist_coef): id_{id}, cam_mat_{cam_mat}, dist_coef_{dist_coef} {}
  cv::Mat GetCamMat() const;
  cv::Mat GetDistCoef() const;
  int GetId() const;
 private:
  int id_;
  cv::Mat cam_mat_;
  cv::Mat dist_coef_;
};

} // namespace robot_vision

#endif // CAMERA_H