#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "transformations.h"

namespace robot_vision {

void SerializeMat(std::ofstream& out, const cv::Mat& mat);

cv::Mat DeserializeMat(std::istream& in);

struct Camera {
  cv::Mat cam_mat;
  cv::Mat dist_coef;
  Transformation pos;
};

void SerializeCam(const Camera& cam, std::ofstream& out);

Camera DeserializeCam(std::ifstream& in);

Camera ConstructCamera(cv::Vec3d cam_pos, double p, double y, double r, cv::Mat cam_mat, cv::Mat dist_coef);

} // namespace robot_vision

#endif // CAMERA_H