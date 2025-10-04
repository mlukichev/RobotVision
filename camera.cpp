#include "camera.h"

#include <iostream>
#include <fstream>
#include <utility>
#include <algorithm>
#include <opencv2/opencv.hpp>

#include "transformations.h"

namespace robot_vision {

void SerializeMat(std::ofstream& out, const cv::Mat& mat) {
  out << mat.rows << " " << mat.cols << std::endl;
  for (int i=0; i<mat.rows; ++i) {
    for (int j=0; j<mat.cols; ++j) {
      out << mat.at<double>(i, j) << " ";
    }
    out << std::endl;
  }
}

cv::Mat DeserializeMat(std::ifstream& in) {
  int m, n;
  in >> m >> n;
  cv::Mat out(m, n, CV_64F);
  for (int i=0; i<m; ++i) {
    for (int j=0; j<n; ++j) {
      in >> out.at<double>(i, j); 
    }
  }
  return out;
}

void SerializeCam(const Camera& cam, std::ofstream& out) {
  SerializeMat(out, cam.cam_mat);
  SerializeMat(out, cam.dist_coef);
  SerializeMat(out, cam.pos);
}

Camera DeserializeCam(std::ifstream& in) {
  Camera cam;
  cam.cam_mat = DeserializeMat(in);
  cam.dist_coef = DeserializeMat(in);
  cam.pos = DeserializeMat(in);
}

Camera ConstructCamera(cv::Vec3d cam_pos, double p, double y, double r, cv::Mat cam_mat, cv::Mat dist_coef) {
  Camera out;
  cv::Mat pitch = (cv::Mat_<double>(3, 3) <<
    std::cos(p), -std::sin(p), 0,
    std::sin(p), std::cos(p), 0,
    0, 0, 1
  );
  cv::Mat yaw = (cv::Mat_<double>(3, 3) <<
    std::cos(y), 0, std::sin(y),
    0, 1, 0,
    -std::sin(y), 0, std::cos(y)
  );
  cv::Mat roll = (cv::Mat_<double>(3, 3) << 
    1, 0, 0,
    0, std::cos(r), -std::sin(r),
    0, std::sin(r), std::cos(r)
  );
  cv::Mat rot_mat = pitch * yaw * roll;
  out.pos = (cv::Mat_<double>(4, 4) << 
    rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2), cam_pos[0],
    rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2), cam_pos[1],
    rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2), cam_pos[2],
    0, 0, 0, 1
  );
  out.cam_mat = cam_mat;
  out.dist_coef = dist_coef;
  return out;
}

}  // namespace robot_vision
