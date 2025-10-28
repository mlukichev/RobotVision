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

// Camera DeserializeCam(std::ifstream& in) {
//   Camera cam;
//   cam.cam_mat = DeserializeMat(in);
//   cam.dist_coef = DeserializeMat(in);
//   return cam;
// }

cv::Mat Camera::GetCamMat() const {
  return cam_mat_;
}

cv::Mat Camera::GetDistCoef() const {
  return dist_coef_;
}

}  // namespace robot_vision
