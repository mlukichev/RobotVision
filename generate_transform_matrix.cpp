#include <iostream>
#include <opencv2/opencv.hpp>
#include "absl/log/log.h"

int main() {
  double p, y, r;
  std::cin >> p >> y >> r;
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
  LOG(INFO) << "\n" << rot_mat;
}