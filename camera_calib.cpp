#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "camera.h"

namespace {

using robot_vision::Camera;
using robot_vision::ConstructCamera;
using robot_vision::SerializeCam;

Camera CreateCamFromImages(cv::Vec3d cam_pos, double p, double y, double r, int chessboard_x, int chessboard_y, double chessboard_l) {
  cv::Mat frame;

  cv::VideoCapture cap(0);

  std::vector<std::vector<cv::Point2i>> img_points;

  if (!cap.isOpened()) {
    std::cerr << "ERROR! Unable to open camera\n";
  }

  std::vector<cv::Vec3d> obj_points;
  for (int i=0; i<chessboard_x; ++i) {
    for (int j=0; j<chessboard_y; ++j) {
      obj_points.push_back(cv::Vec3d(i*chessboard_x, j*chessboard_y, 0.0));
    }
  }

  std::vector<std::vector<cv::Vec3d>> t_obj_points;

  while (true) {
    cap.read(frame);

    if (frame.empty()) {
      std::cerr << "ERROR! blank frame grabbed\n";
      break;
    }

    int key = cv::waitKey(100);

    imshow("Live", frame);

    if (key == (uchar)' ') {
      std::vector<cv::Point2i> find_out;
      cv::findChessboardCorners(frame, cv::Size(chessboard_x, chessboard_y), find_out);
      img_points.push_back(find_out);
      t_obj_points.push_back(obj_points);
      continue;
    }

    if (key >= 0) {
      break;
    }
  }

  cv::Mat cam_mat, dist_coef, rvecs, tvecs;

  cv::calibrateCamera(t_obj_points, img_points, cv::Size(frame.rows, frame.cols), cam_mat, dist_coef, rvecs, tvecs);

  Camera out = ConstructCamera(cam_pos, p, y, r, cam_mat, dist_coef);
}

}  // namespace

int main() {
  Camera cam = CreateCamFromImages(cv::Vec3d(0, 0, 0), 0, 0, 0, 5, 5, 36.1);
  std::ofstream out("a.out");
  SerializeCam(cam, out);
  return 0;
}