#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/log.h"
#include "camera.h"
#include "transformations.h"

ABSL_FLAG(int, camera, 0, "Camera ID.");
ABSL_FLAG(std::string, filename, "camera_calib.out", "File where outputs of calibtation are stored.");

namespace {

using robot_vision::Camera;
// using robot_vision::SerializeCam;

Camera CreateCamFromImages(cv::Vec3d cam_pos, double p, double y, double r, int chessboard_x, int chessboard_y, double chessboard_l) {
  cv::Mat frame;

  cv::VideoCapture cap(absl::GetFlag(FLAGS_camera));

  std::vector<std::vector<cv::Point2f>> img_points;

  if (!cap.isOpened()) {
    std::cerr << "ERROR! Unable to open camera\n";
  }

  std::vector<cv::Point3f> obj_points;
  for (int i=0; i<chessboard_x; ++i) {
    for (int j=0; j<chessboard_y; ++j) {
      obj_points.push_back(cv::Point3f(i*chessboard_l, j*chessboard_l, 0.0));
    }
  }

  std::vector<std::vector<cv::Point3f>> t_obj_points;
  while (true) {
    cap.read(frame);

    if (frame.empty()) {
      std::cerr << "ERROR! blank frame grabbed\n";
      break;
    }

    int key = cv::waitKey(30);
    if (key >= 0) {
      std::cerr << "key: " << (int) key << std::endl; 
    }
    
    std::vector<cv::Point2f> find_out;
    cv::findChessboardCorners(frame, cv::Size(chessboard_x, chessboard_y), find_out);
    if (find_out.size() > 0) {
      cv::Point2f prev = find_out[0];
      for (cv::Point2f e : find_out) {
        cv::line(frame, prev, e, cv::Scalar(255, 0, 0), 2);
        prev = e;
      }
    }
    cv::imshow("Calib", frame);

    if (key == 32) {
      if (find_out.size() == 0) {
        continue;
      }
      img_points.push_back(find_out);
      t_obj_points.push_back(obj_points);
      continue;
    }

    if (key >= 0) {
      break;
    }
  }

  

  // for (int i=0; i<img_points.size(); ++i) {
  //   std::cerr << img_points[i].size() << std::endl;
  // }
  
  cv::Mat cam_mat, dist_coef, rvecs, tvecs;

  cv::calibrateCamera(t_obj_points, img_points, cv::Size(frame.rows, frame.cols), cam_mat, dist_coef, rvecs, tvecs, cv::CALIB_RATIONAL_MODEL);

  return Camera(cam_mat, dist_coef);
}

}  // namespace

int main(int argc, char* argv[]) {
  absl::ParseCommandLine(argc, argv);
  Camera cam = CreateCamFromImages(cv::Vec3d(0, 0, 0), 0, 0, 0, 7, 7, 22.7);
  std::ofstream out(absl::GetFlag(FLAGS_filename));
  //SerializeCam(cam, out);
  return 0;
}