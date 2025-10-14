#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <fstream>
#include <chrono>
#include <thread>
#include <numeric>

#include "camera.h"
#include "transformations.h"
#include "apriltag.h"
#include "tag36h11.h"
#include "camera_handling.h"
#include "absl/log/log.h"
#include "absl/log/check.h"

namespace {

using robot_vision::Camera;
using robot_vision::Tags;
using robot_vision::GetCameraInWorldCoords;
using robot_vision::GetImage;
using robot_vision::CheckOrtho;
using robot_vision::MatToRot;
using robot_vision::MatToVec3d;
using robot_vision::TransformTagToCam;
using robot_vision::GetCameraInTagCoords;
using robot_vision::GetSquareDist;
using robot_vision::CombineRotation;
using robot_vision::CombRotVec;
using robot_vision::MatToVec;

constexpr char famname[] = "tag36h11";

u_int8_t EncodeCameras(int cam_num) {
  CHECK(cam_num <= 8) << "More than 8 cameras requested";
  u_int8_t out = 0;
  for (int i=1; i<cam_num; ++i) {
    cv::VideoCapture cap(i);
    if (cap.isOpened()) {
      out = out | ((u_int8_t)1);
    }
    out = out << 1;
  }
  cv::VideoCapture cap(cam_num);
  if (cap.isOpened()) {
    out = out | ((u_int8_t)1);
  }
  return out;
}

void GetRobotPositionFromCamera(const Camera& cam, const Tags& tags, cv::VideoCapture& cap, int threads, double apriltag_size) {
  cv::TickMeter meter;
  meter.start();

  apriltag_family_t *tf = tag36h11_create();
  
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  PCHECK(errno == 0) << "Unable to add family to detector.";

  td->quad_decimate = 2.0;
  td->quad_sigma = 0;
  td->nthreads = threads;
  td->debug = false;
  td->refine_edges = true;

  int cnt = 0;
  cv::Mat frame, gray;
  cap.read(frame);

  meter.start();
  cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  std::vector<std::pair<int, std::vector<cv::Point2d>>> out = GetImage(cam, gray, td);

  std::optional<std::pair<cv::Mat, cv::Mat>> pos = GetCameraInWorldCoords(cam, tags, tag_id, image_points, apriltag_size);

  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);

  return pos;
}

}  // namespace