#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <fstream>
#include <chrono>
#include <thread>

#include "camera.h"
#include "apriltag.h"
#include "tag36h11.h"
#include "camera_handling.h"
#include "absl/log/log.h"
#include "absl/log/check.h"

namespace {

using robot_vision::Camera;
using robot_vision::Tags;
using robot_vision::RobotInWorldCoords;
using robot_vision::GetImage;

constexpr char famname[] = "tag36h11";

void GetRobotPositionTest(const Camera& cam, const Tags& tags) {
  cv::TickMeter meter;
  meter.start();

  cv::VideoCapture cap(2);
  CHECK(cap.isOpened()) << "Couldn't open video capture device.";

  apriltag_family_t *tf = tag36h11_create();
  
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  PCHECK(errno == 0) << "Unable to add family to detector.";

  td->quad_decimate = 2.0;
  td->quad_sigma = 0;
  td->nthreads = 1;
  td->debug = false;
  td->refine_edges = true;

  meter.stop();
  LOG(INFO) << "Detector " << famname << " initialized in " << std::fixed << std::setprecision(3) << meter.getTimeSec() << " seconds.";
  meter.reset();

  // std::cout << "Enter any key to begin calculation" << std::endl;
  // cv::waitKey(0);

  int cnt = 0;
  cv::Mat frame, gray;
  // Main Loop
  while (true) {
    cap.read(frame);
    cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    std::vector<std::pair<int, std::vector<cv::Point2d>>> out = GetImage(cam, gray, td);
    for (int i=0; i<out.size(); ++i) {
      cv::line(frame, out[i].second[0], out[i].second[1], cv::Scalar(255, 0, 0), 2);
      cv::line(frame, out[i].second[1], out[i].second[2], cv::Scalar(255, 0, 0), 2);
      cv::line(frame, out[i].second[2], out[i].second[3], cv::Scalar(255, 0, 0), 2);
      cv::line(frame, out[i].second[3], out[i].second[0], cv::Scalar(255, 0, 0), 2);
      cv::line(frame, cv::Point2i(0, 0), cv::Point2i(frame.cols-1, frame.rows-1), cv::Scalar(0, 255, 0), 2);
      cv::line(frame, cv::Point2i(frame.cols-1, 0), cv::Point2i(0, frame.rows-1), cv::Scalar(0, 255, 0), 2);
    }
    cv::imshow("Apriltag Detection", frame);
    // LOG(INFO) << "Detection Set " << cnt << " out.size=" << out.size();
    cnt++;
    for (int i=0; i<out.size(); ++i) {
      std::optional<std::pair<cv::Mat, cv::Mat>> pos = RobotInWorldCoords(cam, tags, out[i].first, out[i].second, /*82.55*/64.29);
      if (!pos.has_value()) {
        LOG(INFO) << "No Tags Found In Detection " << i << ".";
        continue;
      }
      LOG(INFO) << "Tag " << out[i].first << " | Solution 1:\n " << pos->first;
      LOG(INFO) << "Tag " << out[i].first << " | Solution 2:\n" << pos->second;
    }
    cv::waitKey(100);
  }

  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
}

}  // namespace


int main() {
  std::ifstream in("a.out");
  robot_vision::Camera cam = robot_vision::DeserializeCam(in);
  robot_vision::Tags tags;
  GetRobotPositionTest(cam, tags);
}