#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <fstream>

#include "camera.h"
#include "apriltag.h"
#include "tag36h11.h"
#include "camera_handling.h"
#include "absl/log/log.h"
#include "absl/log/check.h"

namespace {

using robot_vision::Camera;
using robot_vision::Tags;
using robot_vision::GetRobotPosition;
using robot_vision::GetImage;

constexpr char famname[] = "tag36h11";

void GetRobotPositionTest(const Camera& cam, const Tags& tags) {
  cv::TickMeter meter;
  meter.start();

  cv::VideoCapture cap(0);
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

  std::cout << "Enter any key to begin calculation" << std::endl;
  cv::waitKey(0);

  int cnt = 0;
  // Main Loop
  while (true) {
    std::vector<std::pair<int, std::vector<cv::Point2i>>> out = GetImage(cam, cap, td);
    LOG(INFO) << "Detection Set" << cnt << ".";
    for (int i=0; i<out.size(); ++i) {
      std::optional<std::pair<cv::Mat, cv::Mat>> pos = GetRobotPosition(cam, tags, out[i].first, out[i].second, 82.55);
      if (!pos.has_value()) {
        LOG(INFO) << "No Tags Found In Detection " << i << ".";
        continue;
      }
      LOG(INFO) << "Solution 1: " << pos->first;
      LOG(INFO) << "Solution 2: " << pos->second;
      // for (int j=0; j<pos->first.rows; ++j) {
      //   std::string a;
      //   for (int k=0; k<pos->first.cols; ++k) {
      //     a += std::to_string(pos->first.at<double>(j, k)) + " ";
      //   }
      //   LOG(INFO) << a;
      // }
    }
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