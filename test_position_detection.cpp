#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <fstream>
#include <chrono>
#include <thread>
#include <numeric>

#include "camera.h"
#include "transformations.h"
#include "apriltag_detector.h"

#include "apriltag.h"
#include "tag36h11.h"
#include "camera_handling.h"
#include "absl/log/log.h"
#include "absl/log/check.h"

namespace {

using robot_vision::Camera;
using robot_vision::Tags;
using robot_vision::Transformation;
using robot_vision::ApriltagDetector;
using robot_vision::TagPoints;
// using robot_vision::GetCameraInTagCoords;
// using robot_vision::GetSquareDist;
// using robot_vision::CombineRotation;
// using robot_vision::GetCameraInWorldCoords;

// std::optional<Transformation> GetBestFit(const std::vector<Transformation>& mats, double tolerance) {
//   if(mats.size() < 2) {
//     LOG(INFO) << "Not enough solutions provided.";
//     return std::nullopt;
//   }

//   const double tolerance2 = tolerance * tolerance;
//   std::vector<Transformation> sol1;
//   std::vector<Transformation> sol2;
//   sol1.push_back(mats[0]);
//   sol2.push_back(mats[1]);
//   for (int i=2; i<mats.size(); ++i) {
//     Transformation p = mats[i];
//     if (GetSquareDist(sol1[0].ToVec3d(), p.ToVec3d()) < tolerance2) {
//       sol1.push_back(std::move(p));
//     } else
//     if (GetSquareDist(sol2[0].ToVec3d(), p.ToVec3d())) {
//       sol2.push_back(std::move(p));
//     }
//   }
//   Transformation a = Transformation((cv::Mat_<double>(4, 4) << 
//     0, 0, 0, 0,
//     0, 0, 0, 0,
//     0, 0, 0, 0,
//     0, 0, 0, 0
//   ));
//   if (sol1.size() >= sol2.size()) {
//     Transformation mid = std::reduce(sol1.begin(), sol1.end(), a)/((double)sol1.size());
//     Transformation rots = CombineRotation(mid.ToRot());
//     Transformation pos = mid.ToVec();
//     Transformation frots = Transformation((cv::Mat_<double>(4, 4) <<
//       rots.Self().at<double>(0, 0), rots.Self().at<double>(0, 1), rots.Self().at<double>(0, 2), 0,
//       rots.Self().at<double>(1, 0), rots.Self().at<double>(1, 1), rots.Self().at<double>(1, 2), 0,
//       rots.Self().at<double>(2, 0), rots.Self().at<double>(2, 1), rots.Self().at<double>(2, 2), 0,
//       0, 0, 0, 1
//     ));
//     Transformation fpos = Transformation((cv::Mat_<double>(4, 4) <<
//       0, 0, 0, pos.Self().at<double>(0, 0),
//       0, 0, 0, pos.Self().at<double>(1, 0),
//       0, 0, 0, pos.Self().at<double>(2, 0),
//       0, 0, 0, 1
//     ));
//     return std::optional(frots+fpos);
//   } else {
//     Transformation mid = std::reduce(sol2.begin(), sol2.end(), a)/((double)sol2.size());
//     Transformation rots = CombineRotation(mid.ToRot());
//     Transformation pos = mid.ToVec();
//     Transformation frots = Transformation((cv::Mat_<double>(4, 4) <<
//       rots.Self().at<double>(0, 0), rots.Self().at<double>(0, 1), rots.Self().at<double>(0, 2), 0,
//       rots.Self().at<double>(1, 0), rots.Self().at<double>(1, 1), rots.Self().at<double>(1, 2), 0,
//       rots.Self().at<double>(2, 0), rots.Self().at<double>(2, 1), rots.Self().at<double>(2, 2), 0,
//       0, 0, 0, 1
//     ));
//     Transformation fpos = Transformation((cv::Mat_<double>(4, 4) <<
//       0, 0, 0, pos.Self().at<double>(0, 0),
//       0, 0, 0, pos.Self().at<double>(1, 0),
//       0, 0, 0, pos.Self().at<double>(2, 0),
//       0, 0, 0, 1
//     ));
//     return std::optional(frots+fpos);
//   }
// }

constexpr char famname[] = "tag36h11";

void GetRobotPositionTest(const Camera& cam, const Tags& tags) {
  cv::TickMeter meter;
  meter.start();

  cv::VideoCapture cap(3);
  CHECK(cap.isOpened()) << "Couldn't open video capture device.";

  ApriltagDetector detector;

  meter.stop();
  LOG(INFO) << "Detector " << famname << " initialized in " << std::fixed << std::setprecision(3) << meter.getTimeSec() << " seconds.";
  meter.reset();

  // std::cout << "Enter any key to begin calculation" << std::endl;
  // cv::waitKey(0);

  int cnt = 0;
  cv::Mat frame;
  // Main Loop
  while (true) {
    cap.read(frame);

    meter.start();
    std::vector<TagPoints> out = detector.Detect(frame);
    meter.stop();
    LOG(INFO) << "Apriltag Finding: " << meter.getTimeSec() << " sec";
    meter.reset();

    // cv::line(frame, cv::Point2i(0, 0), cv::Point2i(frame.cols-1, frame.rows-1), cv::Scalar(0, 255, 0), 2);
    // cv::line(frame, cv::Point2i(frame.cols-1, 0), cv::Point2i(0, frame.rows-1), cv::Scalar(0, 255, 0), 2);

    // std::vector<Transformation> sols;
    // for (int i=0; i<out.size(); ++i) {
    //   std::optional<std::pair<Transformation, Transformation>> pos1 = GetCameraInWorldCoords(cam, tags, out[i].id, out[i].points, 64.29);
    //   if (!pos1.has_value()) {
    //     LOG(INFO) << "No Tags Found In Detection " << i << ".";
    //     continue;
    //   }
    //   LOG(INFO) << "Tag " << out[i].id << " | Solution 1:\n " << std::fixed << std::setprecision(2) << pos1->first.Self();
    //   LOG(INFO) << "Tag " << out[i].id << " | Solution 2:\n" << std::fixed << std::setprecision(2) << pos1->second.Self();
    //   sols.push_back(pos1->first);
    //   sols.push_back(pos1->second);

    //   cv::line(frame, out[i].points[0], out[i].points[1], cv::Scalar(255, 0, 0), 2);
    //   cv::line(frame, out[i].points[1], out[i].points[2], cv::Scalar(255, 0, 0), 2);
    //   cv::line(frame, out[i].points[2], out[i].points[3], cv::Scalar(255, 0, 0), 2);
    //   cv::line(frame, out[i].points[3], out[i].points[0], cv::Scalar(255, 0, 0), 2);

    // }
    // std::optional<Transformation> best = GetBestFit(sols, 1000);
    // if (best.has_value()) {
    //   LOG(INFO) << "Best Solution\n" << (*best).Self();
    // }

    cv::imshow("Position Detection", frame);

    cv::waitKey(100);
  }
}

}  // namespace


int main() {
  std::ifstream in("a.out");
  robot_vision::Camera cam = robot_vision::DeserializeCam(in);
  robot_vision::Tags tags;
  GetRobotPositionTest(cam, tags);
}

