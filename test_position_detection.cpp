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


std::optional<cv::Mat> GetBestFit(const std::vector<cv::Mat>& mats, double tolerance) {
  if(mats.size() < 2) {
    LOG(INFO) << "Not enough solutions provided.";
    return std::nullopt;
  }

  const double tolerance2 = tolerance * tolerance;
  std::vector<cv::Mat> sol1;
  std::vector<cv::Mat> sol2;
  sol1.push_back(mats[0]);
  sol2.push_back(mats[1]);
  for (int i=2; i<mats.size(); ++i) {
    cv::Mat p = mats[i];
    if (GetSquareDist(MatToVec3d(sol1[0]), MatToVec3d(p)) < tolerance2) {
      sol1.push_back(std::move(p));
    } else
    if (GetSquareDist(MatToVec3d(sol2[0]), MatToVec3d(p))) {
      sol2.push_back(std::move(p));
    }
  }
  cv::Mat a = (cv::Mat_<double>(4, 4) << 
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0
  );
  if (sol1.size() >= sol2.size()) {
    cv::Mat mid = std::reduce(sol1.begin(), sol1.end(), a)/((double)sol1.size());
    cv::Mat rots = CombineRotation(MatToRot(mid));
    cv::Mat pos = MatToVec(mid);
    return CombRotVec(pos, rots);
  } else {
    cv::Mat mid = std::reduce(sol2.begin(), sol2.end(), a)/((double)sol2.size());
    cv::Mat rots = CombineRotation(MatToRot(mid));
    cv::Mat pos = MatToVec(mid);
    return CombRotVec(pos, rots);
  }
}

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

  // std::cout << "Enter any key to begin calculation" << std::endl;
  // cv::waitKey(0);

  int cnt = 0;
  cv::Mat frame, gray;
  // Main Loop
  while (true) {
    cap.read(frame);

    meter.start();
    cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    std::vector<std::pair<int, std::vector<cv::Point2d>>> out = GetImage(cam, gray, td);
    
    cv::line(frame, cv::Point2i(0, 0), cv::Point2i(frame.cols-1, frame.rows-1), cv::Scalar(0, 255, 0), 2);
    cv::line(frame, cv::Point2i(frame.cols-1, 0), cv::Point2i(0, frame.rows-1), cv::Scalar(0, 255, 0), 2);

    // LOG(INFO) << "Detection Set " << cnt << " out.size=" << out.size();
    std::vector<cv::Mat> sols;
    for (int i=0; i<out.size(); ++i) {
      // std::optional<std::pair<cv::Mat, cv::Mat>> pos = RobotInWorldCoords(cam, tags, out[i].first, out[i].second, /*82.55*/64.29);
      auto [tag_id, image_points] = out[i];
      //std::optional<std::pair<cv::Mat, cv::Mat>> pos = GetCameraInWorldCoords(cam, tags, tag_id, image_points, 64.29);
      //std::optional<std::pair<cv::Mat, cv::Mat>> pos = TransformTagToCam(cam, /*tags, tag_id,*/ image_points, 64.29);
      std::optional<std::pair<cv::Mat, cv::Mat>> pos1 = GetCameraInWorldCoords(cam, tags, tag_id, image_points, 64.29);
      if (!pos1.has_value()) {
        LOG(INFO) << "No Tags Found In Detection " << i << ".";
        continue;
      }
      LOG(INFO) << "Tag " << out[i].first << " | Solution 1:\n " << std::fixed << std::setprecision(2) << pos1->first;
      LOG(INFO) << "Tag " << out[i].first << " | Solution 2:\n" << std::fixed << std::setprecision(2) << pos1->second;
      sols.push_back(pos1->first);
      sols.push_back(pos1->second);
      // LOG(INFO) << "Tag " << out[i].first << " | Check 1:\n " << CheckOrtho(MatToRot(pos->first));
      // LOG(INFO) << "Tag " << out[i].first << " | Check 2:\n " << CheckOrtho(MatToRot(pos->second));

      cv::line(frame, out[i].second[0], out[i].second[1], cv::Scalar(255, 0, 0), 2);
      cv::line(frame, out[i].second[1], out[i].second[2], cv::Scalar(255, 0, 0), 2);
      cv::line(frame, out[i].second[2], out[i].second[3], cv::Scalar(255, 0, 0), 2);
      cv::line(frame, out[i].second[3], out[i].second[0], cv::Scalar(255, 0, 0), 2);
      
      //cv::Mat rvec;
      //cv::Rodrigues(MatToRot(pos->first), rvec);
      //cv::drawFrameAxes(frame, cam.cam_mat, cam.dist_coef, rvec, MatToVec3d(pos->first), 30, 2);
    }
    std::optional<cv::Mat> best = GetBestFit(sols, 1000);
    if (best.has_value()) {
      LOG(INFO) << "Best Solution\n" << *best;
      LOG(INFO) << "Check Ortho\n" << CheckOrtho(MatToRot(*best));
    }
    meter.stop();
    LOG(INFO) << "Frame computations: " << meter.getTimeSec() << " sec";
    meter.reset();

    

    cv::imshow("Position Detection", frame);

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