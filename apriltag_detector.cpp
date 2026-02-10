#include "apriltag_detector.h"
#include "camera_handling.h"
#include "camera.h"
#include "cameras.h"

#include "apriltag.h"
#include "common/timeprofile.h" // from apriltag
#include "tag36h11.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

namespace robot_vision {

ApriltagDetector::ApriltagDetector(double quad_decimate, int quad_sigma, int nthreads, bool debug, bool refine_edges) {
  errno = 0;

  tf_ = tag36h11_create();
  CHECK(tf_ != 0) << "Cannot allocate tag family.";

  td_ = apriltag_detector_create();
  CHECK(td_ != 0) << "Cannot allocate tag detector.";

  apriltag_detector_add_family(td_, tf_);

  PCHECK(errno == 0) << "Unable to add family to detector.";

  td_->quad_decimate = quad_decimate;
  td_->quad_sigma = quad_sigma;
  td_->nthreads = nthreads;
  td_->debug = debug;
  td_->refine_edges = refine_edges;
}

ApriltagDetector::~ApriltagDetector() {
  apriltag_detector_destroy(td_);
  tag36h11_destroy(tf_);
}

std::vector<TagPoints> ApriltagDetector::Detect(cv::Mat& frame, const Cameras& cams, int cam_id) {
  errno = 0;
  std::vector<TagPoints> images;

  // cv::Mat gray;
  // cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

  // LOG(INFO) << "Rows, cols: " << gray.rows << " " << gray.cols;

  image_u8_t im = {frame.cols, frame.rows, frame.cols, frame.data};

  zarray_t *detections = apriltag_detector_detect(td_, &im);
  // TODO Debug-only: pring apriltag stats
  timeprofile_display(td_->tp);

  if (errno == EAGAIN) {
    LOG(FATAL) << "Unable to create the " << td_->nthreads << " threads requested.";
  }

  Camera cam = cams.GetCameraByID(cam_id);
        
  for (int i=0; i<detections->size; ++i) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    cv::Scalar color(255, 0, 0);
    int thickness = 2;

    // line(frame, cv::Point2d(det->p[0][0], det->p[0][1]), cv::Point2d(det->p[1][0], det->p[1][1]), color, thickness);
    // line(frame, cv::Point2d(det->p[1][0], det->p[1][1]), cv::Point2d(det->p[2][0], det->p[2][1]), color, thickness);
    // line(frame, cv::Point2d(det->p[2][0], det->p[2][1]), cv::Point2d(det->p[3][0], det->p[3][1]), color, thickness);
    // line(frame, cv::Point2d(det->p[3][0], det->p[3][1]), cv::Point2d(det->p[0][0], det->p[0][1]), color, thickness);
    // line(frame, cv::Point2d(frame.cols/2, 0), cv::Point2d(frame.cols/2, frame.rows), color, thickness);
    // line(frame, cv::Point2d(0, frame.rows/2), cv::Point2d(frame.cols, frame.rows/2), color, thickness);

    images.push_back({
      .id = det->id, 
      .points = {
        cv::Point2d(det->p[0][0], det->p[0][1]),
        cv::Point2d(det->p[1][0], det->p[1][1]),
        cv::Point2d(det->p[2][0], det->p[2][1]),
        cv::Point2d(det->p[3][0], det->p[3][1])
      }
    });

    // auto t = GetTagToCam(cam, images[i].points, 64.25);
    // if (!t.has_value()) {
    //   continue;
    // }
    // cv::Mat rot;
    // cv::Rodrigues((*t).first.ToRot(), rot);
    // cv::drawFrameAxes(frame, cam.GetCamMat(), cam.GetDistCoef(), rot, (*t).first.ToVec3d(), 15, 2);
  }

  // cv::imshow("", frame);
  // cv::waitKey(1);

  apriltag_detections_destroy(detections);

  return images;
}

}  // namespace robot_vision