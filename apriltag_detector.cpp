#include "apriltag_detector.h"

#include "apriltag.h"
#include "tag36h11.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

namespace robot_vision {

ApriltagDetector::ApriltagDetector() {
  errno = 0;

  tf_ = tag36h11_create();
  CHECK(tf_ != 0) << "Cannot allocate tag family.";

  td_ = apriltag_detector_create();
  CHECK(td_ != 0) << "Cannot allocate tag detector.";

  apriltag_detector_add_family(td_, tf_);

  PCHECK(errno == 0) << "Unable to add family to detector.";

  td_->quad_decimate = 2.0;
  td_->quad_sigma = 0;
  td_->nthreads = 1;
  td_->debug = false;
  td_->refine_edges = true;
}

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

std::vector<TagPoints> ApriltagDetector::Detect(const cv::Mat& frame) {
  errno = 0;
  std::vector<TagPoints> images;

  image_u8_t im = {frame.cols, frame.rows, frame.cols, frame.data};

  zarray_t *detections = apriltag_detector_detect(td_, &im);

  if (errno == EAGAIN) {
    LOG(FATAL) << "Unable to create the " << td_->nthreads << " threads requested.";
  }
        
  for (int i=0; i<detections->size; ++i) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);
  
    images.push_back({
      .id = det->id, 
      .points = {
        cv::Point2d(det->p[0][0], det->p[0][1]),
        cv::Point2d(det->p[1][0], det->p[1][1]),
        cv::Point2d(det->p[2][0], det->p[2][1]),
        cv::Point2d(det->p[3][0], det->p[3][1])
      }
    });
  }

  apriltag_detections_destroy(detections);

  return images;
}

}  // namespace robot_vision