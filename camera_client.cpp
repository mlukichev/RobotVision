#include <atomic>
#include <iostream>
#include <optional>
#include <vector>
#include <memory>
#include <unordered_map>
#include <filesystem>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/cleanup/cleanup.h"
#include "data_handling.pb.h"
#include "data_handling.grpc.pb.h"
#include "grpcpp/grpcpp.h"
#include "opencv2/opencv.hpp"

#include "transformations.h"
#include "camera_handling.h"
#include "apriltag_detector.h"
#include "apriltag_detector.h"
#include "tag_detector.h"
#include "cameras.h"

ABSL_FLAG(std::string, server_address, "10.0.0.1:50001", "Server address");

namespace robot_vision {

namespace {
namespace fs = std::filesystem;

// class OpenedCamera {
//  public:
//   OpenedCamera(const Camera& cam, cv::VideoCapture& cap): cam_{cam}, cap_{std::unique_ptr<cv::VideoCapture>(cap)} {}
//   std::optional<std::pair<Transformation, Transformation>> ComputePosition(double apriltag_size);

//  private:
//   Camera cam_;
//   std::unique_ptr<cv::VideoCapture> cap_;
// };

class CameraSet {
 public:
  void BuildCameraSet();
  void SetCameraCoefficients(int camera_id, const CameraCoefficients& camera_coefficients);
  void TestReading(int camera_id);

  absl::StatusOr<std::unordered_map<int, std::pair<Transformation, Transformation>>> ComputePosition(int cam_id, double apriltag_size);

  std::vector<int> GetKeys();
 
 private:
  absl::Mutex mu_;
  std::unordered_map<int, std::unique_ptr<cv::VideoCapture>> captures_; // ID to VideoCapture
  // std::unordered_map<int, Camera> camera_metadata_; // ID to Camera metadata
  Cameras cameras_;
};

void CameraSet::TestReading(int camera_id) {
  cv::Mat frame;
  captures_[camera_id]->read(frame);
}

absl::StatusOr<std::unordered_map<int, std::pair<Transformation, Transformation>>> CameraSet::ComputePosition(int cam_id, double apriltag_size) {
  absl::MutexLock lock{&mu_};

  auto cap = captures_.find(cam_id);

  if (cap == captures_.end()) {
    LOG_EVERY_N_SEC(ERROR, 1) << "Camera with id " << cam_id << " does not have an opened capture";
    return absl::InvalidArgumentError("Camera id does not have an opened capture");
  }

  if (!cameras_.CameraExists(cam_id)) {
    LOG_EVERY_N_SEC(ERROR, 1) << "Camera with id " << cam_id << " does not have metadata";
    return absl::InvalidArgumentError("Camera id does not have metadata");
  }

  cv::Mat frame;
  if (!cap->second->read(frame)) {
    LOG_EVERY_N_SEC(ERROR, 1) << "Could not read frame for camera id " << cam_id;
    return absl::InternalError("Could not read from camera");
  }

  ApriltagDetector detector;
  std::vector<TagPoints> img_points = detector.Detect(frame);

  // cv::imshow("camid", frame);
  // cv::waitKey(30);

  std::unordered_map<int, std::pair<Transformation, Transformation>> out;
  for (const TagPoints& p : img_points) {
    std::optional<std::pair<Transformation, Transformation>> out_pos = GetTagInCamCoords(cameras_.GetCameraByID(cam_id), p.points, apriltag_size);
    if (out_pos.has_value()) {
      out.emplace(p.id, std::move(*out_pos));
    }
  }
  return absl::StatusOr<std::unordered_map<int, std::pair<Transformation, Transformation>>>(std::move(out));
}

std::optional<int> GetIdFromName(const std::string& name) {
  int loc = name.find("camera-");
  if (loc == -1) {
    return std::nullopt;
  }
  return (int)(name[loc+7]-'0');
}

void CameraSet::BuildCameraSet() {
  absl::MutexLock lock{&mu_};
  fs::path usb_directory("/dev/v4l/by-id");
  std::vector<int> rem;
  rem.reserve(captures_.size());
  for (auto& [k, cap] : captures_) {
    cv::Mat frame;
    if (!cap->isOpened() || !cap->read(frame)) {
      rem.push_back(k);
    }
  }
  for (int e : rem) {
    captures_.find(e)->second.release();
    captures_.erase(e);
    LOG(INFO) << "Erased camera " << e <<" from CameraSet";
  }
  for (auto& p : fs::directory_iterator(usb_directory)) {
    std::optional<int> camera_id = GetIdFromName(static_cast<fs::path>(p).filename());
    if (!camera_id.has_value()) {
      continue;
    }
    auto caps = captures_.find(*camera_id);
    cv::Mat frame;
    if (caps != captures_.end() && caps->second->read(frame)) {
      LOG(INFO) << "Camera " << *camera_id << " is already initialized";
      continue;
    }
    fs::path current_camera = fs::read_symlink(p);
    std::string video_num = current_camera;
    if (video_num.find("video") != 6) {
      continue;
    }
    int port = (int)(video_num[11]-'0');
    std::unique_ptr<cv::VideoCapture> cap(new cv::VideoCapture(port));
    if (cap->isOpened() && cap->read(frame)) {
      LOG(INFO) << "Port: " << port << " | Camera Id: " << *camera_id << " | "<< static_cast<fs::path>(p).filename();
      captures_[*camera_id] = std::move(cap);
    } else {
      LOG(WARNING) << "Port " << port << " cannot be opened";
    }
  }
  LOG(INFO) << "Captures size: " << captures_.size();
}

void CameraSet::SetCameraCoefficients(int camera_id, const CameraCoefficients& camera_coefficients) {
  cv::Mat cam_mat = (cv::Mat_<double>(3, 3) << 
    camera_coefficients.camera_matrix(0), camera_coefficients.camera_matrix(1), camera_coefficients.camera_matrix(2),
    camera_coefficients.camera_matrix(3), camera_coefficients.camera_matrix(4), camera_coefficients.camera_matrix(5),
    camera_coefficients.camera_matrix(6), camera_coefficients.camera_matrix(7), camera_coefficients.camera_matrix(8)
  );
  int size = camera_coefficients.distortion_coefficients_size();
  cv::Mat dist_coef(cv::Size(1, size), CV_64F);
  for (int i=0; i<size; ++i) {
    dist_coef.at<double>(0, i) = camera_coefficients.distortion_coefficients(i);
  }
  absl::MutexLock lock{&mu_};
  //camera_metadata_.emplace(camera_id, Camera(camera_id, cam_mat, dist_coef));
}

std::vector<int> CameraSet::GetKeys() {
  absl::MutexLock lock{&mu_};
  std::vector<int> out;
  for (auto it=captures_.begin(); it != captures_.end(); ++it) {
    if (!cameras_.CameraExists(it->first)) {
      LOG_EVERY_N_SEC(WARNING, 30) << "Camera " << it->first << " doesn't have metadata.";
      continue;
    }
    out.push_back(it->first);
  }
  return out;
}

using grpc::ClientReaderWriter;
using grpc::WriteOptions;

class VisionSystemClient {
 public:
  VisionSystemClient() {}

  absl::Status Run(const std::string& server_address);
  absl::Status ReportCameraPositions(double apriltag_size);
 private:
  absl::Status SendMessage(const ClientRequest& msg);

  absl::Mutex mu_;
  std::optional<ClientReaderWriter<ClientRequest, ServerRequest>*> server_;
  CameraSet camera_set_;
};

absl::Status VisionSystemClient::Run(const std::string& server_address) {
  camera_set_.BuildCameraSet(); // first invocation before connecting to server

  LOG(INFO) << "Connecting to " << server_address;
  std::shared_ptr<grpc::Channel> channel = grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials());
  std::unique_ptr<VisionSystem::Stub> vision_system = VisionSystem::NewStub(channel);

  grpc::ClientContext context;

  std::unique_ptr<grpc::ClientReaderWriter<ClientRequest, ServerRequest>> stream  = vision_system->OpenControlStream(&context);

  {
    absl::MutexLock lock{&mu_};
    if (server_.has_value()) {
      LOG(ERROR) << "Client is already connected.";
      return absl::InvalidArgumentError("Client already connected.");
    }
    server_ = stream.get();
  }

  absl::Cleanup cleanup_client = [&]() { 
    absl::MutexLock lock{&mu_};
    server_.reset(); 
  };

  absl::Mutex threads_mutex;
  std::atomic<bool> stop{false};
  absl::CondVar cv;

  std::thread report_camera_positions([&]() {
    absl::MutexLock lock{&threads_mutex};
    while (!stop) {
      cv.WaitWithTimeout(&threads_mutex, absl::Milliseconds(1));
      if (!stop) {
        absl::Status status = ReportCameraPositions(64.29);
        if (!status.ok()) {
          LOG(ERROR) << "Failed to report camera positions: " << status;
        }
      }
    }
  });

  std::thread build_camera_set([&]() {
    absl::MutexLock lock{&threads_mutex};
    while (!stop) {
      cv.WaitWithTimeout(&threads_mutex, absl::Seconds(10));
      if (!stop) {
        LOG(INFO) << "Start building CameraSet";
        camera_set_.BuildCameraSet();
        LOG(INFO) << "Built CameraSet";
        // camera_set_.TestReading(2);
      }
    }
  }); 

  ServerRequest req;
  while (stream->Read(&req)) {
    switch (req.msg_case()) {
    case ServerRequest::kSetCameraCoefficients:
      camera_set_.SetCameraCoefficients(
        req.set_camera_coefficients().camera_id(),
        req.set_camera_coefficients().camera_coefficients());
      break;
    default:
      LOG(WARNING) << "Dropping unrecognized message: " << req.DebugString();
      break;
    }
  }

  {
    absl::MutexLock lock{&threads_mutex};
    stop = true;
    cv.SignalAll();
  }

  report_camera_positions.join();
  build_camera_set.join();
  
  return absl::OkStatus();

}

absl::Status VisionSystemClient::SendMessage(const ClientRequest& msg) {
  absl::MutexLock lock{&mu_};
  if (!server_.has_value()) {
    LOG(ERROR) << "SendMessage: Not connected to server";
    return absl::InvalidArgumentError("Not connected");
  }

  if (!(*server_)->Write(msg)) {
    LOG(ERROR) << "SendMessage: Write failed";
    return absl::InternalError("Write failed");
  }

  return absl::OkStatus();
}

absl::Status VisionSystemClient::ReportCameraPositions(double apriltag_size) {
  ClientRequest req;
  std::unordered_map<int, std::vector<Transformation>> out;
  robot_vision::ReportCameraPositions* report_camera_positions = req.mutable_report_camera_positions();
  for (int k : camera_set_.GetKeys()) {
    absl::StatusOr<std::unordered_map<int, std::pair<Transformation, Transformation>>> pos = camera_set_.ComputePosition(k, apriltag_size);

    if (!pos.ok()) {
      LOG_EVERY_N_SEC(ERROR, 1) << "Could not compute camera position: " << pos.status();
      continue;
    }

    CameraPosition* camera_position = report_camera_positions->add_camera_position();
    camera_position->set_camera_id(k);

    for (const auto& [tag_id, transformation_pair] : *pos) {
      CameraInTagCoords* camera_in_tag_coords = camera_position->add_camera_in_tag_coords();
      camera_in_tag_coords->set_tag_id(tag_id);
      for (double v : transformation_pair.first.ToVector()) {
        camera_in_tag_coords->add_mat1(v);
      }
      for (double v : transformation_pair.second.ToVector()) {
        camera_in_tag_coords->add_mat2(v);
      }
    }
  }
   
  return SendMessage(req);
}

}  // namespace
}  // namespace robot_vision

int main(int argc, char* argv[]) {
  absl::ParseCommandLine(argc, argv);
  robot_vision::CameraSet camera_set;
  robot_vision::VisionSystemClient client;
  while (true) {
    absl::Status status = client.Run(absl::GetFlag(FLAGS_server_address));
    if (!status.ok()) {
      LOG(ERROR) << "Server connection finished with status: " << status;
    }
  }
}