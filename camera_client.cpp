#include <iostream>
#include <optional>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <filesystem>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/cleanup/cleanup.h"
#include "data_handling.pb.h"
#include "data_handling.grpc.pb.h"
#include "grpcpp/grpcpp.h"

#include "transformations.h"
#include "camera_handling.h"
#include "apriltag_detector.h"
#include "apriltag_detector.h"
#include "tag_detector.h"

namespace robot_vision {

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
  CameraSet() {
    captures_ = {};
    camera_metadata_ = {};
  }

  CameraSet() {}

  void BuildCameraSet();

  absl::StatusOr<std::unordered_map<int, std::pair<Transformation, Transformation>>> ComputePosition(int cam_id, double apriltag_size) const;

  std::vector<int> GetKeys() const;
 
 private:
  absl::Mutex mu_;
  std::unordered_map<int, std::unique_ptr<cv::VideoCapture>> captures_; // ID to VideoCapture
  std::unordered_map<int, Camera> camera_metadata_; // ID to Camera metadata
};

absl::StatusOr<std::unordered_map<int, std::pair<Transformation, Transformation>>> CameraSet::ComputePosition(int cam_id, double apriltag_size) const {
  absl::MutexLock lock{&mu_};

  auto cap = captures_.find(cam_id);

  if (cap == captures_.end()) {
    LOG(ERROR) << "Camera with id " << cam_id << " does not have an opened capture";
    return absl::InvalidArgumentError("Camera id does not have an opened capture");
  }

  auto meta = camera_metadata_.find(cam_id);

  if (meta == camera_metadata_.end()) {
    LOG(ERROR) << "Camera with id " << cam_id << " does not have metadata";
    return absl::InvalidArgumentError("Camera id does not have metadata");
  }

  cv::Mat frame;
  if (!cap->second->read(frame)) {
    LOG(ERROR) << "Could not read frame for camera id " << cam_id;
    return absl::InternalError("Could not read from camera");
  }
  ApriltagDetector detector;
  std::vector<TagPoints> img_points = detector.Detect(frame);
  absl::StatusOr<std::unordered_map<int, std::pair<Transformation, Transformation>>> out;
  for (const TagPoints& p : img_points) {
    std::optional<std::pair<Transformation, Transformation>> out_pos = TransformTagToCam(meta->second, p.points, apriltag_size);
    if (out_pos.has_value()) {
      out.emplace(p.id, std::move(out_pos));
    }
  }
  return out;
}

int GetIdFromName(const std::string& name) {
  int loc = name.find("camera-");
  if (loc == -1) {
    return -1;
  }
  return (int)(name[loc+7]-'0');
}

void CameraSet::BuildCameraSet() {
  absl::MutexLock lock{&mu_};

  captures_.clear();
  fs::path usb_directory("/dev/v4l/by-id");
  for (auto& p : fs::directory_iterator(usb_directory)) {
    fs::path current_camera = fs::read_symlink(p);
    std::string video_num = current_camera;
    if (video_num.find("video") != 6) {
      continue;
    }
    int port = (int)(video_num[11]-'0');
    std::unique_ptr<cv::VideoCapture> cap(new cv::VideoCapture(port));
    
    int camera_id = GetIdFromName(static_cast<fs::path>(p).filename());
    captures_[camera_id] = std::move(cap);
  }  
}

std::vector<int> CameraSet::GetKeys() const {
  absl::MutexLock lock{&mu_};
  std::vector<int> out;
  for (auto it=captures_.begin(); it != captures_.end(); ++it) {
    out.push_back(it->first);
  }
  return out;
}

namespace {

const std::string server_address = "";

namespace fs = std::filesystem;

using grpc::ClientReaderWriter;
using grpc::WriteOptions;

class VisionSystemClient {
 public:
  VisionSystemClient() {}

  absl::Status Run();
  absl::Status ReportCameraPositions(const CameraSet& camera_set, double apriltag_size);
 private:
  absl::Status SendMessage(const ClientRequest& msg);

  absl::Mutex mu_;
  std::optional<ClientReaderWriter<ClientRequest, ServerRequest>*> server_;
};

CameraSet BuildCameraSet() {
  CameraSet cam_set;
}

absl::Status VisionSystemClient::Run() {
  std::shared_ptr<grpc::Channel> channel = grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials());
  std::unique_ptr<VisionSystem::Stub> vision_system = VisionSystem::NewStub(channel);

  grpc::ClientContext context;

  std::unique_ptr<grpc::ClientReaderWriter<ClientRequest, ServerRequest>> stream  = vision_system->OpenControlStream(&context);

  {
    absl::MutexLock lock{&mu_};
    if (!server_.has_value()) {
      LOG(ERROR) << "Client is already connected.";
      return absl::InvalidArgumentError("Client already connected.");
    }
    server_ = stream.get();
  }

  absl::Cleanup cleanup_client = [&]() { 
    absl::MutexLock lock{&mu_};
    server_.reset(); 
  }; 

  ServerRequest req;
  while (stream->Read(&req)) {
    switch (req.msg_case()) {
    case ServerRequest::kSetCameraCoefficients:
      SetCameraCoefficients(req.set_camera_coefficients());
      break;
    default:
      LOG(WARNING) << "Dropping unrecognized message: " << req.DebugString();
      break;
    }
  }

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

absl::Status VisionSystemClient::ReportCameraPositions(const CameraSet& camera_set, double apriltag_size) {
  ClientRequest req;
  std::unordered_map<int, std::vector<Transformation>> out;
  robot_vision::ReportCameraPositions* report_camera_positions = req.mutable_report_camera_positions();
  for (int k : camera_set.GetKeys()) {

    absl::StatusOr<std::unordered_map<int, std::pair<Transformation, Transformation>>> pos = camera_set.ComputePosition(k, apriltag_size);

    if (!pos.ok()) {
      LOG(ERROR) << "Could not compute camera position: " << pos;
      continue;
    }

    CameraPosition* camera_position = report_camera_positions->add_camera_position();
    camera_position->set_camera_id(k);

    for (const auto& [tag_id, transformation_pair] : *pos) {
      CameraInTagCoords* camera_in_tag_coords1 = camera_position->add_camera_in_tag_coords();
      camera_in_tag_coords1->set_tag_id(tag_id);
      for (double v : transformation_pair.first.ToVector()) {
        camera_in_tag_coords1->add_mat(v);
      }
      CameraInTagCoords* camera_in_tag_coords2 = camera_position->add_camera_in_tag_coords();
      camera_in_tag_coords2->set_tag_id(tag_id);
      for (double v : transformation_pair.second.ToVector()) {
        camera_in_tag_coords2->add_mat(v);
      }
    }
  }
   
  return SendMessage(req);
}

}  // namespace
}  // namespace robot_vision

int main(int argc, char* argv[]) {
  robot_vision::CameraSet camera_set;
  robot_vision::VisionSystemClient client;
  while (true) {
    client.Run();
    client.ReportCameraPositions(camera_set, 64.29);
  }
}