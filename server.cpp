#include <iostream>
#include <optional>
#include <memory>

#include "absl/cleanup/cleanup.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/log.h"
#include "data_handling.pb.h"
#include "data_handling.grpc.pb.h"
#include "grpcpp/server_builder.h"
#include "vision_system.h"
#include "transformations.h"
#include <thread>

ABSL_FLAG(std::string, server_address, "0.0.0.0:50001", "Vision system server address");

namespace robot_vision {
namespace {

using grpc::ServerContext;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReaderWriter;
using grpc::Status;
using grpc::StatusCode;
using grpc::WriteOptions;

class VisionSystemImpl: public VisionSystem::Service {
 public:
  VisionSystemImpl(VisionSystemCore* vision_system_core): 
    vision_system_core_{vision_system_core} {}

  Status OpenControlStream(ServerContext* context, ServerReaderWriter<ServerRequest, ClientRequest>* stream) override; 

  Status GetRobotPosition(ServerContext* context, const GetRobotPositionRequest* request, GetRobotPositionResponse* response) override;

  void SetCameraCoefficients(int camera_id);
 private:
  absl::Status SendMessage(const std::string& client, const ServerRequest& msg);

  VisionSystemCore* vision_system_core_;

  absl::Mutex mu_;
  std::unordered_map<std::string, ServerReaderWriter<ServerRequest, ClientRequest>*> clients_;
};

Status VisionSystemImpl::OpenControlStream(
  ServerContext* context,
  ServerReaderWriter<ServerRequest, ClientRequest>* stream) {
  LOG(INFO) << "Client connected: " << context->peer();

  {
    absl::MutexLock lock{&mu_};
    auto [_, inserted] = clients_.try_emplace(context->peer(), stream);
    if (!inserted) {
      LOG(ERROR) << "Client " << context->peer() << " already connected.";
      return Status(StatusCode::INVALID_ARGUMENT, "Client already connected.");
    }
  }

  absl::Cleanup cleanup_client = [&]() { 
    absl::MutexLock lock{&mu_};
    clients_.erase(context->peer()); 
  };
  
  ClientRequest req;
  while (stream->Read(&req)) {
    // LOG(INFO) << "Message received: " << req.DebugString();
    switch (req.msg_case()) {
    case ClientRequest::kReportCameraPositions: {
      vision_system_core_->ClearCameraPosition();
      for (const auto& camera_position : req.report_camera_positions().camera_position()) {
        int cam_id = camera_position.camera_id();
        absl::Status positions_calculated = vision_system_core_->ReportCameraPosition(camera_position);
        if (!positions_calculated.ok()) {
          LOG(ERROR) << "Could not compute position for camera " << cam_id;
        }
      }
      std::optional<Transformation> pos = vision_system_core_->GetRobotPosition();
      if (pos.has_value()) {
        LOG_EVERY_N_SEC(INFO, 1) << "Position: \n" << (std::string)*pos;
      } else {
        LOG_EVERY_N_SEC(INFO, 1) << "Cannot compute position";
      }
    } break;
    default:
      LOG(WARNING) << "Dropping unrecognized message: " << req.DebugString();
      break;
    }
  }
  return Status();
}
 
Status VisionSystemImpl::GetRobotPosition(ServerContext* context, const GetRobotPositionRequest* request, GetRobotPositionResponse* response) {
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  std::optional<Transformation> robot_position = vision_system_core_->GetRobotPosition();
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
  LOG(INFO) << "Finished calculations in " << t.count() << " milliseconds";
  if (!robot_position.has_value()) {
    LOG(ERROR) << "Empty position returned from Vision System Core";
    return Status(StatusCode::INTERNAL, "Empty position");
  }

  for (double e : robot_position->ToVector()) {
    response->add_mat(e);
  }

  return Status();
}

absl::Status VisionSystemImpl::SendMessage(const std::string& client, const ServerRequest& msg) {
  absl::MutexLock lock{&mu_};
  auto it = clients_.find(client);
  if (it == clients_.end()) {
    LOG(ERROR) << "SendMessage: Unknown client: " << client;
    return absl::InvalidArgumentError("Unknown client");
  }

  if (!it->second->Write(msg, WriteOptions())) {
    LOG(ERROR) << "SendMessage: Write failed to client: " << client;
    return absl::InternalError("Write failed");
  }

  return absl::OkStatus();
}

void VisionSystemImpl::SetCameraCoefficients(int camera_id) {
  const std::optional<std::pair<cv::Mat, cv::Mat>>& cams = vision_system_core_->GetCameraById(camera_id);
  if (!cams.has_value()) {
    LOG(ERROR) << "Camera " << camera_id << " does not have metadata";
    return;
  }

  std::vector<std::string> client_keys;
  {
    absl::MutexLock lock{&mu_};
    client_keys.reserve(clients_.size());
    for (const auto& [client, _] : clients_) {
      client_keys.push_back(client);
    } 
  }

  ServerRequest msg;
  auto* set_camera_coefficients = msg.mutable_set_camera_coefficients();
  set_camera_coefficients->set_camera_id(camera_id);
  CameraCoefficients* cam_coefficients = set_camera_coefficients->mutable_camera_coefficients();
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      cam_coefficients->add_camera_matrix(cams->first.at<double>(i, j));
    }
  }
  for (int i=0; i<cams->second.cols; ++i) {
    cam_coefficients->add_distortion_coefficients(cams->second.at<double>(0, i));
  }
  // TODO copy protobuf set_camera_coefficients->
  // TODO use lock for clients_
  for (const std::string& client : client_keys) {
    LOG(INFO) << "Sending camera coefficients to " << client;
    absl::Status status = SendMessage(client, msg);
    if (!status.ok()) {
      LOG(WARNING) << "Could not write camera coefficients to client: " << client;
    }
    // ignore errors
  }
}


void RunServer(VisionSystemCore* vision_system_core, const std::string& server_address) {
  VisionSystemImpl service(vision_system_core);

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  
  absl::Mutex threads_mutex;
  std::atomic<bool> stop{false};
  absl::CondVar cv;

  std::thread set_camera_coefficients([&]() {
    absl::MutexLock lock{&threads_mutex};
    while (!stop) {
      cv.WaitWithTimeout(&threads_mutex, absl::Milliseconds(2000));
      if (!stop) {
        for (int e : vision_system_core->GetKeys()) {
          service.SetCameraCoefficients(e);
        }
      }
    }
  });

  LOG(INFO) << "Server listening on " << server_address;
  server->Wait();

  {
    absl::MutexLock lock{&threads_mutex};
    stop = true;
    cv.SignalAll();
  }

  set_camera_coefficients.join();

}

}  // namespace
}  // namespace robot_vision

int main(int argc, char* argv[]) {
  absl::ParseCommandLine(argc, argv);

  robot_vision::VisionSystemCore vision_system_core(127.0, 4);

  robot_vision::RunServer(&vision_system_core, absl::GetFlag(FLAGS_server_address));
}