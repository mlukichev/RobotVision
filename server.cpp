#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <memory>

#include "grpcpp/grpcpp.h"
#include "data_handling_server.grpc.pb.h"

// Enter port address
std::string port_address = "";
std::shared_ptr<grpc::Channel> channel = grpc::CreateChannel(port_address, grpc::InsecureChannelCredentials());

// Put address of computers with cameras
std::vector<std::string> recive = {

};

// Put address of control computers data need to be sent too
std::vector<std::string> send = {

};

class RobotPositionClient {
  public:
    std::optional<cv::Mat> DeserializeMat(data_handling::RobotPositionResponse res) {
      if (res.data().empty()) {
        return std::nullopt;
      }
      int rows = res.rows();
      int cols = res.cols();
      cv::Mat out(rows, cols, (void*)res.data().c_str());
      return out.clone();
    }

    cv::Mat 

    cv::Mat GetRobotPos() {
      for (std::string e : ) {

      }
    }

  private:
    std::unique_ptr<data_handling::RobotPositionRequest::Stub> stub_;
};