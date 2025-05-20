#include "rclcpp/rclcpp.hpp"
#include "garbage_classification/srv/garbage_classification.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char ​**argv) {
  rclcpp::init(argc, argv);

  if (argc != 2) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: client [garbage_name]");
    return 1;
  }

  auto node = rclcpp::Node::make_shared("garbage_classification_client");
  auto client = node->create_client<garbage_classification::srv::GarbageClassification>("garbage_classification");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  auto request = std::make_shared<garbage_classification::srv::GarbageClassification::Request>();
  request->name = argv[1];

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "分类结果: %s", result.get()->category.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "操作指引: %s", result.get()->guide.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}