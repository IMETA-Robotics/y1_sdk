#include <glog/logging.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "y1_controller.h"

using namespace imeta::y1_controller;

void signalHandler(int signum) { std::exit(signum); }

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::signal(SIGINT, signalHandler);

  auto y1_controller = std::make_shared<Y1Controller>();
  if (!y1_controller->Init()) {
    std::cout << "Failed to init Planning module!" << std::endl;
    return -1;
  }

  rclcpp::spin(y1_controller);
  rclcpp::shutdown();

  return 0;
}