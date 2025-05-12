#include <glog/logging.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>
#include <iostream>

#include "ros/init.h"
#include "y1_controller.h"

using namespace imeta::controller;

int main(int argc, char** argv) {
  ros::init(argc, argv, "imeta_y1_controller");

  // init gflags
  google::ParseCommandLineFlags(&argc, &argv, true);

  // init glog
  google::InitGoogleLogging(argv[0]);
  FLAGS_minloglevel = google::INFO;
  FLAGS_colorlogtostderr = true;
  FLAGS_alsologtostderr = true;

  std::string package_path = ros::package::getPath("imeta_y1_controller");
  std::string log_dir = package_path + "/log";
  if (!boost::filesystem::exists(log_dir)) {
    try {
      if (!boost::filesystem::create_directories(log_dir)) {
        std::cout << "Could not create directory " << log_dir << std::endl;
        return -1;
      }
    } catch (const boost::filesystem::filesystem_error& e) {
      std::cout << "Failed to create directories " << log_dir << ": "
                << e.what() << std::endl;
      return -1;
    }
  }
  FLAGS_log_dir = log_dir;
  std::cout << "imeta_y1_controller log dir: " << FLAGS_log_dir << std::endl;

  Y1Controller y1_controller;
  if (!y1_controller.Init()) {
    std::cout << "Failed to init Planning module!" << std::endl;

    return -1;
  }

  ros::spin();

  return 0;
}
