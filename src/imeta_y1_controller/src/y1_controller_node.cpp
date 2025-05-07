#include "y1_controller.h"
// #include "gflags/planning_gflags.h"
#include <glog/logging.h>
// #include <boost/filesystem.hpp>

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

  // std::string log_dir = FLAGS_planning_log_dir;
  // if (!boost::filesystem::exists(log_dir)) {
  //   try {
  //     if (!boost::filesystem::create_directories(log_dir)) {
  //       AERROR << "Could not create directory " << log_dir;
  //       return -1;
  //     }
  //   } catch (const boost::filesystem::filesystem_error& e) {
  //     AERROR << "Failed to create directories " << log_dir << ": " << e.what();
  //     return -1;
  //   }
  // }
  // FLAGS_log_dir = log_dir;
  // AINFO << "Planning log dir: " << FLAGS_log_dir;

  // planning
  //  planning_ros;
  // if (!planning_ros.Init()) {
  //   AERROR << "Failed to init Planning module!";

  //   return -1;
  // }

  // planning_ros.Proc();

  return 0;
}
