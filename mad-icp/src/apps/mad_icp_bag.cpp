// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>

// ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <random>

#include <common/timer/timer.h>
#include "preprocess/cloud_convert2/cloud_convert2.h"
#include "mad_icp/odometry/pipeline.h"

#include "common/io_utils.h"

nav_msgs::Path laserOdoPath;

DEFINE_string(config_directory, "/home/cc/catkin_context/src/mad_icp/config", "配置文件目录");
#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "log/" + name))

Pipeline *lio;
mad_icp::CloudConvert *convert;
std::string config_file;

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::cout << ANSI_COLOR_GREEN << "config_file:" << FLAGS_config_directory << ANSI_COLOR_RESET << std::endl;
    config_file = std::string(FLAGS_config_directory) + "/mapping_bag.yaml";
    std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;

    auto yaml = YAML::LoadFile(config_file);
    std::string laser_topic = yaml["common"]["lid_topic"].as<std::string>();
    std::string bag_path_ = yaml["bag_path"].as<std::string>();

    double sensor_hz = yaml["preprocess"]["sensor_hz"].as<double>();
    bool deskew = yaml["preprocess"]["deskew"].as<bool>();
    double b_max = yaml["mad_icp"]["b_max"].as<double>();
    double rho_ker = yaml["mad_icp"]["rho_ker"].as<double>();
    double p_th = yaml["mad_icp"]["p_th"].as<double>();
    double b_min = yaml["mad_icp"]["b_min"].as<double>();
    double b_ratio = yaml["mad_icp"]["b_ratio"].as<double>();
    int num_keyframes = 4;
    int num_cores = 4;
    double realtime = true;

    convert = new mad_icp::CloudConvert;
    convert->LoadFromYAML(config_file);
    std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;

    lio = new Pipeline(sensor_hz, deskew, b_max, rho_ker, p_th,
                       b_min, b_ratio, num_keyframes, num_cores, realtime);

    std::cout << ANSI_COLOR_RED_BOLD << "process bag: " << bag_path_ << ANSI_COLOR_RESET << std::endl;

    zjloc::RosbagIO rosbag_io(bag_path_);
    rosbag_io
        .AddPointCloud2Handle(laser_topic,
                              [&](sensor_msgs::PointCloud2::Ptr msg) -> bool
                              {
                                  std::vector<Eigen::Vector3d> cloud_out;
                                  std::vector<double> ts_out;
                                  zjloc::common::Timer::Evaluate([&]()
                                                                 { convert->Process(msg, cloud_out, ts_out); },
                                                                 "laser convert");

                                  double cloud_stamp = msg->header.stamp.toSec();
                                  std::cout << "Loading frame # " << lio->currentID() << std::endl;
                                  lio->compute(cloud_stamp, cloud_out);
                                  Eigen::Matrix<double, 4, 4> lidar_to_world = lio->currentPose();
                              })
        .Go();

    std::cout << ANSI_COLOR_GREEN_BOLD << "rosbag_io out done. " << ANSI_COLOR_RESET << std::endl;

    zjloc::common::Timer::PrintAll();
    zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));

    cout << ANSI_COLOR_GREEN_BOLD << "push enter to quit" << ANSI_COLOR_RESET << endl;
    getchar();

    LOG(INFO) << "Finish done.";

    std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

    sleep(3);
    return 0;
}