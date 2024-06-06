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

//  mad_icp
#include <filesystem>
#include "mad_icp/odometry/pipeline.h"
#include "mad_icp/tools/mad_tree.h"

#include "common/io_utils.h"

nav_msgs::Path laserOdoPath;

DEFINE_string(config_directory, "/home/cc/catkin_context/src/mad_icp/config", "配置文件目录");
#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "log/" + name))

mad_icp::CloudConvert *convert;
std::unique_ptr<Pipeline> lio = nullptr;
std::string config_file;

std::deque<std::pair<double, std::vector<Eigen::Vector3d>>> data_buffer_;

ros::Publisher pubLaserOdometry, pubLaserOdometryPath;
ros::Publisher pub_scan;

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

    std::vector<Eigen::Vector3d> cloud_out;
    std::vector<double> ts_out;
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_out); },
                                   "laser convert");
    cloud_out.shrink_to_fit();
    double cloud_stamp = msg->header.stamp.toSec();
    data_buffer_.push_back(std::make_pair(cloud_stamp, cloud_out));
}

void process()
{
    if (!data_buffer_.empty())
    {
        auto data = data_buffer_.front();
        double stamp = data.first;
        std::cout << "Loading frame # " << lio->currentID() << std::endl;
        zjloc::common::Timer::Evaluate([&]()
                                       { lio->compute(stamp, data.second); },
                                       "process measure");

        Eigen::Matrix<double, 4, 4> lidar_to_world = lio->currentPose();
        data_buffer_.pop_front();

        {

            zjloc::CloudPtr trans_cloud(new zjloc::PointCloudType());
            Eigen::Isometry3d T = Eigen::Isometry3d(lidar_to_world);
            {
                zjloc::PointType point;
                for (auto pt : data.second)
                {
                    Eigen::Vector3d pt_w = T * pt;
                    point.x = pt_w[0], point.y = pt_w[1], point.z = pt_w[2];
                    trans_cloud->push_back(point);
                }
                sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
                pcl::toROSMsg(*trans_cloud, *cloud_ptr_output);

                cloud_ptr_output->header.stamp = ros::Time().fromSec(stamp);
                cloud_ptr_output->header.frame_id = "map";
                pub_scan.publish(*cloud_ptr_output);
            }

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            Eigen::Quaterniond q_current(T.rotation());
            Eigen::Vector3d pose = T.translation();
            transform.setOrigin(tf::Vector3(pose[0], pose[1], pose[2]));
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            transform.setRotation(q);

            br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "map", "base_link"));

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "base_link";
            laserOdometry.header.stamp = ros::Time().fromSec(stamp);

            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = pose[0];
            laserOdometry.pose.pose.position.y = pose[1];
            laserOdometry.pose.pose.position.z = pose[2];
            pubLaserOdometry.publish(laserOdometry);

            //  publish path
            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserOdoPath.header.stamp = laserOdometry.header.stamp;
            laserOdoPath.poses.push_back(laserPose);
            laserOdoPath.header.frame_id = "/map";
            pubLaserOdometryPath.publish(laserOdoPath);
        }
    }
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

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

    lio = std::make_unique<Pipeline>(sensor_hz, deskew, b_max, rho_ker, p_th,
                                     b_min, b_ratio, num_keyframes, num_cores, realtime);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(laser_topic, 100, standard_pcl_cbk);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    pubLaserOdometryPath = nh.advertise<nav_msgs::Path>("/odometry_path", 5);

    pub_scan = nh.advertise<sensor_msgs::PointCloud2>("current_cloud", 10);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        process();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    zjloc::common::Timer::PrintAll();
    zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));

    std::cout << ANSI_COLOR_GREEN_BOLD << " Finish done. " << ANSI_COLOR_RESET << std::endl;

    sleep(3);
    return 0;
}