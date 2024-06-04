#include "cloud_convert2.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
// #include <execution>

namespace mad_icp
{

    // void CloudConvert::Process(const livox_ros_driver::CustomMsg::ConstPtr &msg,
    //                            std::vector<Eigen::Vector3d> &cloud_out, std::vector<double> &ts_out)
    // {
    //     AviaHandler(msg);
    //     cloud_out = cloud_vec;
    //     ts_out = ts_vec;
    // }

    void CloudConvert::Process(const sensor_msgs::PointCloud2::ConstPtr &msg,
                               std::vector<Eigen::Vector3d> &cloud_out, std::vector<double> &ts_out)
    {
        switch (param_.lidar_type)
        {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

            // case LidarType::VELO32:
            //     VelodyneHandler(msg);
            //     break;

            // case LidarType::ROBOSENSE16:
            //     RobosenseHandler(msg);
            //     break;

            // case LidarType::PANDAR:
            //     PandarHandler(msg);
            //     break;

        default:
            LOG(ERROR) << "Error LiDAR Type: " << int(lidar_type_);
            break;
        }
        cloud_out = cloud_vec;
        ts_out = ts_vec;
    }

    // void CloudConvert::AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    // {
    //     std::vector<Eigen::Vector3d>().swap(cloud_vec);
    //     std::vector<double>().swap(ts_vec);

    //     scan->timestamp = msg->header.stamp.toNSec();
    //     scan->size = msg->point_num;
    //     scan->points.resize(scan->size);
    //     static double tm_scale = 1e-9;
    //     for (size_t j = 0; j < scan->size; ++j)
    //     {
    //         scan->points[j].x = msg->points[j].x;
    //         scan->points[j].y = msg->points[j].y;
    //         scan->points[j].z = msg->points[j].z;
    //         scan->points[j].intensity = msg->points[j].reflectivity;
    //         scan->points[j].ts =
    //             (double)(scan->timestamp + msg->points[j].offset_time) * tm_scale;
    //     }
    // }

    void CloudConvert::Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (cloud_vec.size() > 0)
        {
            std::vector<Eigen::Vector3d>().swap(cloud_vec);
            std::vector<double>().swap(ts_vec);
        }

        pcl::PointCloud<ouster_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();

        cloud_vec.reserve(plsize);
        ts_vec.reserve(plsize);

        static double tm_scale = 1e9;

        double headertime = msg->header.stamp.toSec();
        timespan_ = pl_orig.points.back().t / tm_scale;
        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].t / tm_scale
        //           << " , 100: " << pl_orig.points[100].t / tm_scale
        //           << std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % param_.point_filter_num != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < param_.blind * param_.blind)
                continue;

            cloud_vec.push_back(Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z));
            ts_vec.push_back(pl_orig.points[i].t / timespan_);
        }
    }

    /*void CloudConvert::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        scan.reset(new Scan);
        scan->timestamp = msg->header.stamp.toNSec(); // ns

        pcl::PointCloud<velodyne_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        scan->points.resize(plsize);

        double headertime = msg->header.stamp.toSec(); //    秒

        static double tm_scale = 1; //   1e6 - nclt kaist or 1

        //  FIXME:  nclt 及kaist时间戳大于0.1
        auto time_list_velodyne = [&](velodyne_ros::Point &point_1, velodyne_ros::Point &point_2)
        {
            return (point_1.time < point_2.time);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_velodyne);
        while (pl_orig.points[plsize - 1].time / tm_scale >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }

        int point_num = 0;
        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < param_.blind * param_.blind)
                continue;

            scan->points[point_num].x = pl_orig.points[i].x;
            scan->points[point_num].y = pl_orig.points[i].y;
            scan->points[point_num].z = pl_orig.points[i].z;
            scan->points[point_num].intensity = pl_orig.points[i].intensity;
            scan->points[point_num].ts = headertime + pl_orig.points[i].time / tm_scale;

            point_num++;
        }
        scan->size = scan->points.size();
    }

    void CloudConvert::RobosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        scan.reset(new Scan);
        scan->timestamp = msg->header.stamp.toNSec(); // ns

        pcl::PointCloud<robosense_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        scan->points.resize(plsize);

        double headertime = msg->header.stamp.toSec();
        //  FIXME:  时间戳大于0.1
        auto time_list_robosense = [&](robosense_ros::Point &point_1, robosense_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_robosense);
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }

        // std::cout << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;
        int point_num = 0;
        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < param_.blind * param_.blind)
                continue;

            scan->points[point_num].x = pl_orig.points[i].x;
            scan->points[point_num].y = pl_orig.points[i].y;
            scan->points[point_num].z = pl_orig.points[i].z;
            scan->points[point_num].intensity = pl_orig.points[i].intensity;
            scan->points[point_num].ts = pl_orig.points[i].timestamp;

            point_num++;
        }
        scan->size = scan->points.size();
    }


    void CloudConvert::PandarHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<pandar_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();

        static double tm_scale = 1; //   1e6

        auto time_list_pandar = [&](pandar_ros::Point &point_1, pandar_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_pandar);
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }
        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp
        //           << " , 100: " << pl_orig.points[100].timestamp - pl_orig.points[0].timestamp
        //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % param_.point_filter_num != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > 150 * 150 || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp;
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }*/

    void CloudConvert::initFromConfig(const CVTParam &param)
    {
        param_.point_filter_num = param.point_filter_num;
        param_.blind = param.blind;
        param_.lidar_type = param.lidar_type;
        lidar_type_ = param.lidar_type;
        if (param_.lidar_type == LidarType::AVIA)
            LOG(INFO) << "Using AVIA Lidar";
        else if (param_.lidar_type == LidarType::VELO32)
            LOG(INFO) << "Using Velodyne 32 Lidar";
        else if (param_.lidar_type == LidarType::OUST64)
            LOG(INFO) << "Using OUST 64 Lidar";
        else if (param_.lidar_type == LidarType::ROBOSENSE16)
            LOG(INFO) << "Using Robosense 16 LIdar";
        else if (param_.lidar_type == LidarType::PANDAR)
            LOG(INFO) << "Using Pandar LIdar";
        else
            LOG(WARNING) << "unknown lidar_type";
    }

    void CloudConvert::LoadFromYAML(const std::string &yaml_file)
    {
        auto yaml = YAML::LoadFile(yaml_file);
        int lidar_type = yaml["preprocess"]["lidar_type"].as<int>();

        param_.point_filter_num = yaml["preprocess"]["point_filter_num"].as<int>();
        param_.blind = yaml["preprocess"]["blind"].as<double>();

        if (lidar_type == 1)
        {
            lidar_type_ = LidarType::AVIA;
            LOG(INFO) << "Using AVIA Lidar";
        }
        else if (lidar_type == 2)
        {
            lidar_type_ = LidarType::VELO32;
            LOG(INFO) << "Using Velodyne 32 Lidar";
        }
        else if (lidar_type == 3)
        {
            lidar_type_ = LidarType::OUST64;
            LOG(INFO) << "Using OUST 64 Lidar";
        }
        else if (lidar_type == 4)
        {
            lidar_type_ = LidarType::ROBOSENSE16;
            LOG(INFO) << "Using Robosense 16 LIdar";
        }
        else if (lidar_type == 5)
        {
            lidar_type_ = LidarType::PANDAR;
            LOG(INFO) << "Using Pandar LIdar";
        }
        else
        {
            LOG(WARNING) << "unknown lidar_type";
        }
        param_.lidar_type = lidar_type_;
    }

} // namespace zjloc
