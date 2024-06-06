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

    void CloudConvert::Process(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<Eigen::Vector3d> &cloud_out)
    {
        switch (param_.lidar_type)
        {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::ROBOSENSE16:
            RobosenseHandler(msg);
            break;

            // case LidarType::PANDAR:
            //     PandarHandler(msg);
            //     break;

        default:
            LOG(ERROR) << "Error LiDAR Type: " << int(lidar_type_);
            break;
        }

        cloud_out = cloud_vec;
        // ts_out = ts_vec;
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
            cloud_vec.clear();
            ts_vec.clear();
        }

        pcl::PointCloud<ouster_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();

        cloud_vec.reserve(plsize);
        ts_vec.reserve(plsize); // FIXME: 存储为相对时间，在(0,1)之间

        // static int idx = 0;
        // static std::stringstream os;
        // os.str("");
        // os << "/home/cc/catkin_context/src/mad_icp/log/" << idx;
        // std::ofstream bin_file(os.str(), std::ios::out | std::ios::binary | std::ios::app);

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

            // bin_file.write((char *)&pl_orig.points[i].x, 3 * sizeof(float));
            // bin_file.write((char *)&pl_orig.points[i].intensity, sizeof(float));

            cloud_vec.push_back(Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z));
            ts_vec.push_back(pl_orig.points[i].t / timespan_);
        }
        // bin_file.close();
        // idx++;
    }

    void CloudConvert::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (cloud_vec.size() > 0)
        {
            cloud_vec.clear();
            ts_vec.clear();
        }

        pcl::PointCloud<velodyne_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();

        cloud_vec.reserve(plsize);
        ts_vec.reserve(plsize);

        double headertime = msg->header.stamp.toSec(); //    秒

        static double tm_scale = 1; //   1e6 - nclt kaist or 1

        timespan_ = pl_orig.points.back().time / tm_scale;

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

            cloud_vec.push_back(Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z));
            ts_vec.push_back(pl_orig.points[i].time / timespan_);
        }
    }

    void CloudConvert::RobosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (cloud_vec.size() > 0)
        {
            cloud_vec.clear();
            ts_vec.clear();
        }

        pcl::PointCloud<robosense_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();

        cloud_vec.reserve(plsize);
        ts_vec.reserve(plsize);

        double headertime = msg->header.stamp.toSec();

        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

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

            cloud_vec.push_back(Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z));
            ts_vec.push_back((pl_orig.points[i].timestamp - pl_orig.points[0].timestamp) / timespan_);
        }
    }

    void CloudConvert::PandarHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (cloud_vec.size() > 0)
        {
            cloud_vec.clear();
            ts_vec.clear();
        }

        pcl::PointCloud<pandar_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();

        cloud_vec.reserve(plsize);
        ts_vec.reserve(plsize);

        double headertime = msg->header.stamp.toSec();

        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

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

            cloud_vec.push_back(Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z));
            ts_vec.push_back((pl_orig.points[i].timestamp - pl_orig.points[0].timestamp) / timespan_);
        }
    }

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
