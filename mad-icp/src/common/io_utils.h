//
// Created by xiang on 2021/7/20.
//

#ifndef SLAM_IN_AUTO_DRIVING_IO_UTILS_H
#define SLAM_IN_AUTO_DRIVING_IO_UTILS_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <functional>
#include <utility>

#include <common/global_flags.h>
#include "common/message_def.h"

#include <chrono>

namespace zjloc
{

    /**
     * ROSBAG IO
     * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
     */
    class RosbagIO
    {
    public:
        explicit RosbagIO(std::string bag_file) : bag_file_(std::move(bag_file)) {}

        using MessageProcessFunction = std::function<bool(const rosbag::MessageInstance &m)>;

        /// 一些方便直接使用的topics, messages
        using PointCloud2Handle = std::function<bool(sensor_msgs::PointCloud2::Ptr)>;

        // 遍历文件内容，调用回调函数
        void Go();

        /// 通用处理函数
        RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func)
        {
            process_func_.emplace(topic_name, func);
            return *this;
        }

        /// point cloud2 的处理
        RosbagIO &AddPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f)
        {
            return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) -> bool
                             {
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (msg == nullptr) {
                return false;
            }
            return f(msg); });
        }

        /// 清除现有的处理函数
        void CleanProcessFunc() { process_func_.clear(); }

    private:
        std::map<std::string, MessageProcessFunction> process_func_;
        std::string bag_file_;
    };

} // namespace zjloc

#endif // SLAM_IN_AUTO_DRIVING_IO_UTILS_H
