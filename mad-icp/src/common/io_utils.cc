//
// Created by xiang on 2021/7/20.
//
#include "common/io_utils.h"

#include <glog/logging.h>

namespace zjloc
{

    void RosbagIO::Go()
    {
        rosbag::Bag bag(bag_file_);
        LOG(INFO) << "running in " << bag_file_ << ", reg process func: " << process_func_.size();

        if (!bag.isOpen())
        {
            LOG(ERROR) << "cannot open " << bag_file_;
            return;
        }

        // auto view = rosbag::View(bag);
        rosbag::View view(bag);

        auto start_real_time = std::chrono::high_resolution_clock::now();
        auto start_sim_time = view.getBeginTime();

        // auto prev_real_time = start_real_time;
        // auto prev_sim_time = start_sim_time;

        for (const rosbag::MessageInstance &m : view)
        {
            auto iter = process_func_.find(m.getTopic());
            if (iter != process_func_.end())
            {
                iter->second(m);
            }

            if (global::FLAG_EXIT)
            {
                break;
            }

            // auto real_time = std::chrono::high_resolution_clock::now();
            // if (real_time - prev_real_time > std::chrono::seconds(5))
            // {
            //     auto sim_time = m.getTime();
            //     auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time - prev_real_time).count() * 0.001;
            //     auto delta_sim = (sim_time - prev_sim_time).toSec();
            //     printf("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);
            //     prev_sim_time = sim_time;
            //     prev_real_time = real_time;
            // }
        }

        bag.close();

        auto real_time = std::chrono::high_resolution_clock::now();
        auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time - start_real_time).count() * 0.001;
        auto delta_sim = (view.getEndTime() - start_sim_time).toSec();
        printf("Entire rosbag processed at %.1fX speed", delta_sim / delta_real);

        LOG(INFO) << "bag " << bag_file_ << " finished.";
    }

} // namespace zjloc