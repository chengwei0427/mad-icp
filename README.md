
# MAD-ICP Ros Version

This is a Ros Version Mad-ICP

<div align="center">
    <a href="./demo.mp4" target="_blank">
    <img src="./demo_pic.png" width=50% />
</div>

## New Features
1. ROS support,with a simple viewer
2. multi-type lidar support(velodyne,ouster,robosense,and pandar)

#  -------------------------------------------------------------
# MAD-ICP

## It Is All About Matching Data -- Robust and Informed LiDAR Odometry

A minimal, robust, accurate, and real-time LiDAR odometry.
<div align="center">
    <img src="mad-icp.gif" width="720"/>
</div>
This version is mainly for reviewers. Soon, you can install this system via `pip` and have a proper viewer. Our <a href="paper_with_supplementary.pdf">preprint</a> is available for more details and results.


## Building ##

Building has been tested on Ubuntu 20.04 (with g++).

The following external dependencies are required.

| Dependency   | Version(s) known to work |
| ------------ | ------------------------ |
| [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) | 3.3 |
| [OpenMP](https://www.openmp.org/) |  |
| [pybind11](https://pybind11.readthedocs.io/en/stable/) |  |

After obtaining all dependencies, the application can be built with CMake, for example, as follows:

```bash
cd mad-icp
mkdir build && cd build && cmake .. && make -j
```

## Running

A few other Python packages need to be installed for running, namely: `numpy,` `pyyaml`, and `rosbags.` You can find the specific versions in the `requirements.txt`. We suggest to create a virtual env and run `pip3 install -r requirements.txt`.

We provide a Python launcher for both rosbags and bin formats (we are currently working on bin format luncher; for this reason, you do not find configurations for KITTI and Mulran). The configuration file is important for the sensor characteristics and extrinsic information (most of the time, ground truths are not in the LiDAR frame). The internal parameters are in `configurations/params.cfg`; all the experiments have been run with this same set.

How to run (rosbag), make sure `estimate_path` and `data_path` point to a folder:
```bash
cd apps
python3 rosbag_runner.py --data_path /path_to_bag_folder/ --estimate_path /path_to_estimate_folder/ --dataset_config ../configurations/datasets/dataset_config_file --mad_icp_config ../configurations/params.cfg --num_cores 4 --num_keyframes 4 --realtime
```

Our runner directly saves the odometry estimate file in KITTI format (homogenous matrix row-major 12 components); in the near future, we will provide more available formats like TUM. 
Our pipeline is anytime realtime, therefore you can play with parameters `num_keyframes` and `num_cores`, if you have enough computation we suggest to increase these (we run demo with 16 and 16), if not you can leave it in the proposed way.
