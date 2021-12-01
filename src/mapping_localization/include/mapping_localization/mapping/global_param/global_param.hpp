/*
 * @Description:
 * @Created Date: 2021-11-30 14:55:23
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-11-30 17:29:51
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MAPPING_GLOBAL_PARAM_GLOBAL_PARAM_HPP_
#define MAPPING_LOCALIZATION_MAPPING_GLOBAL_PARAM_GLOBAL_PARAM_HPP_

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>

namespace test {
class TT {};
}  // namespace test

namespace mapping_localization {
class A {
public:
    int a = 1;
};
class GlobalParam {
public:
    std::string data_path;  // 数据存放路径

    //  世界坐标系 (world frame)
    // 参考东北天坐标系 （原点和初始时刻激光雷达中心重合）
    std::string global_frame_id;
    // 以激光雷达初始位姿为原点的全局坐标系（和参考东北天坐标系相差一个旋转角度）
    std::string lidar_global_frame_id;

    // 传感器坐标系（sensor frame），主要用于计算不同传感器之间的外参
    std::string imu_frame_id;
    std::string lidar_frame_id;

    // 载体坐标系 (body frame)
    // 参考载体坐标系---------载体在 reference_global_frame 下的的参考位姿
    std::string vehicle_ref_frame_id;
    // Lidar 坐标系----------载体在 lidar_global_frame 下的估计位姿 （前端里程计的输出结果）
    std::string lidar_odom_frame_id;
    // 估计载体坐标系---------载体在 reference_global_frame 下的估计位姿 (和东北天对齐)
    std::string vehicle_odom_frame_id;
    // 优化后的估计载体坐标系--载体在 reference_global_frame 下经过优化后的位姿
    std::string vehicle_optimized_frame_id;
    std::string vehicle_corrected_frame_id;

    // 原始消息 topic
    std::string pointcloud_topic = "";
    std::string imu_topic;
    std::string velocity_topic;
    std::string gnss_topic;

    // 同步后的消息 topic
    std::string synced_pointcloud_topic;
    std::string synced_gnss_topic;

    // 里程计消息：
    std::string lidar_odometry_topic;
    std::string vehicle_odometry_topic;
    std::string vehicle_optimized_odometry_topic;

    // 轨迹消息
    std::string gnss_trajectory_topic;
    std::string lidar_odometry_trajectory_topic;

    std::string vehicle_odometry_trajectory_topic;
    std::string vehicle_optimized_trajectory_topic;
    std::string vehicle_corrected_trajectory_topic;

    // 地图消息
    std::string global_map_topic;
    std::string local_map_topic;
    std::string current_scan_topic;

    //其他
    std::string loop_pose_topic;
    //回环消息
    std::string key_frame_topic;
    //关键帧对应的里程计估计位姿
    std::string key_frame_gnss_topic;
    //关键帧对应的 GNSS 观测位姿
    std::string key_frame_optimized_topic;
    //关键帧对应优化后的位姿
public:
    GlobalParam() = default;
    GlobalParam(const YAML::Node& node);

    bool loadGlobalParamFromYaml(const YAML::Node& global_config_node);
};

}  // namespace mapping_localization

#endif