/*
 * @Description:
 * @Created Date: 2021-11-30 15:24:59
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-02 00:53:29
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/mapping/global_param/global_param.hpp"

namespace mapping_localization {

GlobalParam::GlobalParam(const YAML::Node& node) { loadGlobalParamFromYaml(node); }

bool GlobalParam::loadGlobalParamFromYaml(const YAML::Node& global_config_node) {
    data_path = global_config_node["data_path"].as<std::string>();

    global_frame_id = global_config_node["global_frame_id"].as<std::string>();
    lidar_global_frame_id = global_config_node["lidar_global_frame_id"].as<std::string>();

    // imu_frame_id = global_config_node["imu_frame_id"].as<std::string>();
    // lidar_frame_id = global_config_node["lidar_frame_id"].as<std::string>();

    vehicle_ref_frame_id = global_config_node["vehicle_ref_frame_id"].as<std::string>();
    lidar_odom_frame_id = global_config_node["lidar_odom_frame_id"].as<std::string>();
    vehicle_odom_frame_id = global_config_node["vehicle_odom_frame_id"].as<std::string>();
    vehicle_optimized_frame_id = global_config_node["vehicle_optimized_frame_id"].as<std::string>();
    vehicle_corrected_frame_id = global_config_node["vehicle_corrected_frame_id"].as<std::string>();

    // pointcloud_topic = global_config_node["pointcloud_topic"].as<std::string>();
    // imu_topic = global_config_node["imu_topic"].as<std::string>();
    // velocity_topic = global_config_node["velocity_topic"].as<std::string>();
    // gnss_topic = global_config_node["gnss_topic"].as<std::string>();

    synced_pointcloud_topic = global_config_node["synced_pointcloud_topic"].as<std::string>();
    synced_gnss_topic = global_config_node["synced_gnss_topic"].as<std::string>();

    lidar_odometry_topic = global_config_node["lidar_odometry_topic"].as<std::string>();
    vehicle_odometry_topic = global_config_node["vehicle_odometry_topic"].as<std::string>();
    vehicle_optimized_odometry_topic = global_config_node["vehicle_optimized_odometry_topic"].as<std::string>();

    gnss_trajectory_topic = global_config_node["gnss_trajectory_topic"].as<std::string>();
    lidar_odometry_trajectory_topic = global_config_node["lidar_odometry_trajectory_topic"].as<std::string>();

    vehicle_odometry_trajectory_topic = global_config_node["vehicle_odometry_trajectory_topic"].as<std::string>();
    vehicle_optimized_trajectory_topic = global_config_node["vehicle_optimized_trajectory_topic"].as<std::string>();
    vehicle_corrected_trajectory_topic = global_config_node["vehicle_corrected_trajectory_topic"].as<std::string>();

    global_map_topic = global_config_node["global_map_topic"].as<std::string>();
    local_map_topic = global_config_node["local_map_topic"].as<std::string>();
    current_scan_topic = global_config_node["current_scan_topic"].as<std::string>();

    loop_pose_topic = global_config_node["loop_pose_topic"].as<std::string>();
    key_frame_topic = global_config_node["key_frame_topic"].as<std::string>();
    key_frame_gnss_topic = global_config_node["key_frame_gnss_topic"].as<std::string>();
    key_frame_optimized_topic = global_config_node["key_frame_optimized_topic"].as<std::string>();

    return true;
}
}  // namespace mapping_localization
