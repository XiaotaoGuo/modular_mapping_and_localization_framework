/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Created Date: 2020-02-10 08:31:22
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-30 17:43:09
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define MAPPING_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
// subscriber
#include "mapping_localization/subscriber/cloud_subscriber.hpp"
#include "mapping_localization/subscriber/gnss_subscriber.hpp"
#include "mapping_localization/subscriber/imu_subscriber.hpp"
#include "mapping_localization/subscriber/velocity_subscriber.hpp"
#include "mapping_localization/tf_listener/tf_listener.hpp"
// publisher
#include "mapping_localization/publisher/cloud_publisher.hpp"
#include "mapping_localization/publisher/odometry_publisher.hpp"
#include "mapping_localization/publisher/trajectory_publisher.hpp"
// models
#include "mapping_localization/models/scan_adjust/distortion_adjust.hpp"

namespace mapping_localization {
class DataPretreatFlow {
public:
    DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic);

    bool Run();

private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool PublishData();

private:
    // subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    // models
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<VelocityData> velocity_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;

    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    VelocityData current_velocity_data_;
    GNSSData current_gnss_data_;

    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};
}  // namespace mapping_localization

#endif