/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Created Date: 2020-02-10 08:38:42
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-12-03 17:12:57
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/data_pretreat/data_pretreat_flow.hpp"

#include <glog/logging.h>

#include "mapping_localization/global_defination/global_defination.h"
#include "mapping_localization/mapping/global_param/global_param.hpp"
#include "mapping_localization/tools/tic_toc.hpp"

namespace mapping_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic) {
    std::string data_pretreat_config_file_path = WORK_SPACE_PATH + "/config/mapping/data_pretreat.yaml";
    YAML::Node data_pretreat_config_node = YAML::LoadFile(data_pretreat_config_file_path);

    std::string pointcloud_topic = data_pretreat_config_node["pointcloud_topic"].as<std::string>();
    std::string imu_topic = data_pretreat_config_node["imu_topic"].as<std::string>();
    std::string velocity_topic = data_pretreat_config_node["velocity_topic"].as<std::string>();
    std::string gnss_topic = data_pretreat_config_node["gnss_topic"].as<std::string>();

    use_external_laser_odom_ = data_pretreat_config_node["use_external_front_end"].as<bool>();
    cloud_data_buff_min_size_ = data_pretreat_config_node["lidar_buffer_size"].as<unsigned int>();

    undistort_pointcloud_ = data_pretreat_config_node["undistort_pointcloud"].as<bool>();

    std::string global_config_file_path = WORK_SPACE_PATH + "/config/mapping/global.yaml";
    YAML::Node global_config_node = YAML::LoadFile(global_config_file_path);

    GlobalParam gp(global_config_node);

    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, pointcloud_topic, 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, imu_topic, 1000000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, velocity_topic, 1000000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, gnss_topic, 1000000);
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, gp.imu_frame_id, gp.lidar_frame_id);

    // publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, gp.synced_pointcloud_topic, gp.lidar_frame_id, 10);
    gnss_pub_ptr_ =
        std::make_shared<OdometryPublisher>(nh, gp.synced_gnss_topic, gp.global_frame_id, gp.vehicle_ref_frame_id, 10);

    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();

    if (use_external_laser_odom_) {
        std::string external_laser_odom_topic =
            data_pretreat_config_node["external_laser_odom_topic"].as<std::string>();
        std::string laser_odom_topic = gp.lidar_odometry_topic;
        std::shared_ptr<OdometryPublisher> synced_laser_odom_pub_ptr =
            std::make_shared<OdometryPublisher>(nh, laser_odom_topic, gp.global_frame_id, gp.lidar_odom_frame_id, 10);
        std::shared_ptr<OdometrySubscriber> external_laser_odom_sub_ptr =
            std::make_shared<OdometrySubscriber>(nh, external_laser_odom_topic, 100000000);

        std::vector<float> pose_to_pointcloud =
            data_pretreat_config_node["pose_to_pointcloud"].as<std::vector<float>>();
        external_front_end_ptr_ = std::make_shared<ExternalFrontEndAdapter>(
            external_laser_odom_sub_ptr, synced_laser_odom_pub_ptr, pose_to_pointcloud);
    }
}

bool DataPretreatFlow::Run() {
    if (!ReadData()) return false;

    if (!InitCalibration()) return false;

    if (!InitGNSS()) return false;

    while (HasData()) {
        if (!ValidData()) continue;

        TransformData();
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_;

    // 从 subscribers 的缓存中读取数据
    imu_sub_ptr_->ParseData(unsynced_imu_);
    velocity_sub_ptr_->ParseData(unsynced_velocity_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if (cloud_data_buff_.size() == 0) return false;

    double cloud_time = cloud_data_buff_.front().time;
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu || !valid_velocity || !valid_gnss) {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

bool DataPretreatFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() < cloud_data_buff_min_size_) return false;
    if (cloud_data_buff_.size() == 0) return false;
    if (imu_data_buff_.size() == 0) return false;
    if (velocity_data_buff_.size() == 0) return false;
    if (gnss_data_buff_.size() == 0) return false;

    return true;
}

bool DataPretreatFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;

    // 点云过旧
    if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (use_external_laser_odom_) {
        if (!external_front_end_ptr_->SyncData(current_cloud_data_.time)) {
            cloud_data_buff_.pop_front();
            return false;
        }
    }

    // 其他传感器数据过旧
    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }

    if (diff_velocity_time > 0.05) {
        velocity_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::TransformData() {
    gnss_pose_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0, 3) = current_gnss_data_.local_E;
    gnss_pose_(1, 3) = current_gnss_data_.local_N;
    gnss_pose_(2, 3) = current_gnss_data_.local_U;
    gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();

    if (undistort_pointcloud_)
    {
        // current_velocity_data_.TransformCoordinate(lidar_to_imu_);
        current_velocity_data_.TransformCoordinate(lidar_to_imu_.inverse());
        distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
        distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);
    }

    return true;
}

bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time);

    if (use_external_laser_odom_) {
        external_front_end_ptr_->PublishData();
    }

    return true;
}
}  // namespace mapping_localization