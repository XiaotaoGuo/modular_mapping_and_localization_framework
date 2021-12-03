/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Created Date: 2020-02-10 08:38:42
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-12-03 17:08:57
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/mapping/back_end/back_end_flow.hpp"

#include <glog/logging.h>

#include "mapping_localization/mapping/global_param/global_param.hpp"

#include "mapping_localization/global_defination/global_defination.h"
#include "mapping_localization/tools/file_manager.hpp"
#include "mapping_localization/tools/tic_toc.hpp"

namespace mapping_localization {
BackEndFlow::BackEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic) {
    std::string global_config_file_path = WORK_SPACE_PATH + "/config/mapping/global.yaml";
    std::string back_end_config_file_path = WORK_SPACE_PATH + "/config/mapping/back_end.yaml";

    YAML::Node global_config_node = YAML::LoadFile(global_config_file_path);
    YAML::Node back_end_config_node = YAML::LoadFile(back_end_config_file_path);

    GlobalParam gp(global_config_node);

    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, gp.synced_pointcloud_topic, 100000);
    gnss_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, gp.synced_gnss_topic, 100000);
    laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, gp.lidar_odometry_topic, 100000);
    loop_pose_sub_ptr_ = std::make_shared<LoopPoseSubscriber>(nh, gp.loop_pose_topic, 100000);

    transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
        nh, gp.vehicle_odometry_topic, gp.global_frame_id, gp.vehicle_odom_frame_id, 10);
    key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, gp.key_frame_topic, gp.global_frame_id, 100);
    key_gnss_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, gp.key_frame_gnss_topic, gp.global_frame_id, 10);

    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, gp.imu_frame_id, gp.lidar_frame_id);

    optimized_tracjectory_pub_ptr_ =
        std::make_shared<TrajectoryPublisher>(nh, gp.vehicle_optimized_trajectory_topic, gp.global_frame_id, 10);
    key_gnss_trajectory_pub_ptr_ =
        std::make_shared<TrajectoryPublisher>(nh, gp.gnss_trajectory_topic, gp.global_frame_id, 10);
    corrected_trajectory_pub_ptr_ =
        std::make_shared<TrajectoryPublisher>(nh, gp.vehicle_corrected_trajectory_topic, gp.global_frame_id, 10);
    vehicle_tracjectory_pub_ptr_ =
        std::make_shared<TrajectoryPublisher>(nh, gp.vehicle_odometry_trajectory_topic, gp.global_frame_id, 10, true);

    back_end_ptr_ = std::make_shared<BackEnd>(global_config_node, back_end_config_node);
}

bool BackEndFlow::Run() {
    if (!ReadData()) return false;

    MaybeInsertLoopPose();

    while (HasData()) {
        if (!ValidData()) continue;

        UpdateBackEnd();

        PublishData();
    }

    return true;
}

bool BackEndFlow::ForceOptimize() {
    back_end_ptr_->ForceOptimize();
    if (back_end_ptr_->HasNewOptimized()) {
        std::deque<KeyFrame> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        optimized_tracjectory_pub_ptr_->Publish(optimized_key_frames);
        corrected_trajectory_pub_ptr_->Publish(optimized_key_frames);
    }
    return true;
}

bool BackEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);
    laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);
    loop_pose_sub_ptr_->ParseData(loop_pose_data_buff_);

    return true;
}

bool BackEndFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool BackEndFlow::MaybeInsertLoopPose() {
    while (loop_pose_data_buff_.size() > 0) {
        LoopPose loop_pose = loop_pose_data_buff_.front();
        if (loop_pose_data_buff_.front().confidence) {
            back_end_ptr_->InsertLoopPose(loop_pose);
        }
        vehicle_tracjectory_pub_ptr_->AddEdge(loop_pose.index0, loop_pose.index1, loop_pose.confidence);
        loop_pose_data_buff_.pop_front();
    }
    return true;
}

bool BackEndFlow::HasData() {
    if (cloud_data_buff_.size() == 0) return false;
    if (gnss_pose_data_buff_.size() == 0) return false;
    if (laser_odom_data_buff_.size() == 0) return false;

    return true;
}

bool BackEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_gnss_pose_data_ = gnss_pose_data_buff_.front();
    current_laser_odom_data_ = laser_odom_data_buff_.front();

    double diff_gnss_time = current_cloud_data_.time - current_gnss_pose_data_.time;
    double diff_laser_time = current_cloud_data_.time - current_laser_odom_data_.time;

    if (diff_gnss_time < -0.05 || diff_laser_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_pose_data_buff_.pop_front();
        return false;
    }

    if (diff_laser_time > 0.05) {
        laser_odom_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_pose_data_buff_.pop_front();
    laser_odom_data_buff_.pop_front();

    return true;
}

bool BackEndFlow::UpdateBackEnd() {
    static bool odometry_inited = false;
    static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

    // 将激光雷达相对于起始点的位姿转为 IMU 相对于起始点的位姿
    // Convert T_lidar_to_lidar_init -> T_IMU_to_IMU_init
    if (!InitCalibration()) return false;
    current_laser_odom_data_.pose = lidar_to_imu_ * current_laser_odom_data_.pose * lidar_to_imu_.inverse();

    // 初始化：获取 IMU 坐标系相对于世界（东北天）坐标系的姿态，用于与重力对齐
    // Get attitude between imu_init and world frame
    if (!odometry_inited) {
        odometry_inited = true;
        odom_init_pose = current_gnss_pose_data_.pose;
    }
    current_laser_odom_data_.pose = odom_init_pose * current_laser_odom_data_.pose;

    return back_end_ptr_->Update(current_cloud_data_, current_laser_odom_data_, current_gnss_pose_data_);
}

bool BackEndFlow::PublishData() {
    transformed_odom_pub_ptr_->Publish(current_laser_odom_data_.pose, current_laser_odom_data_.time);

    if (back_end_ptr_->HasNewKeyFrame()) {
        KeyFrame key_frame;

        back_end_ptr_->GetLatestKeyFrame(key_frame);
        key_frame_pub_ptr_->Publish(key_frame);
        vehicle_tracjectory_pub_ptr_->Publish(key_frame);

        back_end_ptr_->GetLatestKeyGNSS(key_frame);
        key_gnss_pub_ptr_->Publish(key_frame);
        key_gnss_trajectory_pub_ptr_->Publish(key_frame);

        if (!back_end_ptr_->HasNewOptimized()) {
            back_end_ptr_->GetLatestCorrectedKeyFrame(key_frame);
            corrected_trajectory_pub_ptr_->Publish(key_frame);
        }
    }

    if (back_end_ptr_->HasNewOptimized()) {
        std::deque<KeyFrame> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        optimized_tracjectory_pub_ptr_->Publish(optimized_key_frames);
        corrected_trajectory_pub_ptr_->Publish(optimized_key_frames);
    }

    return true;
}
}  // namespace mapping_localization