/*
 * @Description:
 * @Created Date: 2020-02-10 08:38:42
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-30 18:45:15
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/mapping/viewer/viewer_flow.hpp"

#include <glog/logging.h>

#include "mapping_localization/global_defination/global_defination.h"
#include "mapping_localization/mapping/global_param/global_param.hpp"
#include "mapping_localization/tools/tic_toc.hpp"

namespace mapping_localization {
ViewerFlow::ViewerFlow(ros::NodeHandle& nh, std::string cloud_topic) {
    std::string global_config_file_path = WORK_SPACE_PATH + "/config/mapping/global.yaml";
    std::string viewer_config_file_path = WORK_SPACE_PATH + "/config/mapping/viewer.yaml";

    YAML::Node global_config_node = YAML::LoadFile(global_config_file_path);
    YAML::Node viewer_config_node = YAML::LoadFile(viewer_config_file_path);

    GlobalParam gp(global_config_node);

    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/test", 100000);
    key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, gp.key_frame_topic, 100000);
    key_gnss_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, gp.key_frame_gnss_topic, 100000);
    transformed_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, gp.vehicle_odometry_topic, 100000);

    optimized_key_frames_sub_ptr_ =
        std::make_shared<TrajectorySubscriber>(nh, gp.vehicle_optimized_trajectory_topic, 100000);

    // publisher
    optimized_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
        nh, gp.vehicle_optimized_odometry_topic, gp.global_frame_id, gp.vehicle_optimized_frame_id, 100);
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, gp.current_scan_topic, gp.global_frame_id, 10);
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, gp.global_map_topic, gp.global_frame_id, 10);
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, gp.local_map_topic, gp.global_frame_id, 10);
    // vehicle_tracjectory_pub_ptr_ =
    //     std::make_shared<TrajectoryPublisher>(nh, gp.vehicle_odometry_trajectory_topic, gp.global_frame_id, 10);
    // corrected_tracjectory_pub_ptr_ =
    //     std::make_shared<TrajectoryPublisher>(nh, gp.vehicle_corrected_trajectory_topic, gp.global_frame_id, 10);

    // viewer
    viewer_ptr_ = std::make_shared<Viewer>(global_config_node, viewer_config_node);
}

bool ViewerFlow::Run() {
    if (!ReadData()) return false;

    while (HasData()) {
        if (ValidData()) {
            viewer_ptr_->UpdateWithNewKeyFrame(key_frame_buff_, current_transformed_odom_, current_cloud_data_);
            PublishLocalData();
        }
    }

    if (optimized_key_frames_.size() > 0) {
        viewer_ptr_->UpdateWithOptimizedKeyFrames(optimized_key_frames_);
        PublishGlobalData();
    }

    return true;
}

bool ViewerFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    transformed_odom_sub_ptr_->ParseData(transformed_odom_buff_);
    key_frame_sub_ptr_->ParseData(key_frame_buff_);
    key_gnss_sub_ptr_->ParseData(key_gnss_buff_);
    optimized_key_frames_sub_ptr_->ParseData(optimized_key_frames_);

    return true;
}

bool ViewerFlow::HasData() {
    if (cloud_data_buff_.size() == 0) return false;
    if (transformed_odom_buff_.size() == 0) return false;

    return true;
}

bool ViewerFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_transformed_odom_ = transformed_odom_buff_.front();

    double diff_odom_time = current_cloud_data_.time - current_transformed_odom_.time;

    if (diff_odom_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_odom_time > 0.05) {
        transformed_odom_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    transformed_odom_buff_.pop_front();

    return true;
}

bool ViewerFlow::PublishGlobalData() {
    if (viewer_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
        CloudData::Cloud_Ptr cloud_ptr(new CloudData::Cloud());
        viewer_ptr_->GetGlobalMap(cloud_ptr);
        global_map_pub_ptr_->Publish(cloud_ptr);
    }

    return true;
}

bool ViewerFlow::PublishLocalData() {
    optimized_odom_pub_ptr_->Publish(viewer_ptr_->GetCurrentPose());
    current_scan_pub_ptr_->Publish(viewer_ptr_->GetCurrentScan());

    if (viewer_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers()) {
        CloudData::Cloud_Ptr cloud_ptr(new CloudData::Cloud());
        viewer_ptr_->GetLocalMap(cloud_ptr);
        local_map_pub_ptr_->Publish(cloud_ptr);
    }

    return true;
}

bool ViewerFlow::SaveMap() { return viewer_ptr_->SaveMap(); }
}  // namespace mapping_localization