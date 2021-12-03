/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Created Date: 2020-02-10 08:38:42
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-12-03 15:10:23
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/mapping/front_end/front_end_flow.hpp"

#include <glog/logging.h>

#include "mapping_localization/global_defination/global_defination.h"
#include "mapping_localization/mapping/global_param/global_param.hpp"

#include "mapping_localization/tools/tic_toc.hpp"

namespace mapping_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    std::string global_config_file_path = WORK_SPACE_PATH + "/config/mapping/global.yaml";
    std::string front_end_config_file_path = WORK_SPACE_PATH + "/config/mapping/front_end.yaml";

    YAML::Node global_config_node = YAML::LoadFile(global_config_file_path);
    YAML::Node front_end_config_node = YAML::LoadFile(front_end_config_file_path);

    GlobalParam gp(global_config_node);

    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, gp.synced_pointcloud_topic, 100000);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
        nh, gp.lidar_odometry_topic, gp.lidar_init_frame_id, gp.lidar_odom_frame_id, 10);

    front_end_ptr_ = std::make_shared<FrontEnd>(global_config_node, front_end_config_node);
}

bool FrontEndFlow::Run() {
    if (!ReadData()) return false;

    while (HasData()) {
        if (!ValidData()) continue;

        if (UpdateLaserOdometry()) {
            PublishData();
        }
    }
    return true;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    return true;
}

bool FrontEndFlow::HasData() { return cloud_data_buff_.size() > 0; }

bool FrontEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
    static bool odometry_inited = false;
    if (!odometry_inited) {
        odometry_inited = true;
        front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }

    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool FrontEndFlow::PublishData() {
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);

    return true;
}
}  // namespace mapping_localization