/*
 * @Description:
 * @Created Date: 2021-12-02 00:46:57
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-03 17:36:23
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/data_pretreat/external_front_end_adapter.hpp"

namespace mapping_localization {

ExternalFrontEndAdapter::ExternalFrontEndAdapter(
    std::shared_ptr<OdometrySubscriber>& external_laser_odom_sub_ptr,
    std::shared_ptr<OdometryPublisher>& synced_external_lasesr_odom_pub_ptr,
    const std::vector<float>& pose_to_pointcloud)
    : external_laser_odom_sub_ptr_(external_laser_odom_sub_ptr),
      synced_external_lasesr_odom_pub_ptr_(synced_external_lasesr_odom_pub_ptr) {
    double tx = pose_to_pointcloud[0];
    double ty = pose_to_pointcloud[1];
    double tz = pose_to_pointcloud[2];
    double yaw = pose_to_pointcloud[3];
    double pitch = pose_to_pointcloud[4];
    double roll = pose_to_pointcloud[5];

    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;

    Eigen::Matrix3f rotationMatrix = q.matrix();

    pose_to_pointcloud_.block<3, 1>(0, 3) = Eigen::Vector3f(tx, ty, tz);
    pose_to_pointcloud_.block<3, 3>(0, 0) = rotationMatrix;
}

bool ExternalFrontEndAdapter::SyncData(double time) {
    external_laser_odom_sub_ptr_->ParseData(external_odom_buff_);

    while (!external_odom_buff_.empty() && external_odom_buff_.front().time < time) {
        external_odom_buff_.pop_front();
    }

    if (external_odom_buff_.empty() || external_odom_buff_.front().time != time) {
        return false;
    }

    return true;
}

bool ExternalFrontEndAdapter::HasData() const { return external_odom_buff_.size(); }

void ExternalFrontEndAdapter::PublishData() {
    PoseData pose = external_odom_buff_.front();
    external_odom_buff_.pop_front();
    // transform external odometry pose estitimation to lidar pose under lidar_init frame
    pose.pose = pose_to_pointcloud_.inverse() * pose.pose * pose_to_pointcloud_;
    synced_external_lasesr_odom_pub_ptr_->Publish(pose.pose, pose.time);
}

ExternalFrontEndAdapter::~ExternalFrontEndAdapter() {}
}  // namespace mapping_localization
