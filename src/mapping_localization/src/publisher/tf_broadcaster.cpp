/*
 * @Description: 发布tf的类
 * @Created Date: 2020-03-05 15:23:26
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-12-12 16:07:18
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/publisher/tf_broadcaster.hpp"

namespace mapping_localization {
TFBroadCaster::TFBroadCaster(std::string frame_id, std::string child_frame_id) {
    transform_.frame_id_ = frame_id;
    transform_.child_frame_id_ = child_frame_id;
}

void TFBroadCaster::SendTransform(Eigen::Matrix4f pose, double time) {
    Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
    ros::Time ros_time(time);
    transform_.stamp_ = ros_time;
    transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
    broadcaster_.sendTransform(transform_);
}
}  // namespace mapping_localization