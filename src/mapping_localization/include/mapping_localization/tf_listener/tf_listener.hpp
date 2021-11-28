/*
 * @Description: tf监听模块
 * @Created Date: 2020-02-06 16:01:21
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_TF_LISTENER_HPP_
#define MAPPING_LOCALIZATION_TF_LISTENER_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>
#include <string>

namespace mapping_localization {
class TFListener {
public:
    TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
    TFListener() = default;

    bool LookupData(Eigen::Matrix4f& transform_matrix);

private:
    bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);

private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;
};
}  // namespace mapping_localization

#endif