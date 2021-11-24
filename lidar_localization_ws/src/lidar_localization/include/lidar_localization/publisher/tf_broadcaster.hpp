/*
 * @Description: 发布tf的类
 * @Created Date: 2020-03-05 15:23:26
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:27:43
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_PUBLISHER_TF_BROADCASTER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_TF_BROADCASTER_HPP_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <string>

namespace lidar_localization {
class TFBroadCaster {
public:
    TFBroadCaster(std::string frame_id, std::string child_frame_id);
    TFBroadCaster() = default;
    void SendTransform(Eigen::Matrix4f pose, double time);

protected:
    tf::StampedTransform transform_;
    tf::TransformBroadcaster broadcaster_;
};
}  // namespace lidar_localization
#endif