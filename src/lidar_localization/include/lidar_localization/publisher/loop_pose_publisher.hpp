/*
 * @Description: 发送闭环检测的相对位姿
 * @Created Date: 2020-02-06 21:05:47
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:27:02
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_PUBLISHER_LOOP_POSE_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_LOOP_POSE_PUBLISHER_HPP_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <string>

#include "lidar_localization/sensor_data/loop_pose.hpp"

namespace lidar_localization {
class LoopPosePublisher {
public:
    LoopPosePublisher(ros::NodeHandle& nh,
                      std::string topic_name,
                      std::string frame_id,
                      int buff_size);
    LoopPosePublisher() = default;

    void Publish(LoopPose& loop_pose);

    bool HasSubscribers();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}  // namespace lidar_localization
#endif