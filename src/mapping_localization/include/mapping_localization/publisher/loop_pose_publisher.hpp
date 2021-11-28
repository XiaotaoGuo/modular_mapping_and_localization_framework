/*
 * @Description: 发送闭环检测的相对位姿
 * @Created Date: 2020-02-06 21:05:47
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_PUBLISHER_LOOP_POSE_PUBLISHER_HPP_
#define MAPPING_LOCALIZATION_PUBLISHER_LOOP_POSE_PUBLISHER_HPP_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <string>

#include "mapping_localization/sensor_data/loop_pose.hpp"

namespace mapping_localization {
class LoopPosePublisher {
public:
    LoopPosePublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size);
    LoopPosePublisher() = default;

    void Publish(LoopPose& loop_pose);

    bool HasSubscribers();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}  // namespace mapping_localization
#endif