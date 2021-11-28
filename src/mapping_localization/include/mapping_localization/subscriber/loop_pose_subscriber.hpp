/*
 * @Description: 订阅 闭环检测位姿 数据
 * @Created Date: 2019-08-19 19:22:17
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_HPP_
#define MAPPING_LOCALIZATION_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_HPP_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <deque>
#include <mutex>
#include <thread>

#include "mapping_localization/sensor_data/loop_pose.hpp"

namespace mapping_localization {
class LoopPoseSubscriber {
public:
    LoopPoseSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    LoopPoseSubscriber() = default;
    void ParseData(std::deque<LoopPose>& loop_pose_buff);

private:
    void msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& loop_pose_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<LoopPose> new_loop_pose_;

    std::mutex buff_mutex_;
};
}  // namespace mapping_localization
#endif