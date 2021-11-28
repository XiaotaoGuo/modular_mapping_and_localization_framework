/*
 * @Description: 订阅odometry数据
 * @Created Date: 2019-08-19 19:22:17
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_
#define MAPPING_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <deque>
#include <mutex>
#include <thread>

#include "mapping_localization/sensor_data/pose_data.hpp"

namespace mapping_localization {
class OdometrySubscriber {
public:
    OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    OdometrySubscriber() = default;
    void ParseData(std::deque<PoseData>& deque_pose_data);

private:
    void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<PoseData> new_pose_data_;

    std::mutex buff_mutex_;
};
}  // namespace mapping_localization
#endif