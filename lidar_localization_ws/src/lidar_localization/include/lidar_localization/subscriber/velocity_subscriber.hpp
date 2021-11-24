/*
 * @Description: 订阅velocity数据
 * @Created Date: 2019-08-19 19:22:17
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:30:39
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <ros/ros.h>

#include <deque>
#include <mutex>
#include <thread>

#include "geometry_msgs/TwistStamped.h"
#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
class VelocitySubscriber {
public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData>& deque_velocity_data);

private:
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<VelocityData> new_velocity_data_;

    std::mutex buff_mutex_;
};
}  // namespace lidar_localization
#endif